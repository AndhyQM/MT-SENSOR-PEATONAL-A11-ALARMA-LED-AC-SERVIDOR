#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nimble_server.h"
#include "nvs_flash.h"
#include "a11_sensor.h"
#include "cJSON.h"
#include <math.h>
#include "freertos/semphr.h"

#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_BLUE    "\x1b[34m"
#define ANSI_MAGENTA "\x1b[35m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[0m"

#define TAG             "SP_SERVER"
#define NVS_NAMESPACE   "config"

#define ALARMA        GPIO_NUM_17
#define ROJO          GPIO_NUM_1
#define VERDE         GPIO_NUM_22

#define PIN_WATCHDOG    GPIO_NUM_19
#define WDG_INTERVAL_MS 2000         // pulso cada 2s

// Servicio 0 — App móvil
#define UUID_APP_SVC    "CC100010-0000-0000-0000-000000000000"
#define UUID_APP_TX     "CC100010-0000-0000-0000-000000000001"  // notify → app
#define UUID_APP_RX     "CC100010-0000-0000-0000-000000000002"  // write  ← app

// Servicio 1 — ESP32 clientes
#define UUID_ESP_SVC    "DD200020-0000-0000-0000-000000000000"
#define UUID_ESP_TX     "DD200020-0000-0000-0000-000000000001"  // notify → clientes
#define UUID_ESP_RX     "DD200020-0000-0000-0000-000000000002"  // write  ← clientes

// ── INDICES ───────────────────────────────
#define SVC_APP     0
#define SVC_ESP     1

// Chars servicio APP
#define APP_TX      0
#define APP_RX      1

// Chars servicio ESP
#define ESP_TX      0
#define ESP_RX      1

#define ON          1
#define OFF         0

static int contador_server = 0;
static int contador_cliente = 0;

static bool paso_libre    = true;

static TimerHandle_t timer_cruce    = NULL;
static TimerHandle_t timer_reinicio = NULL;

static SemaphoreHandle_t config_mutex = NULL;
static TaskHandle_t sensor_task_handle = NULL;

static volatile bool sensor_conectado = false;
static volatile bool persona_detectada = false;
static volatile bool hold_activo = false;
static volatile bool hold_new_person = false;
static volatile float last_dist_m = 0.0f;

static volatile bool flag_reinicio = false;

static volatile bool alarma_activa = false;
static volatile bool verde_activo  = true;
static volatile bool rojo_activo  = false;
static volatile bool sensor_cambio = false;

static volatile bool timers_cambio = false;

static volatile bool flag_apply_config     = false;
static volatile bool flag_send_config_esp  = false;


void config_sensor(void);
void reiniciar_sensor(void);

// ===================== PARÁMETROS TASK SENSOR A11 =====================
typedef struct {
    a11_sensor_handle_t sensor;
    gpio_num_t          output_pin;
    const char         *name;
    uint32_t            start_delay_ms;
} sensor_task_params_t;

static sensor_task_params_t p1;

// =====================================================================
//                          CONFIGURACION DE NVS
// =====================================================================
typedef struct {

    int tiempo_cruce_ms;
    int tiempo_reinicio_ms;

    bool state_alarma;
    bool state_rojo;
    bool state_verde;

    float threshold_m;
    int hold_time_ms;

} Config;

// Valores por defecto
static Config config = {

    .tiempo_cruce_ms        = 12000,
    .tiempo_reinicio_ms     = 20000,
    .state_alarma           = false,
    .state_rojo             = true,
    .state_verde            = true,

    .threshold_m            = 0.5f,
    .hold_time_ms           = 1500,
};

static void guardar_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        nvs_set_blob(nvs_handle, "config", &config, sizeof(Config));
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "✅ Configuración guardada en NVS");
    } else {
        ESP_LOGE(TAG, "❌ Error abriendo NVS");
    }
}

static void cargar_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    
    if (err == ESP_OK) {
        size_t required_size = sizeof(Config);
        err = nvs_get_blob(nvs_handle, "config", &config, &required_size);
        
        if (err == ESP_OK) {

            ESP_LOGI(TAG, "===============================");
            ESP_LOGI(TAG, "✅ Configuración cargada desde NVS");
            ESP_LOGI(TAG, "   Tiempo cruce: %d ms", config.tiempo_cruce_ms);
            ESP_LOGI(TAG, "   Tiempo reinicio: %d ms", config.tiempo_reinicio_ms);
            ESP_LOGI(TAG, "   Estado alarma: %s", config.state_alarma ? "ON" : "OFF");
            ESP_LOGI(TAG, "   Estado rojo: %s", config.state_rojo ? "ON" : "OFF");
            ESP_LOGI(TAG, "   Estado verde: %s", config.state_verde ? "ON" : "OFF");
            ESP_LOGI(TAG, "   Umbral (m): %.2f", config.threshold_m);
            ESP_LOGI(TAG, "   Tiempo de espera: %d ms", config.hold_time_ms);
            ESP_LOGI(TAG, "===============================");

        } else {
            ESP_LOGI(TAG, "⚙️ Usando configuración por defecto");
            guardar_config();
        }
        nvs_close(nvs_handle);
    }
}

static void send_config_esp(uint16_t conn_handle)
{
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"tc\":%d,\"tr\":%d,"
        "\"sa\":%d,\"sr\":%d,\"sv\":%d,\"th\":%.2f,\"hm\":%d}", 
        config.tiempo_cruce_ms, 
        config.tiempo_reinicio_ms,
        config.state_alarma    ? 1 : 0,
        config.state_rojo    ? 1 : 0,
        config.state_verde    ? 1 : 0,
        config.threshold_m,
        config.hold_time_ms
    );
    nimble_server_send_str(conn_handle, SVC_ESP, ESP_TX, buf);
    ESP_LOGI(TAG, "CONFIGURACION ENVIADA A ESP32 CLIENTE");
}

static void send_config_app(uint16_t conn_handle)
{
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"tc\":%d,\"tr\":%d,"
        "\"sa\":%d,\"sr\":%d,\"sv\":%d,\"th\":%.2f,\"hm\":%d}", 
        config.tiempo_cruce_ms, 
        config.tiempo_reinicio_ms,
        config.state_alarma    ? 1 : 0,
        config.state_rojo    ? 1 : 0,
        config.state_verde    ? 1 : 0,
        config.threshold_m,
        config.hold_time_ms
    );
    nimble_server_send_str(conn_handle, SVC_APP, APP_TX, buf);
    ESP_LOGI(TAG, "CONFIGURACION ENVIADA A APP");
}

static void send_sensor_app(void)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"sen\",\"dist\":%.2f,\"det\":%d,\"conn\":%d}",
        last_dist_m,
        persona_detectada ? 1 : 0,
        sensor_conectado  ? 1 : 0
    );
    nimble_server_send_str(NIMBLE_CONN_ALL, SVC_APP, APP_TX, buf);
}

static inline void set_alarma(void)
{
    if (alarma_activa && config.state_alarma) {
        gpio_set_level(ALARMA, ON);
    } else {
        gpio_set_level(ALARMA, OFF);
    }
}

static inline void set_verde(void)
{
    if (verde_activo && config.state_verde) {
        gpio_set_level(VERDE, ON);
    } else {
        gpio_set_level(VERDE, OFF);
    }
}

static inline void set_rojo(void)
{
    if (rojo_activo && config.state_rojo) {
        gpio_set_level(ROJO, ON);
    } else {
        gpio_set_level(ROJO, OFF);
    }
}

static void check_contadores_vacios(void)
{
    if (contador_server == 0 && contador_cliente == 0)
    {
        ESP_LOGI(TAG, "🚶 Peatonal vacío → reinicio inmediato");
        xTimerStop(timer_cruce,    0);
        xTimerStop(timer_reinicio, 0);   // ← cancela el timer, ya no hace falta
        flag_reinicio = true;            // ← misma lógica que si hubiera expirado
    }
}

static void aplicar_nuevos_timers(void)
{
    ESP_LOGI(TAG, "Aplicando nuevos períodos a los temporizadores...");

    // Verificar que los temporizadores estén inicializados
    if (timer_cruce == NULL || timer_reinicio == NULL) {
        ESP_LOGE(TAG, "❌ Los temporizadores no están inicializados.");
        return;
    }

    // Detener los temporizadores antes de cambiar el período
    if (xTimerIsTimerActive(timer_cruce)) {
        ESP_LOGI(TAG, "El temporizador timer_cruce está activo. Deteniéndolo...");
        xTimerStop(timer_cruce, 0); // Detener el temporizador
    }
    if (xTimerIsTimerActive(timer_reinicio)) {
        ESP_LOGI(TAG, "El temporizador timer_reinicio está activo. Deteniéndolo...");
        xTimerStop(timer_reinicio, 0); // Detener el temporizador
    }

    // Cambiar el período del temporizador de cruce
    if (xTimerChangePeriod(timer_cruce, pdMS_TO_TICKS(config.tiempo_cruce_ms), 0) == pdPASS) {
        ESP_LOGI(TAG, "✅ Período de timer_cruce cambiado a %d ms", config.tiempo_cruce_ms);
    } else {
        ESP_LOGE(TAG, "❌ Error al cambiar el período de timer_cruce");
    }

    // Cambiar el período del temporizador de reinicio
    if (xTimerChangePeriod(timer_reinicio, pdMS_TO_TICKS(config.tiempo_reinicio_ms), 0) == pdPASS) {
        ESP_LOGI(TAG, "✅ Período de timer_reinicio cambiado a %d ms", config.tiempo_reinicio_ms);
    } else {
        ESP_LOGE(TAG, "❌ Error al cambiar el período de timer_reinicio");
    }

    // Asegurarse de que los temporizadores estén detenidos
    if (xTimerStop(timer_cruce, 0) == pdPASS) {
        ESP_LOGI(TAG, "✅ Temporizador timer_cruce detenido.");
    } else {
        ESP_LOGE(TAG, "❌ Error al detener el temporizador timer_cruce.");
    }

    if (xTimerStop(timer_reinicio, 0) == pdPASS) {
        ESP_LOGI(TAG, "✅ Temporizador timer_reinicio detenido.");
    } else {
        ESP_LOGE(TAG, "❌ Error al detener el temporizador timer_reinicio.");
    }

    guardar_config();
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    ESP_LOGI(TAG, "✅ Los temporizadores han sido actualizados y están en estado STOP.");
}

// =====================================================================
//                          CONFIGURACION DE BLE
// =====================================================================

static void on_connect(const nimble_conn_info_t *conn)
{
    ESP_LOGI(TAG, "✓ Conectado  conn=%d  MAC=%s  MTU=%d",
             conn->conn_handle, conn->addr_str, conn->mtu);
}

static void on_disconnect(const nimble_conn_info_t *conn, int reason)
{
    ESP_LOGW(TAG, "✗ Desconectado  conn=%d  MAC=%s  reason=%d",
             conn->conn_handle, conn->addr_str, reason);
}

static void on_receive(const nimble_conn_info_t *conn, uint8_t svc_idx, uint8_t char_idx, const uint8_t *data, uint16_t len)
{
    /*ESP_LOGI(TAG, "← [svc=%d char=%d] conn=%d MAC=%s → %.*s",
             svc_idx, char_idx,
             conn->conn_handle, conn->addr_str,
             len, data);*/

    // ── APP → servidor ────────────────────────────────────────
    if (svc_idx == SVC_APP && char_idx == APP_RX)
    {
        char buf[512];
        int copy_len = len < (int)(sizeof(buf)-1) ? len : (int)(sizeof(buf)-1);
        memcpy(buf, data, copy_len);
        buf[copy_len] = '\0';

        cJSON* root = cJSON_Parse(buf);
        if (!root) {
            nimble_server_send_str(conn->conn_handle, SVC_APP, APP_TX,
                "{\"ok\":false,\"err\":\"parse\"}");
            return;
        }

        cJSON* rst = cJSON_GetObjectItem(root, "rst");
        if (cJSON_IsTrue(rst)) {
            nimble_server_send_str(conn->conn_handle, SVC_APP, APP_TX, "{\"ok\":true,\"rst\":true}");
            cJSON_Delete(root);
            vTaskDelay(pdMS_TO_TICKS(400));
            esp_restart();
            return;
        }

        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE){

            cJSON* tc = cJSON_GetObjectItem(root, "tc");
            if (cJSON_IsNumber(tc) && tc->valueint != config.tiempo_cruce_ms) {
                config.tiempo_cruce_ms = tc->valueint;
                timers_cambio = true;
            }

            cJSON* tr = cJSON_GetObjectItem(root, "tr");
            if (cJSON_IsNumber(tr) && tr->valueint != config.tiempo_reinicio_ms) {
                config.tiempo_reinicio_ms = tr->valueint;
                timers_cambio = true;
            }

            cJSON* hm = cJSON_GetObjectItem(root, "hm");
            if (cJSON_IsNumber(hm) && hm->valueint != config.hold_time_ms) {
                config.hold_time_ms = hm->valueint;
                sensor_cambio = true;
            }

            cJSON* th = cJSON_GetObjectItem(root, "th");
            if (cJSON_IsNumber(th)) {
                float nuevo = (float)th->valuedouble;
                if (fabsf(nuevo - config.threshold_m) > 0.001f) {
                    config.threshold_m = nuevo;
                    sensor_cambio = true;
                }
            }

            cJSON* sa = cJSON_GetObjectItem(root, "sa");
            if (cJSON_IsBool(sa) && cJSON_IsTrue(sa) != config.state_alarma) {
                config.state_alarma = cJSON_IsTrue(sa);
                set_alarma();
            }

            cJSON* sv = cJSON_GetObjectItem(root, "sv");
            if (cJSON_IsBool(sv) && cJSON_IsTrue(sv) != config.state_verde) {
                config.state_verde = cJSON_IsTrue(sv);
                set_verde();
            }

            cJSON* sr = cJSON_GetObjectItem(root, "sr");
            if (cJSON_IsBool(sr) && cJSON_IsTrue(sr) != config.state_rojo) {
                config.state_rojo = cJSON_IsTrue(sr);
                set_rojo();
            }


            flag_apply_config    = true;
            flag_send_config_esp = true;
            xSemaphoreGive(config_mutex);
        }
            cJSON_Delete(root);
        nimble_server_send_str(conn->conn_handle, SVC_APP, APP_TX, "{\"ok\":true}");
        ESP_LOGI(TAG, "✅ Config actualizada y guardada");

    }

    // ── ESP32 cliente → servidor ──────────────────────────────
    else if (svc_idx == SVC_ESP && char_idx == ESP_RX)
    {

        if (len == 2 && memcmp(data, "IN", 2) == 0)
        {
            ESP_LOGI(TAG, ANSI_CYAN ">>> ENTRADA CLIENTE");

            xTimerStop(timer_cruce,    0);
            xTimerStop(timer_reinicio, 0);
            xTimerStart(timer_cruce,    0);
            xTimerStart(timer_reinicio, 0);
            contador_cliente++;

            alarma_activa = true;
            rojo_activo   = true;
            verde_activo  = false;

            set_alarma();
            set_rojo();
            set_verde();

            vTaskDelay(pdMS_TO_TICKS(50));

            return;

        }
        else if (len == 3 && memcmp(data, "OUT", 3) == 0)
        {
            if (contador_server > 0) contador_server--;
            check_contadores_vacios();
            return;
        }
    }
}

static void on_subscribe(const nimble_conn_info_t *conn, uint8_t svc_idx, uint8_t char_idx, bool notify_enabled, bool indicate_enabled)
{
    const char *svc_name = svc_idx == SVC_APP ? "APP" :
                           svc_idx == SVC_ESP ? "ESP" : "UNK";

    ESP_LOGI(TAG, "NOTIFY %s — [%s] char=%d  conn=%d  MAC=%s",
             notify_enabled ? "ON" : "OFF",
             svc_name, char_idx,
             conn->conn_handle, conn->addr_str);

    // ✅ App móvil se suscribe → enviar config + estado actual
    if (svc_idx == SVC_APP && char_idx == APP_TX && notify_enabled) {
        send_config_app(conn->conn_handle);
    }

    // ✅ ESP32 cliente se suscribe → enviar config
    if (svc_idx == SVC_ESP && char_idx == ESP_TX && notify_enabled) {
        send_config_esp(conn->conn_handle);
    }
}

static void on_mtu(const nimble_conn_info_t *conn, uint16_t mtu)
{
    ESP_LOGI(TAG, "MTU conn=%d → %d bytes útiles por paquete",
             conn->conn_handle, mtu - 3);
}

static void on_rssi(const nimble_conn_info_t *conn, int8_t rssi)
{
    ESP_LOGI(TAG, "RSSI conn=%d MAC=%s → %d dBm",
             conn->conn_handle, conn->addr_str, rssi);
}

// Servicio 0 — App móvil
static nimble_char_cfg_t app_chars[] = {
    { .uuid_str = UUID_APP_TX,     .properties = BLE_PROP_NOTIFY,                    .max_len = 512,     .label = "app_tx"},
    { .uuid_str = UUID_APP_RX,     .properties = BLE_PROP_WRITE | BLE_PROP_WRITE_NR, .max_len = 512,     .label = "app_rx"},
};

// Servicio 1 — ESP32 clientes
static nimble_char_cfg_t esp_chars[] = {
    { .uuid_str = UUID_ESP_TX,  .properties = BLE_PROP_NOTIFY,                     .max_len = 512,   .label = "esp_tx"  },
    { .uuid_str = UUID_ESP_RX,  .properties = BLE_PROP_WRITE | BLE_PROP_WRITE_NR,  .max_len = 512,   .label = "esp_rx"  },
};

static nimble_service_cfg_t services[] = {
    {
        .uuid_str        = UUID_APP_SVC,
        .chars           = app_chars,
        .num_chars       = 2,
        .max_connections = 1,           // solo 1 app móvil
        .label           = "app_movil",
    },
    {
        .uuid_str        = UUID_ESP_SVC,
        .chars           = esp_chars,
        .num_chars       = 2,
        .max_connections = 4,           // hasta 4 ESP32 clientes
        .label           = "esp32_clientes",
    },

};

static nimble_server_cfg_t server_cfg = {
    .device_name          = "A11_LEDAC_SRV_01",
    .services             = services,
    .num_services         = 2,
    .max_connections      = 5,          // 1 + 4
    .on_connect           = on_connect,
    .on_disconnect        = on_disconnect,
    .on_receive           = on_receive,
    .on_subscribe         = on_subscribe,
    .on_mtu               = on_mtu,
    .on_rssi              = on_rssi,
    .enable_rssi          = false,      // RSSI APAGADO
    .rssi_interval_ms     = 10000,      // RSSI cada 10s
    .auto_restart_adv     = true,
    .adv_interval_min_ms  = 100,
    .adv_interval_max_ms  = 200,
};

// =====================================================================
//                      CONFIGURACION DE SENSOR DE DISTANCIA
// =====================================================================
void config_sensor(void)
{
    a11_sensor_config_t cfg1 = A11_SENSOR_DEFAULT_CONFIG();
    cfg1.uart_num          = UART_NUM_1;
    cfg1.rx_pin            = GPIO_NUM_20;
    cfg1.trigger_pin       = GPIO_NUM_18;
    cfg1.threshold_m       = config.threshold_m;
    cfg1.hold_time_ms      = config.hold_time_ms;
    cfg1.retry_interval_ms = 5000;

    a11_sensor_handle_t sensor1 = NULL;
    ESP_ERROR_CHECK(a11_sensor_init(&cfg1, &sensor1));

    p1.sensor         = sensor1;
    p1.output_pin     = GPIO_NUM_NC;
    p1.name           = "S1";
    p1.start_delay_ms = 50;
}

void reiniciar_sensor(void)
{
    ESP_LOGI(TAG, "Reiniciando sensor...");

    // 1. Pausar sensor_task antes de tocar nada
    if (sensor_task_handle != NULL)
        vTaskSuspend(sensor_task_handle);

    // 2. Esperar que termine su ciclo actual
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. Ahora sí es seguro destruir
    if (p1.sensor != NULL) {
        a11_sensor_deinit(p1.sensor);
        p1.sensor = NULL;
    }

    // 4. Recrear con nueva config
    config_sensor();

    // 5. Reanudar
    if (sensor_task_handle != NULL)
        vTaskResume(sensor_task_handle);

    ESP_LOGI(TAG, "✅ Sensor reiniciado correctamente");
}

// =====================================================================
//                      CONFIGURACION DE PINES COMO SALIDA
// =====================================================================
static void output_gpio_init(gpio_num_t pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(pin, OFF));
}

// ============================================================
// TIMERS DE CRUVE Y REINCIO
// ============================================================

static void timer_cruce_cb(TimerHandle_t xTimer)
{
    paso_libre    = false;
    ESP_LOGI(TAG, "⏱ Timer cruce expiró");
}

static void resetear_sistema(void)
{
    paso_libre        = true;
    persona_detectada = false;
    hold_new_person   = false;
    hold_activo       = false;

    contador_cliente  = 0;
    contador_server   = 0;

    alarma_activa = false;
    verde_activo = true;
    rojo_activo = false;
    set_alarma();
    set_verde();
    set_rojo();
}

static void timer_reinicio_cb(TimerHandle_t xTimer)
{
    flag_reinicio = true;
    xTimerStop(timer_cruce,    0);
    xTimerStop(timer_reinicio, 0);
    ESP_LOGI(TAG, "⏱ Timer reinicio expiró");
}

// ============================================================
// TAREAS DE EJECUCION
// ============================================================
static void main_task(void* arg)
{
    while (1)
    {

        if (flag_apply_config) {
            flag_apply_config = false;
            guardar_config();
            ESP_LOGI(TAG, "✅ Config guardada");
        }

        if (flag_send_config_esp) {
            flag_send_config_esp = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            send_config_esp(NIMBLE_CONN_ALL);
        }

        // SI SE CAMBIO LOS TIEMPOS DE LOS TIMER SE REINICIA
        if (timers_cambio) {
            timers_cambio = false;
            aplicar_nuevos_timers();
        }

        //SI SE CAMBIO UNA CONFIGURACION DEL SENSOR SE REINICIA
        if (sensor_cambio) {
            sensor_cambio = false;
            reiniciar_sensor();
        }

        // SI SE REINICIO O LOS CONTADORES SE PUSIERON EN 0 
        if (flag_reinicio) {
            flag_reinicio = false;
            resetear_sistema();
        }

        // ── Solo actuar si el cruce está disponible ───────────
        //ENTRADA
        if (paso_libre && hold_new_person)
        {
            ESP_LOGI(TAG, ANSI_MAGENTA ">>> ENTRADA SERVER");

            hold_new_person = false;

            xTimerStop(timer_cruce,    0);
            xTimerStop(timer_reinicio, 0);
            xTimerStart(timer_cruce,    0);
            xTimerStart(timer_reinicio, 0);

            contador_server++;
            
            nimble_server_send_str(NIMBLE_CONN_ALL, SVC_ESP, ESP_TX, "IN");

            rojo_activo = true;
            alarma_activa = true;
            verde_activo = false;
            set_alarma();
            set_rojo();
            set_verde();
            vTaskDelay(pdMS_TO_TICKS(50));

        }

        //SALIDA
        else if (!paso_libre && hold_new_person) //SALIDA
        {
            hold_new_person = false;

            nimble_server_send_str(NIMBLE_CONN_ALL, SVC_ESP, ESP_TX, "OUT");

            if(contador_cliente > 0){
                contador_cliente --;
            }

            check_contadores_vacios();

        }

        /*ESP_LOGI(TAG, "📊 server=%d  cliente=%d  paso=%s  dist=%.2fm",
                contador_server,
                contador_cliente,
                paso_libre ? "LIBRE" : "OCUPADO",
                last_dist_m);*/

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


// ===================== TASK SENSOR =====================
static void sensor_task(void *arg)
{
    sensor_task_params_t *p = (sensor_task_params_t *)arg;

    if (p->start_delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(p->start_delay_ms));
    }

    while (1) {

        TickType_t loop_start = xTaskGetTickCount();

        // ---- Retry automático ----
        if (a11_sensor_should_retry(p->sensor)) {
            //ESP_LOGW(TAG, "[%s] Reintentando conexión...", p->name);
            a11_sensor_reset(p->sensor);
        }

        // ---- Sensor Desconectado ----
        if (a11_sensor_get_state(p->sensor) == A11_SENSOR_STATE_DEAD) 
        {
            sensor_conectado = false;
            persona_detectada = false;
            hold_new_person = false;

            send_sensor_app();

            TickType_t e = xTaskGetTickCount() - loop_start;
            if (e < pdMS_TO_TICKS(110)) vTaskDelay(pdMS_TO_TICKS(110) - e);
            continue;
        }
        else {
            sensor_conectado = true;
        }
        
        // ---- Detección ----
        float        dist_m = 0.0f;
        a11_detect_t det    = a11_sensor_detect(p->sensor, &dist_m);
        if (dist_m > 0.0f) last_dist_m = dist_m;

        if (det == A11_DETECT_NEAR){

            persona_detectada = true;
            if (!hold_activo)           // ← solo en el PRIMER flanco
            {
                hold_activo     = true;
                hold_new_person = true; // ← avisa UNA sola vez a main_task
            }
        }
        
        else{
            persona_detectada = false;
            hold_activo       = false;
        }

        send_sensor_app();

        ESP_LOGI(TAG, "[%s] dist=%.3f m | det=%d", p->name, dist_m, (int)det);
        // ---- Periodo 110ms ----
        TickType_t elapsed = xTaskGetTickCount() - loop_start;
        if (elapsed < pdMS_TO_TICKS(110)) vTaskDelay(pdMS_TO_TICKS(110) - elapsed);
    }
}

static void watchdog_task(void* arg)
{
    output_gpio_init(PIN_WATCHDOG);

    while (1)
    {
        gpio_set_level(PIN_WATCHDOG, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));   // pulso 100ms alto
        gpio_set_level(PIN_WATCHDOG, 0);
        vTaskDelay(pdMS_TO_TICKS(WDG_INTERVAL_MS)); // espera 2s
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);       // Oculta INFO de todos
    esp_log_level_set("SP_SERVER", ESP_LOG_INFO); // Muestra solo tus logs

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //cargar_config();

    ESP_LOGI(TAG, "Iniciando server BLE con 2 servicios...");
    ESP_ERROR_CHECK(nimble_server_init(&server_cfg));

    ESP_LOGI(TAG, "Panel inicializado correctamente");

    vTaskDelay(pdMS_TO_TICKS(5000));
    config_sensor();
    
    output_gpio_init(ALARMA);
    output_gpio_init(VERDE);
    output_gpio_init(ROJO);

    set_alarma();
    set_verde();   // ← enciende verde porque verde_activo = true
    set_rojo();

    timer_cruce    = xTimerCreate("t_cruce",    pdMS_TO_TICKS(config.tiempo_cruce_ms),pdFALSE, NULL, timer_cruce_cb);
    timer_reinicio = xTimerCreate("t_reinicio", pdMS_TO_TICKS(config.tiempo_reinicio_ms),pdFALSE, NULL, timer_reinicio_cb);

    config_mutex = xSemaphoreCreateMutex();

    xTaskCreate(sensor_task,    "sensor1_task",     6144,   &p1,        5, &sensor_task_handle);
    xTaskCreate(main_task,      "main_task",        4096,   NULL,       4, NULL);
    xTaskCreate(watchdog_task,  "wdg_task",         2048,   NULL,       3, NULL);
}