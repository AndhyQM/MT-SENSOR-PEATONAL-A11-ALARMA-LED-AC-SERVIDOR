#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nimble_server.h"
#include "nvs_flash.h"
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

#define ALARMA              GPIO_NUM_5
#define ROJO                GPIO_NUM_1
#define VERDE               GPIO_NUM_22

#define BOTON_PIN           GPIO_NUM_18
#define PIN_WATCHDOG        GPIO_NUM_19
#define WDG_INTERVAL_MS     2000         // pulso cada 2s
#define BOTON_DEBOUNCE_MS   20

// Servicio 0 — App móvil
#define UUID_APP_SVC            "CC100010-0000-0000-0000-000000000000"
#define UUID_APP_TX             "CC100010-0000-0000-0000-000000000001"  // notify → app
#define UUID_APP_RX             "CC100010-0000-0000-0000-000000000002"  // write  ← app

// Servicio 1 — ESP32 clientes
#define UUID_ESP_SVC            "DD200020-0000-0000-0000-000000000000"
#define UUID_ESP_TX             "DD200020-0000-0000-0000-000000000001"  // notify → clientes
#define UUID_ESP_RX             "DD200020-0000-0000-0000-000000000002"  // write  ← clientes

// Servicio 2 — ESP32 CLIENTE DUAL
#define UUID_ESP_SVC_DUAL       "EE200020-0000-0000-0000-000000000000"
#define UUID_ESP_TX_DUAL        "EE200020-0000-0000-0000-000000000001"  // notify → clientes
#define UUID_ESP_RX_DUAL        "EE200020-0000-0000-0000-000000000002"  // write  ← clientes

// ── INDICES ───────────────────────────────
#define SVC_APP         0
#define SVC_ESP         1
#define SVC_ESP_DUAL    2

// Chars servicio APP
#define APP_TX      0
#define APP_RX      1

// Chars servicio ESP
#define ESP_TX      0
#define ESP_RX      1

#define ESP_TX_DUAL      0
#define ESP_RX_DUAL      1

#define ON          1
#define OFF         0

// ── Tabla de clientes ─────────────────────────
#define MAX_ESP_CLIENTS 4

static uint16_t s_mine_handles[MAX_ESP_CLIENTS] = {0};  // clientes propios
static uint8_t  s_mine_count = 0;

static bool is_mine(uint16_t conn_handle) {
    for (int i = 0; i < s_mine_count; i++)
        if (s_mine_handles[i] == conn_handle) return true;
    return false;
}


static SemaphoreHandle_t config_mutex = NULL;

static volatile bool persona_dentro = false;   // true = ya entró, false = ya salió (o nunca entró)
static TimerHandle_t timer_reinicio = NULL;
static volatile bool hold_new_person = false;
static volatile bool flag_reinicio = false;
static volatile bool alarma_activa = false;
static volatile bool verde_activo  = true;
static volatile bool rojo_activo  = false;

static volatile bool timers_cambio = false;

static volatile bool flag_apply_config     = false;
static volatile bool flag_send_config_esp  = false;
static volatile bool flag_sync_local = false;  // sincroniza salidas físicas, NO reenvía BLE

// =====================================================================
//                          CONFIGURACION DE NVS
// =====================================================================
typedef struct {

    int tiempo_reinicio_ms;
    bool state_alarma;
    bool state_rojo;
    bool state_verde;

} Config;

// Valores por defecto
static Config config = {
    .tiempo_reinicio_ms     = 2000,
    .state_alarma           = true,
    .state_rojo             = true,
    .state_verde            = true,
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
            ESP_LOGI(TAG, "   Tiempo reinicio: %d ms", config.tiempo_reinicio_ms);
            ESP_LOGI(TAG, "   Estado alarma: %s", config.state_alarma ? "ON" : "OFF");
            ESP_LOGI(TAG, "   Estado rojo: %s", config.state_rojo ? "ON" : "OFF");
            ESP_LOGI(TAG, "   Estado verde: %s", config.state_verde ? "ON" : "OFF");
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
        "{\"tr\":%d,"
        "\"sa\":%d,\"sr\":%d,\"sv\":%d}", 
        config.tiempo_reinicio_ms,
        config.state_alarma    ? 1 : 0,
        config.state_rojo    ? 1 : 0,
        config.state_verde    ? 1 : 0
    );
    nimble_server_send_str(conn_handle, SVC_ESP, ESP_TX, buf);
    ESP_LOGI(TAG, "CONFIGURACION ENVIADA A ESP32 CLIENTE");
}

static void send_config_app(uint16_t conn_handle)
{
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"tr\":%d,"
        "\"sa\":%d,\"sr\":%d,\"sv\":%d}", 
        config.tiempo_reinicio_ms,
        config.state_alarma    ? 1 : 0,
        config.state_rojo    ? 1 : 0,
        config.state_verde    ? 1 : 0
    );
    nimble_server_send_str(conn_handle, SVC_APP, APP_TX, buf);
    ESP_LOGI(TAG, "CONFIGURACION ENVIADA A APP");
}

static inline void set_alarma(void)
{
    bool estado = alarma_activa && config.state_alarma;
    gpio_set_level(ALARMA, estado ? ON : OFF);
    ESP_LOGI(TAG, "🔔 ALARMA → %s", estado ? "ON" : "OFF");
}

static inline void set_verde(void)
{
    bool estado = verde_activo && config.state_verde;
    gpio_set_level(VERDE, estado ? ON : OFF);
    ESP_LOGI(TAG, "🟢 VERDE  → %s", estado ? "ON" : "OFF");
}

static inline void set_rojo(void)
{
    bool estado = rojo_activo && config.state_rojo;
    gpio_set_level(ROJO, estado ? ON : OFF);
    ESP_LOGI(TAG, "🔴 ROJO   → %s", estado ? "ON" : "OFF");
}

static void aplicar_nuevos_timers(void)
{
    guardar_config();
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "✅ Los temporizadores han sido actualizados y están en estado STOP.");
    esp_restart();
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

static void boton_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "✅ Botón inicializado en GPIO%d", BOTON_PIN);
}

static void aplicar_estado_local(void)
{
    xTimerStop(timer_reinicio, 0);

    if (persona_dentro) {
        xTimerStart(timer_reinicio, 0);
        rojo_activo   = true;
        alarma_activa = true;
        verde_activo  = false;
    } else {
        rojo_activo   = false;
        alarma_activa = false;
        verde_activo  = true;
    }

    set_alarma();
    set_rojo();
    set_verde();
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

    for (int i = 0; i < s_mine_count; i++) {
        if (s_mine_handles[i] == conn->conn_handle) {
            s_mine_handles[i] = s_mine_handles[--s_mine_count];
            ESP_LOGI(TAG, "🗑 Cliente propio eliminado conn=%d",
                     conn->conn_handle);
            break;
        }
    }
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

            cJSON* tr = cJSON_GetObjectItem(root, "tr");
            if (cJSON_IsNumber(tr) && tr->valueint != config.tiempo_reinicio_ms) {
                config.tiempo_reinicio_ms = tr->valueint;
                timers_cambio = true;
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

                // ── Identificación ────────────────────────
        if (len == 4 && memcmp(data, "MINE", 4) == 0) {
            s_mine_handles[s_mine_count++] = conn->conn_handle;
            ESP_LOGI(TAG, "✅ Cliente propio registrado conn=%d", conn->conn_handle);
            return;
        }

        if (len == 4 && memcmp(data, "DUAL", 4) == 0) {
            ESP_LOGI(TAG, ANSI_CYAN "✅ Cliente dual registrado conn=%d", conn->conn_handle);
            return;  // solo registrar, no procesar nada más
        }

        // ── Solo procesar si es cliente propio ────
        if (!is_mine(conn->conn_handle)) return;

        if (len == 2 && memcmp(data, "IN", 2) == 0)
        {
            ESP_LOGI(TAG, ANSI_CYAN ">>> ENTRADA CLIENTE");
            nimble_server_send_str(NIMBLE_CONN_ALL, SVC_ESP_DUAL, ESP_TX_DUAL , "IN");
            persona_dentro   = true;
            flag_sync_local  = true;   // ← solo sincroniza, no reenvía
            return;
        }
        if (len == 3 && memcmp(data, "OUT", 3) == 0)
        {
            ESP_LOGI(TAG, ANSI_CYAN ">>>>>>>>> SALIDA CLIENTE");
            nimble_server_send_str(NIMBLE_CONN_ALL, SVC_ESP_DUAL, ESP_TX_DUAL , "OUT");
            persona_dentro   = false;
            flag_sync_local  = true;   // ← solo sincroniza, no reenvía
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

// Servicio 2 — ESP32 cliente DUAL
static nimble_char_cfg_t esp_chars_dual[] = {
    { .uuid_str = UUID_ESP_TX_DUAL,  .properties = BLE_PROP_NOTIFY,                     .max_len = 512,   .label = "esp_tx_dual"  },
    { .uuid_str = UUID_ESP_RX_DUAL,  .properties = BLE_PROP_WRITE | BLE_PROP_WRITE_NR,  .max_len = 512,   .label = "esp_rx_dual|"  },
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
        .max_connections = 2,           // hasta 4 ESP32 clientes
        .label           = "esp32_clientes",
    },

    {
        .uuid_str        = UUID_ESP_SVC_DUAL,
        .chars           = esp_chars_dual,
        .num_chars       = 2,
        .max_connections = 2,           // hasta 4 ESP32 clientes
        .label           = "esp32_cliente_dual",
    },

};

static nimble_server_cfg_t server_cfg = {
    .device_name          = "A11_LED_AC_SRV_01",
    .services             = services,
    .num_services         = 3,
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

static void resetear_sistema(void)
{
    xTimerStop(timer_reinicio, 0);
    hold_new_person = false;   // ← cancela cualquier evento de botón pendiente
    flag_sync_local = false;   // ← cancela cualquier sync BLE pendiente
    ESP_LOGI(TAG, ANSI_RED ">>> SALIDA FORZADA (timeout)");
    persona_dentro = false;
    aplicar_estado_local();
}

static void timer_reinicio_cb(TimerHandle_t xTimer)
{
    flag_reinicio = true;
    ESP_LOGI(TAG, ANSI_RED "⏱ Timer reinicio expiró");
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

        // SI SE REINICIO O LOS CONTADORES SE PUSIERON EN 0 
        if (flag_reinicio) {
            flag_reinicio = false;
            resetear_sistema();
        }

        if (hold_new_person)
        {
            hold_new_person = false;
            ESP_LOGI(TAG, ANSI_MAGENTA "%s SERVER", persona_dentro ? ">>> ENTRADA" : ">>>>>>>>> SALIDA");

            nimble_server_send_str(NIMBLE_CONN_ALL, SVC_ESP, ESP_TX, persona_dentro ? "IN" : "OUT");
            nimble_server_send_str(NIMBLE_CONN_ALL, SVC_ESP_DUAL, ESP_TX_DUAL, persona_dentro ? "IN" : "OUT");

            aplicar_estado_local();
        }

        if (flag_sync_local)
        {
            flag_sync_local = false;
            aplicar_estado_local();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void boton_task(void *arg)
{
    bool estado_estable    = true;   // true = suelto (pull-up reposo = HIGH)
    bool ultima_lectura     = true;
    TickType_t t_ultimo_cambio = xTaskGetTickCount();

    while (1) {
        bool nivel_raw = gpio_get_level(BOTON_PIN);   // 1=suelto, 0=presionado

        // ---- Detecta cambio crudo y reinicia el conteo de antirrebote ----
        if (nivel_raw != ultima_lectura) {
            t_ultimo_cambio = xTaskGetTickCount();
            ultima_lectura  = nivel_raw;
        }

        // ---- Si ya pasó el tiempo de antirrebote, el nivel es estable ----
        if ((xTaskGetTickCount() - t_ultimo_cambio) > pdMS_TO_TICKS(BOTON_DEBOUNCE_MS))
        {
            if (nivel_raw != estado_estable)
            {
                estado_estable = nivel_raw;
                // ---- Flanco de bajada confirmado (HIGH→LOW) = presión real ----
                if (estado_estable == false)
                {
                    hold_new_person = true;
                    if (!persona_dentro)
                    {
                        // ── ENTRADA ──
                        persona_dentro = true;
                        //ESP_LOGI(TAG, ANSI_GREEN ">>> ENTRADA (botón)");
                        // próximamente: hold_new_person = true; (para enganchar al main_task)
                    }
                    else
                    {
                        // ── SALIDA ──
                        persona_dentro = false;
                        //ESP_LOGI(TAG, ANSI_RED ">>> SALIDA (botón)");
                        // próximamente: hold_new_person = true;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));   // polling rápido, antirrebote filtra el ruido
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

    /*nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "🗑️ NVS namespace '%s' borrado", NVS_NAMESPACE);
    }*/

    cargar_config();

    config_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "Iniciando server BLE con 2 servicios...");
    ESP_ERROR_CHECK(nimble_server_init(&server_cfg));

    output_gpio_init(ALARMA);
    output_gpio_init(VERDE);
    output_gpio_init(ROJO);    
    boton_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    set_alarma();
    set_verde();
    set_rojo();

    timer_reinicio =xTimerCreate("t_reinicio", pdMS_TO_TICKS(config.tiempo_reinicio_ms),pdFALSE, NULL, timer_reinicio_cb);

    xTaskCreate(main_task,      "main_task",        4096,   NULL,       4, NULL);
    xTaskCreate(watchdog_task,  "wdg_task",         2048,   NULL,       3, NULL);
    xTaskCreate(boton_task,     "boton_task",       2048,   NULL,       5, NULL);
}