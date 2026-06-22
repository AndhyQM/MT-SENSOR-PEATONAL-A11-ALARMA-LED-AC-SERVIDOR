#include "a11_sensor.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_idf_version.h"

static const char *TAG = "A11_SENSOR";

#define FRAME_SIZE      4
#define DIST_MM_MIN     10      // < 10mm = eco inválido

// ===================== CONTEXTO INTERNO =====================
typedef struct a11_sensor_ctx_t {
    a11_sensor_config_t cfg;
    uint8_t             fail_count;
    a11_sensor_state_t  state;

    // Hold state — solo timestamps, sin bucles
    bool       in_hold;
    TickType_t hold_start;

    // Retry state
    TickType_t dead_since;
} a11_sensor_ctx_t;

// ===================== TRIGGER =====================
static void sensor_trigger(const a11_sensor_config_t *cfg)
{
    gpio_set_level(cfg->trigger_pin, 1);
    esp_rom_delay_us(50);
    gpio_set_level(cfg->trigger_pin, 0);
    esp_rom_delay_us(300);
    gpio_set_level(cfg->trigger_pin, 1);
}

// ===================== PARSE =====================
// Frame: [0xFF][Data_H][Data_L][SUM]
// SUM  : (0xFF + Data_H + Data_L) & 0xFF
static bool sensor_parse(const uint8_t *buf, int len, uint16_t *dist_mm)
{
    if (len < FRAME_SIZE) return false;

    for (int i = 0; i <= len - FRAME_SIZE; i++) {
        if (buf[i] != 0xFF) continue;

        uint8_t data_h   = buf[i + 1];
        uint8_t data_l   = buf[i + 2];
        uint8_t sum_rx   = buf[i + 3];
        uint8_t sum_calc = (uint8_t)((0xFF + data_h + data_l) & 0xFF);

        if (sum_rx != sum_calc) continue;

        *dist_mm = ((uint16_t)data_h << 8) | data_l;
        return true;
    }
    return false;
}

// ===================== LECTURA INTERNA =====================
static bool sensor_do_read(a11_sensor_ctx_t *ctx, uint16_t *dist_mm)
{
    uint8_t rx_buf[16] = {0};

    uart_flush_input(ctx->cfg.uart_num);
    sensor_trigger(&ctx->cfg);
    vTaskDelay(pdMS_TO_TICKS(ctx->cfg.response_ms));

    int len = uart_read_bytes(ctx->cfg.uart_num,
                               rx_buf,
                               sizeof(rx_buf),
                               pdMS_TO_TICKS(ctx->cfg.read_timeout_ms));

    bool ok = sensor_parse(rx_buf, len, dist_mm);

    if (!ok) {
        ctx->fail_count++;
        if (ctx->fail_count >= ctx->cfg.max_fail_count) {
            if (ctx->state != A11_SENSOR_STATE_DEAD) {
                ctx->dead_since = xTaskGetTickCount();
                /*ESP_LOGE(TAG, "Sensor UART%d sin respuesta — desconectado?",
                         ctx->cfg.uart_num);*/
            }
            ctx->state = A11_SENSOR_STATE_DEAD;
        } else {
            ctx->state = A11_SENSOR_STATE_FAILING;
            /*ESP_LOGW(TAG, "Sensor UART%d fallo %d/%d",
                     ctx->cfg.uart_num,
                     ctx->fail_count,
                     ctx->cfg.max_fail_count);*/
        }
    } else {
        if (ctx->state != A11_SENSOR_STATE_OK) {
            //ESP_LOGI(TAG, "Sensor UART%d recuperado", ctx->cfg.uart_num);
        }
        ctx->fail_count = 0;
        ctx->state      = A11_SENSOR_STATE_OK;
    }

    return ok;
}

// ===================== INIT =====================
esp_err_t a11_sensor_init(const a11_sensor_config_t *config,
                           a11_sensor_handle_t *handle)
{
    if (!config || !handle) return ESP_ERR_INVALID_ARG;

    a11_sensor_ctx_t *ctx = calloc(1, sizeof(a11_sensor_ctx_t));
    if (!ctx) return ESP_ERR_NO_MEM;

    ctx->cfg        = *config;
    ctx->fail_count = 0;
    ctx->state      = A11_SENSOR_STATE_OK;
    ctx->in_hold    = false;
    ctx->hold_start = 0;
    ctx->dead_since = 0;

    gpio_config_t io_trigger = {
        .pin_bit_mask = (1ULL << config->trigger_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_trigger));
    ESP_ERROR_CHECK(gpio_set_level(config->trigger_pin, 1));

    const uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };

    ESP_ERROR_CHECK(uart_driver_install(config->uart_num,
                                        config->buf_size, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(config->uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_num,
                                  UART_PIN_NO_CHANGE,
                                  config->rx_pin,
                                  UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE));

    *handle = ctx;
    /*ESP_LOGI(TAG, "Sensor UART%d listo (trigger=GPIO%d, rx=GPIO%d, "
                  "umbral=%.2fm, hold=%lums)",
             config->uart_num, config->trigger_pin, config->rx_pin,
             config->threshold_m, (unsigned long)config->hold_time_ms);*/
    return ESP_OK;
}

// ===================== READ mm =====================
bool a11_sensor_read(a11_sensor_handle_t handle, uint16_t *dist_mm)
{
    if (!handle || !dist_mm) return false;
    a11_sensor_ctx_t *ctx = (a11_sensor_ctx_t *)handle;
    if (ctx->state == A11_SENSOR_STATE_DEAD) return false;
    return sensor_do_read(ctx, dist_mm);
}

// ===================== READ metros =====================
bool a11_sensor_read_m(a11_sensor_handle_t handle, float *dist_m)
{
    if (!handle || !dist_m) return false;
    a11_sensor_ctx_t *ctx = (a11_sensor_ctx_t *)handle;
    if (ctx->state == A11_SENSOR_STATE_DEAD) return false;

    uint16_t dist_mm = 0;
    bool ok = sensor_do_read(ctx, &dist_mm);

    if (ok) {
        if (dist_mm < DIST_MM_MIN) {
            /*ESP_LOGW(TAG, "Sensor UART%d eco invalido (%umm) — descartado",
                     ctx->cfg.uart_num, dist_mm);*/
            return false;
        }
        *dist_m = roundf((float)dist_mm / 10.0f) / 100.0f;
    }
    return ok;
}

// ===================== DETECT =====================
a11_detect_t a11_sensor_detect(a11_sensor_handle_t handle, float *dist_m)
{
    if (!handle) return A11_DETECT_NONE;
    a11_sensor_ctx_t *ctx = (a11_sensor_ctx_t *)handle;

    if (ctx->state == A11_SENSOR_STATE_DEAD) {
        ctx->in_hold = false;
        return A11_DETECT_NONE;
    }

    // ---- Si estamos en hold, NO medimos ----
    // Solo esperamos; recién al expirar volvemos a medir.
    if (ctx->in_hold) {
        TickType_t ahora = xTaskGetTickCount();
        bool hold_vivo = (ahora - ctx->hold_start) <
                          pdMS_TO_TICKS(ctx->cfg.hold_time_ms);
        if (hold_vivo) {
            return A11_DETECT_NEAR;   // hold activo, pin sigue en 0
        }
        // Hold expiró → salimos y medimos normalmente
        ctx->in_hold = false;
    }

    // ---- Medición normal ----
    uint16_t dist_mm = 0;
    bool ok = sensor_do_read(ctx, &dist_mm);
    if (!ok) return A11_DETECT_NONE;

    // ---- Eco inválido ----
    if (dist_mm < DIST_MM_MIN) {
        /*ESP_LOGW(TAG, "Sensor UART%d eco invalido (%umm) — descartado",
                 ctx->cfg.uart_num, dist_mm);*/
        return A11_DETECT_NONE;
    }

    float d = roundf((float)dist_mm / 10.0f) / 100.0f;
    if (dist_m) *dist_m = d;

    if (d < ctx->cfg.threshold_m) {
        // ---- CERCA → arrancar hold (una sola vez, sin recarga) ----
        ctx->hold_start = xTaskGetTickCount();
        ctx->in_hold    = true;
        return A11_DETECT_NEAR;
    }

    return A11_DETECT_NONE;
}


// ===================== SHOULD RETRY =====================
bool a11_sensor_should_retry(a11_sensor_handle_t handle)
{
    if (!handle) return false;
    a11_sensor_ctx_t *ctx = (a11_sensor_ctx_t *)handle;

    if (ctx->state != A11_SENSOR_STATE_DEAD) return false;
    if (ctx->cfg.retry_interval_ms == 0)     return false;

    TickType_t elapsed = xTaskGetTickCount() - ctx->dead_since;
    return elapsed >= pdMS_TO_TICKS(ctx->cfg.retry_interval_ms);
}

// ===================== GET STATE =====================
a11_sensor_state_t a11_sensor_get_state(a11_sensor_handle_t handle)
{
    if (!handle) return A11_SENSOR_STATE_DEAD;
    return ((a11_sensor_ctx_t *)handle)->state;
}

// ===================== RESET =====================
void a11_sensor_reset(a11_sensor_handle_t handle)
{
    if (!handle) return;
    a11_sensor_ctx_t *ctx = (a11_sensor_ctx_t *)handle;
    ctx->fail_count = 0;
    ctx->state      = A11_SENSOR_STATE_OK;
    ctx->dead_since = 0;
    ctx->in_hold    = false;
    ESP_LOGI(TAG, "Sensor UART%d reseteado — reintentando...", ctx->cfg.uart_num);
}

// ===================== DEINIT =====================
esp_err_t a11_sensor_deinit(a11_sensor_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    a11_sensor_ctx_t *ctx = (a11_sensor_ctx_t *)handle;
    uart_driver_delete(ctx->cfg.uart_num);
    free(ctx);
    return ESP_OK;
}