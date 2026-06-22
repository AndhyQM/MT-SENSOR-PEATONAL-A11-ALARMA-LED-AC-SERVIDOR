#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===================== ESTADO =====================
typedef enum {
    A11_SENSOR_STATE_OK      = 0,   // leyendo bien
    A11_SENSOR_STATE_FAILING = 1,   // fallos parciales, aún intenta
    A11_SENSOR_STATE_DEAD    = 2,   // máx fallos alcanzado
} a11_sensor_state_t;

// ===================== DETECCIÓN =====================
typedef enum {
    A11_DETECT_NONE  = 0,   // sin objeto (lejos)
    A11_DETECT_NEAR  = 1,   // objeto cerca O en hold — PIN = 0
} a11_detect_t;

// ===================== CONFIGURACIÓN =====================
typedef struct {
    uart_port_t uart_num;
    gpio_num_t  rx_pin;
    gpio_num_t  trigger_pin;
    int         baud_rate;
    int         buf_size;
    uint32_t    response_ms;        // espera tras trigger (ms)
    uint32_t    read_timeout_ms;    // timeout lectura UART (ms)
    uint8_t     max_fail_count;     // fallos antes de DEAD
    uint32_t    retry_interval_ms;  // 0 = no reintentar si DEAD
    float       threshold_m;        // distancia umbral detección (metros)
    uint32_t    hold_time_ms;       // tiempo hold tras perder detección (ms)
} a11_sensor_config_t;

// ===================== DEFAULTS =====================
#define A11_SENSOR_DEFAULT_CONFIG() {       \
    .uart_num           = UART_NUM_1,       \
    .rx_pin             = GPIO_NUM_18,      \
    .trigger_pin        = GPIO_NUM_17,      \
    .baud_rate          = 9600,             \
    .buf_size           = 256,              \
    .response_ms        = 40,               \
    .read_timeout_ms    = 10,               \
    .max_fail_count     = 5,                \
    .retry_interval_ms  = 5000,             \
    .threshold_m        = 1.00f,            \
    .hold_time_ms       = 1500,             \
}

typedef struct a11_sensor_ctx_t* a11_sensor_handle_t;

// ===================== API =====================
esp_err_t          a11_sensor_init        (const a11_sensor_config_t *config,
                                           a11_sensor_handle_t *handle);

// Lectura cruda en milímetros
bool               a11_sensor_read        (a11_sensor_handle_t handle,
                                           uint16_t *dist_mm);

// Lectura en metros con 2 decimales
bool               a11_sensor_read_m      (a11_sensor_handle_t handle,
                                           float *dist_m);

// Detección con umbral + hold integrado
// dist_m puede ser NULL si no se necesita el valor numérico
a11_detect_t       a11_sensor_detect      (a11_sensor_handle_t handle,
                                           float *dist_m);

// Retorna true si retry_interval_ms > 0 y ya pasó ese tiempo desde DEAD
bool               a11_sensor_should_retry(a11_sensor_handle_t handle);

a11_sensor_state_t a11_sensor_get_state   (a11_sensor_handle_t handle);
void               a11_sensor_reset       (a11_sensor_handle_t handle);
esp_err_t          a11_sensor_deinit      (a11_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif
