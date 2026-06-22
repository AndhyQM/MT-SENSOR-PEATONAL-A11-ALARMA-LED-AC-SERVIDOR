#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ─── Propiedades de característica ───────────────────────────────────────────
#define BLE_PROP_READ        (1 << 0)
#define BLE_PROP_WRITE       (1 << 1)
#define BLE_PROP_NOTIFY      (1 << 2)
#define BLE_PROP_INDICATE    (1 << 3)
#define BLE_PROP_WRITE_NR    (1 << 4)

// ─── Constantes globales ──────────────────────────────────────────────────────
#define NIMBLE_CONN_ALL       0xFFFF
#define NIMBLE_MTU_DEFAULT    247
#define NIMBLE_MAX_CONN       5

// ─── Info de una conexión ─────────────────────────────────────────────────────
typedef struct {
    uint16_t conn_handle;
    uint8_t  peer_addr[6];
    char     addr_str[18];   // "AA:BB:CC:DD:EE:FF\0"
    uint16_t mtu;
    int8_t   rssi;
    bool     encrypted;
} nimble_conn_info_t;

// ─── Callbacks comunes ────────────────────────────────────────────────────────
typedef void (*nimble_on_connect_cb_t)   (const nimble_conn_info_t *info);
typedef void (*nimble_on_disconnect_cb_t)(const nimble_conn_info_t *info);
typedef void (*nimble_on_data_cb_t)      (uint16_t conn_handle,
                                          const uint8_t *data,
                                          uint16_t len);
