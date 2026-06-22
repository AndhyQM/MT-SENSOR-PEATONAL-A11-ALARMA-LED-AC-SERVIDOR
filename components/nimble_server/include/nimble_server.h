#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
//  Constantes
// =============================================================================

#define NIMBLE_CONN_ALL   0xFFFF   // Broadcast a todas las conexiones

// Propiedades de característica
#define BLE_PROP_READ        (1 << 0)
#define BLE_PROP_WRITE       (1 << 1)
#define BLE_PROP_WRITE_NR    (1 << 2)   // Write without response
#define BLE_PROP_NOTIFY      (1 << 3)
#define BLE_PROP_INDICATE    (1 << 4)

// =============================================================================
//  Estructuras públicas
// =============================================================================

/** Info de conexión — disponible en todos los callbacks */
typedef struct {
    uint16_t conn_handle;       // Handle BLE de la conexión
    char     addr_str[18];      // MAC en formato "AA:BB:CC:DD:EE:FF"
    uint16_t mtu;               // MTU negociado
    int8_t   rssi;              // RSSI en dBm (actualizado en connect)
} nimble_conn_info_t;

/** Configuración de una característica */
typedef struct {
    const char *uuid_str;       // UUID 128-bit como string
    uint8_t     properties;     // BLE_PROP_*
    uint16_t    max_len;        // Tamaño máximo del valor (default 512 si 0)
    const char *label;          // Nombre legible para logs
} nimble_char_cfg_t;

/** Configuración de un servicio */
typedef struct {
    const char         *uuid_str;       // UUID 128-bit como string
    nimble_char_cfg_t  *chars;          // Array de características
    uint8_t             num_chars;      // Cantidad de características
    uint8_t             max_connections;// Máx clientes en este servicio
    const char         *label;          // Nombre legible para logs
} nimble_service_cfg_t;

/** Configuración principal del server */
typedef struct {
    const char            *device_name;     // Nombre BLE visible
    nimble_service_cfg_t  *services;        // Array de servicios
    uint8_t                num_services;    // Cantidad de servicios
    uint8_t                max_connections; // Máx conexiones simultáneas globales

    // ── Callbacks ──────────────────────────────────────────────────────────
    /** Cliente conectado */
    void (*on_connect)   (const nimble_conn_info_t *conn);

    /** Cliente desconectado — reason = código BLE estándar */
    void (*on_disconnect)(const nimble_conn_info_t *conn, int reason);

    /** Datos recibidos por write en cualquier característica */
    void (*on_receive)   (const nimble_conn_info_t *conn,
                          uint8_t svc_idx, uint8_t char_idx,
                          const uint8_t *data, uint16_t len);

    /** Cliente activó/desactivó notify o indicate */
    void (*on_subscribe) (const nimble_conn_info_t *conn,
                          uint8_t svc_idx, uint8_t char_idx,
                          bool notify_enabled, bool indicate_enabled);

    /** MTU negociado con el cliente */
    void (*on_mtu)       (const nimble_conn_info_t *conn, uint16_t mtu);

    /** RSSI actualizado (llamado periódicamente si enable_rssi=true) */
    void (*on_rssi)      (const nimble_conn_info_t *conn, int8_t rssi);

    // ── Opciones avanzadas ─────────────────────────────────────────────────
    bool     enable_rssi;           // Activa polling de RSSI cada rssi_interval_ms
    uint32_t rssi_interval_ms;      // Intervalo RSSI (default 5000ms)
    bool     auto_restart_adv;      // Re-advertise al desconectar (default true)
    uint16_t adv_interval_min_ms;   // Intervalo mínimo advertising (default 100ms)
    uint16_t adv_interval_max_ms;   // Intervalo máximo advertising (default 200ms)
} nimble_server_cfg_t;

// =============================================================================
//  API pública
// =============================================================================

/**
 * Inicializa el stack NimBLE y arranca advertising.
 * Llama a nvs_flash_init() antes de esta función.
 */
esp_err_t nimble_server_init(const nimble_server_cfg_t *cfg);

/** Deinicializa el stack y libera recursos */
esp_err_t nimble_server_deinit(void);

/**
 * Envía bytes a una conexión específica o a todas (NIMBLE_CONN_ALL).
 * svc_idx  → índice del servicio en el array services[]
 * char_idx → índice de la característica en el array chars[]
 */
esp_err_t nimble_server_send_bytes(uint16_t conn_handle,
                                   uint8_t  svc_idx,
                                   uint8_t  char_idx,
                                   const uint8_t *data,
                                   uint16_t len);

/** Igual que send_bytes pero acepta string terminado en '\0' */
esp_err_t nimble_server_send_str(uint16_t    conn_handle,
                                 uint8_t     svc_idx,
                                 uint8_t     char_idx,
                                 const char *str);

/** Retorna true si hay al menos una conexión activa */
bool nimble_server_is_connected(void);

/** Retorna el número de conexiones activas */
uint8_t nimble_server_connected_count(void);

/**
 * Llena info con los datos de la conexión indicada.
 * Retorna ESP_ERR_NOT_FOUND si el handle no existe.
 */
esp_err_t nimble_server_get_conn_info(uint16_t conn_handle,
                                      nimble_conn_info_t *info);

/** Desconecta a un cliente específico */
esp_err_t nimble_server_disconnect(uint16_t conn_handle);

/** Fuerza re-inicio del advertising manualmente */
esp_err_t nimble_server_start_advertising(void);

/** Actualiza el RSSI de todas las conexiones activas */
void nimble_server_update_rssi(void);

#ifdef __cplusplus
}
#endif
