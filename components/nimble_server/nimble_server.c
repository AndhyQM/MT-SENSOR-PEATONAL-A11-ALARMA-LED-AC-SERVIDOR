#include "nimble_server.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"            // ✅ MEJORA 1: mutex

#include <string.h>
#include <stdlib.h>

#define TAG "NIMBLE_SERVER"

// =============================================================================
//  Límites internos
// =============================================================================

#define MAX_SERVICES     8
#define MAX_CHARS        16
#define MAX_CONNECTIONS  8

// =============================================================================
//  Estructuras internas
// =============================================================================

typedef struct {
    uint16_t val_handle;
    uint16_t dsc_handle;
    uint8_t  svc_idx;
    uint8_t  char_idx;
    uint8_t  global_idx;
    uint8_t  properties;
    uint8_t  buf[512];
    uint16_t buf_len;
} char_entry_t;

typedef struct {
    uint16_t           handle;
    nimble_conn_info_t info;
    bool               subscribed[MAX_CHARS];   // indexado por global_idx
} conn_entry_t;

// =============================================================================
//  Estado global
// =============================================================================

static const nimble_server_cfg_t *s_cfg             = NULL;
static char_entry_t               s_chars[MAX_CHARS];
static uint8_t                    s_char_count       = 0;
static conn_entry_t               s_conns[MAX_CONNECTIONS];
static uint8_t                    s_conn_count       = 0;
static TaskHandle_t               s_rssi_task_handle = NULL;
static SemaphoreHandle_t          s_mutex            = NULL; // ✅ MEJORA 1

static ble_uuid128_t s_svc_uuids [MAX_SERVICES];
static ble_uuid128_t s_char_uuids[MAX_CHARS];

static struct ble_gatt_chr_def  *s_gatt_chars[MAX_SERVICES];
static struct ble_gatt_svc_def   s_gatt_svcs [MAX_SERVICES + 1];

// =============================================================================
//  Forward declaration
// =============================================================================

static int gap_event_cb(struct ble_gap_event *event, void *arg);

// =============================================================================
//  Helpers
// =============================================================================

static bool parse_uuid128(const char *str, ble_uuid128_t *out)
{
    out->u.type = BLE_UUID_TYPE_128;
    uint8_t *b  = out->value;
    int r = sscanf(str,
        "%02hhx%02hhx%02hhx%02hhx-"
        "%02hhx%02hhx-"
        "%02hhx%02hhx-"
        "%02hhx%02hhx-"
        "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
        &b[15],&b[14],&b[13],&b[12],
        &b[11],&b[10],
        &b[9], &b[8],
        &b[7], &b[6],
        &b[5], &b[4],&b[3],&b[2],&b[1],&b[0]);
    return (r == 16);
}

static void addr_to_str(const ble_addr_t *addr, char *out)
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

static conn_entry_t *find_conn(uint16_t handle)
{
    for (int i = 0; i < s_conn_count; i++) {
        if (s_conns[i].handle == handle) return &s_conns[i];
    }
    return NULL;
}

static char_entry_t *find_char_by_val_handle(uint16_t val_handle)
{
    for (int i = 0; i < s_char_count; i++) {
        if (s_chars[i].val_handle == val_handle) return &s_chars[i];
    }
    return NULL;
}

// ✅ MEJORA 5: calcula el máximo payload seguro para una conexión
static uint16_t safe_payload_len(uint16_t conn_handle, uint16_t desired_len)
{
    uint16_t mtu = ble_att_mtu(conn_handle);
    if (mtu < 4) mtu = 23;             // fallback mínimo BLE
    uint16_t max_payload = mtu - 3;    // ATT overhead = 3 bytes
    return (desired_len < max_payload) ? desired_len : max_payload;
}

// =============================================================================
//  Advertising
// =============================================================================

static esp_err_t start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    uint16_t min_ms = s_cfg->adv_interval_min_ms ? s_cfg->adv_interval_min_ms : 100;
    uint16_t max_ms = s_cfg->adv_interval_max_ms ? s_cfg->adv_interval_max_ms : 200;
    adv_params.itvl_min = (min_ms * 1000) / 625;
    adv_params.itvl_max = (max_ms * 1000) / 625;

    struct ble_hs_adv_fields fields = {0};
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name                  = (const uint8_t *)s_cfg->device_name;
    fields.name_len              = strlen(s_cfg->device_name);
    fields.name_is_complete      = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Advertising iniciado [%s]", s_cfg->device_name);
    return ESP_OK;
}

// =============================================================================
//  GATT callback
// =============================================================================

static int gatt_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char_entry_t *ce = find_char_by_val_handle(attr_handle);
    if (!ce) return BLE_ATT_ERR_ATTR_NOT_FOUND;

    switch (ctxt->op) {

    case BLE_GATT_ACCESS_OP_READ_CHR:
        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 1
        os_mbuf_append(ctxt->om, ce->buf, ce->buf_len);
        xSemaphoreGive(s_mutex);
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 1
        ble_hs_mbuf_to_flat(ctxt->om, ce->buf, sizeof(ce->buf), &ce->buf_len);
        // Copia local para no llamar callback con mutex tomado
        uint8_t  tmp_buf[512];
        uint16_t tmp_len = ce->buf_len;
        memcpy(tmp_buf, ce->buf, tmp_len);
        uint8_t svc_idx  = ce->svc_idx;
        uint8_t char_idx = ce->char_idx;
        xSemaphoreGive(s_mutex);

        conn_entry_t *conn = find_conn(conn_handle);
        if (s_cfg->on_receive && conn) {
            s_cfg->on_receive(&conn->info, svc_idx, char_idx,
                              tmp_buf, tmp_len);
        }
        return 0;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// =============================================================================
//  GAP event callback
// =============================================================================

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Conexión fallida status=%d", event->connect.status);
            start_advertising();
            return 0;
        }

        if (s_conn_count >= s_cfg->max_connections) {
            ESP_LOGW(TAG, "Máx conexiones alcanzado, rechazando conn=%d",
                     event->connect.conn_handle);
            ble_gap_terminate(event->connect.conn_handle, BLE_ERR_CONN_LIMIT);
            return 0;
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 1
        conn_entry_t *ce = &s_conns[s_conn_count++];
        memset(ce, 0, sizeof(*ce));
        ce->handle = event->connect.conn_handle;

        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0) {
            addr_to_str(&desc.peer_id_addr, ce->info.addr_str);
            ce->info.mtu = ble_att_mtu(event->connect.conn_handle);
        }
        ce->info.conn_handle = event->connect.conn_handle;
        nimble_conn_info_t info_copy = ce->info;
        xSemaphoreGive(s_mutex);

        ESP_LOGI(TAG, "Conectado conn=%d MAC=%s MTU=%d",
                 info_copy.conn_handle, info_copy.addr_str, info_copy.mtu);

        if (s_cfg->on_connect) s_cfg->on_connect(&info_copy);

        if (s_conn_count < s_cfg->max_connections) {
            start_advertising();
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        uint16_t hdl = event->disconnect.conn.conn_handle;

        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 1
        nimble_conn_info_t info_copy = {0};
        int reason = event->disconnect.reason;
        for (int i = 0; i < s_conn_count; i++) {
            if (s_conns[i].handle == hdl) {
                info_copy    = s_conns[i].info;
                s_conns[i]   = s_conns[--s_conn_count];
                break;
            }
        }
        xSemaphoreGive(s_mutex);

        ESP_LOGW(TAG, "Desconectado conn=%d MAC=%s reason=%d",
                 hdl, info_copy.addr_str, reason);

        if (s_cfg->on_disconnect) s_cfg->on_disconnect(&info_copy, reason);

        // ✅ MEJORA 3: fix bug — antes `== 0` siempre era true
        if (s_cfg->auto_restart_adv) {
            start_advertising();
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU: {
        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 1
        conn_entry_t *ce = find_conn(event->mtu.conn_handle);
        nimble_conn_info_t info_copy = {0};
        uint16_t mtu = 0;
        if (ce) {
            ce->info.mtu = event->mtu.value;
            info_copy    = ce->info;
            mtu          = ce->info.mtu;
        }
        xSemaphoreGive(s_mutex);

        if (ce) {
            ESP_LOGI(TAG, "MTU conn=%d → %d (%d bytes útiles)",
                     event->mtu.conn_handle, mtu, mtu - 3);
            if (s_cfg->on_mtu) s_cfg->on_mtu(&info_copy, mtu);
        }
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE: {
        char_entry_t *ch = find_char_by_val_handle(event->subscribe.attr_handle);

        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 1
        conn_entry_t *ce = find_conn(event->subscribe.conn_handle);
        nimble_conn_info_t info_copy = {0};
        if (ce && ch) {
            bool notify   = event->subscribe.cur_notify;
            bool indicate = event->subscribe.cur_indicate;
            ce->subscribed[ch->global_idx] = notify || indicate;
            info_copy = ce->info;

            // ✅ MEJORA 7: log con label del servicio y char
            const char *svc_label  = (ch->svc_idx < s_cfg->num_services &&
                                      s_cfg->services[ch->svc_idx].label)
                                     ? s_cfg->services[ch->svc_idx].label : "?";
            const char *char_label = (ch->char_idx < s_cfg->services[ch->svc_idx].num_chars &&
                                      s_cfg->services[ch->svc_idx].chars[ch->char_idx].label)
                                     ? s_cfg->services[ch->svc_idx].chars[ch->char_idx].label : "?";

            ESP_LOGI(TAG, "Subscribe conn=%d [%s/%s] global=%d notify=%d indicate=%d",
                     ce->handle, svc_label, char_label,
                     ch->global_idx, notify, indicate);

            xSemaphoreGive(s_mutex);

            if (s_cfg->on_subscribe) {
                s_cfg->on_subscribe(&info_copy,
                                    ch->svc_idx, ch->char_idx,
                                    notify, indicate);
            }
        } else {
            xSemaphoreGive(s_mutex);
        }
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        return 0;
    }
}

// =============================================================================
//  NimBLE host task
// =============================================================================

static void on_stack_reset(int reason)
{
    ESP_LOGE(TAG, "Stack reset reason=%d — limpiando estado", reason);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_conn_count = 0;
    xSemaphoreGive(s_mutex);
}

static void on_stack_sync(void)
{
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        return;
    }
    start_advertising();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// =============================================================================
//  RSSI task — ✅ MEJORA 6: mutex al iterar conexiones
// =============================================================================

static void rssi_task(void *param)
{
    while (1) {
        uint32_t interval = (s_cfg && s_cfg->rssi_interval_ms)
                            ? s_cfg->rssi_interval_ms : 5000;
        vTaskDelay(pdMS_TO_TICKS(interval));

        if (!s_cfg) continue;

        // Snapshot de handles bajo mutex — no llamamos callback con mutex tomado
        uint16_t handles[MAX_CONNECTIONS];
        uint8_t  count = 0;

        xSemaphoreTake(s_mutex, portMAX_DELAY);         // ✅ MEJORA 6
        count = s_conn_count;
        for (int i = 0; i < count; i++) {
            handles[i] = s_conns[i].handle;
        }
        xSemaphoreGive(s_mutex);

        for (int i = 0; i < count; i++) {
            int8_t rssi = 0;
            if (ble_gap_conn_rssi(handles[i], &rssi) != 0) continue;

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            conn_entry_t *ce = find_conn(handles[i]);
            nimble_conn_info_t info_copy = {0};
            if (ce) {
                ce->info.rssi = rssi;
                info_copy     = ce->info;
            }
            xSemaphoreGive(s_mutex);

            if (ce && s_cfg->on_rssi) {
                s_cfg->on_rssi(&info_copy, rssi);
            }
        }
    }
}

// =============================================================================
//  Construcción dinámica de tabla GATT
// =============================================================================

static esp_err_t build_gatt_table(void)
{
    s_char_count = 0;
    memset(s_gatt_svcs, 0, sizeof(s_gatt_svcs));

    for (int si = 0; si < s_cfg->num_services; si++) {
        const nimble_service_cfg_t *svc = &s_cfg->services[si];

        if (!parse_uuid128(svc->uuid_str, &s_svc_uuids[si])) {
            ESP_LOGE(TAG, "UUID inválido servicio %d: %s", si, svc->uuid_str);
            return ESP_ERR_INVALID_ARG;
        }

        struct ble_gatt_chr_def *chr_table =
            calloc(svc->num_chars + 1, sizeof(struct ble_gatt_chr_def));
        if (!chr_table) return ESP_ERR_NO_MEM;

        for (int ci = 0; ci < svc->num_chars; ci++) {
            const nimble_char_cfg_t *ch = &svc->chars[ci];
            uint8_t global_idx = s_char_count;

            if (!parse_uuid128(ch->uuid_str, &s_char_uuids[global_idx])) {
                ESP_LOGE(TAG, "UUID inválido char %d.%d: %s", si, ci, ch->uuid_str);
                free(chr_table);
                return ESP_ERR_INVALID_ARG;
            }

            s_chars[global_idx].svc_idx    = si;
            s_chars[global_idx].char_idx   = ci;
            s_chars[global_idx].global_idx = global_idx;
            s_chars[global_idx].properties = ch->properties;

            ble_gatt_chr_flags flags = 0;
            if (ch->properties & BLE_PROP_READ)     flags |= BLE_GATT_CHR_F_READ;
            if (ch->properties & BLE_PROP_WRITE)    flags |= BLE_GATT_CHR_F_WRITE;
            if (ch->properties & BLE_PROP_WRITE_NR) flags |= BLE_GATT_CHR_F_WRITE_NO_RSP;
            if (ch->properties & BLE_PROP_NOTIFY)   flags |= BLE_GATT_CHR_F_NOTIFY;
            if (ch->properties & BLE_PROP_INDICATE) flags |= BLE_GATT_CHR_F_INDICATE;

            chr_table[ci] = (struct ble_gatt_chr_def){
                .uuid       = &s_char_uuids[global_idx].u,
                .access_cb  = gatt_chr_access_cb,
                .val_handle = &s_chars[global_idx].val_handle,
                .flags      = flags,
            };

            s_char_count++;
        }

        s_gatt_chars[si] = chr_table;
        s_gatt_svcs[si]  = (struct ble_gatt_svc_def){
            .type            = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid            = &s_svc_uuids[si].u,
            .characteristics = chr_table,
        };
    }

    return ESP_OK;
}

// =============================================================================
//  API pública
// =============================================================================

esp_err_t nimble_server_init(const nimble_server_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = cfg;
    s_conn_count = 0;

    // ✅ MEJORA 1: crear mutex antes de cualquier task
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    esp_err_t err = build_gatt_table();
    if (err != ESP_OK) return err;

    nimble_port_init();
    ble_hs_cfg.reset_cb        = on_stack_reset;
    ble_hs_cfg.sync_cb         = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs rc=%d", rc);
        return ESP_FAIL;
    }

    ble_svc_gap_device_name_set(cfg->device_name);

    if (cfg->enable_rssi) {
        xTaskCreate(rssi_task, "rssi_task", 3072, NULL, 2, &s_rssi_task_handle);
    }

    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "nimble_server iniciado — servicios=%d max_conn=%d",
             cfg->num_services, cfg->max_connections);
    return ESP_OK;
}

esp_err_t nimble_server_deinit(void)
{
    if (s_rssi_task_handle) {
        vTaskDelete(s_rssi_task_handle);
        s_rssi_task_handle = NULL;
    }
    for (int i = 0; i < s_cfg->num_services; i++) {
        if (s_gatt_chars[i]) {
            free(s_gatt_chars[i]);
            s_gatt_chars[i] = NULL;
        }
    }
    nimble_port_deinit();

    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    s_cfg = NULL;
    s_conn_count = 0;
    return ESP_OK;
}

esp_err_t nimble_server_send_bytes(uint16_t conn_handle,
                                   uint8_t  svc_idx,
                                   uint8_t  char_idx,
                                   const uint8_t *data,
                                   uint16_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    char_entry_t *ce         = NULL;
    uint8_t       global_idx = 0;
    for (int i = 0; i < s_char_count; i++) {
        if (s_chars[i].svc_idx == svc_idx &&
            s_chars[i].char_idx == char_idx) {
            ce         = &s_chars[i];
            global_idx = s_chars[i].global_idx;
            break;
        }
    }
    if (!ce) return ESP_ERR_NOT_FOUND;

    xSemaphoreTake(s_mutex, portMAX_DELAY);             // ✅ MEJORA 1
    uint16_t copy_len = len < sizeof(ce->buf) ? len : sizeof(ce->buf);
    memcpy(ce->buf, data, copy_len);
    ce->buf_len = copy_len;
    xSemaphoreGive(s_mutex);

    esp_err_t result = ESP_OK;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint8_t  snap_count                    = s_conn_count;
    uint16_t snap_handles[MAX_CONNECTIONS];
    bool     snap_subscribed[MAX_CONNECTIONS];
    for (int i = 0; i < snap_count; i++) {
        snap_handles[i]    = s_conns[i].handle;
        snap_subscribed[i] = s_conns[i].subscribed[global_idx];
    }
    xSemaphoreGive(s_mutex);

    for (int i = 0; i < snap_count; i++) {
        if (conn_handle != NIMBLE_CONN_ALL &&
            snap_handles[i] != conn_handle) continue;

        if (!snap_subscribed[i]) continue;

        // ✅ MEJORA 5: limitar al MTU real de cada conexión
        uint16_t safe_len = safe_payload_len(snap_handles[i], copy_len);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, safe_len);
        if (!om) { result = ESP_ERR_NO_MEM; continue; }

        int rc = ble_gatts_notify_custom(snap_handles[i], ce->val_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "notify falló conn=%d rc=%d", snap_handles[i], rc);
            result = ESP_FAIL;
        }
    }
    return result;
}

esp_err_t nimble_server_send_str(uint16_t    conn_handle,
                                 uint8_t     svc_idx,
                                 uint8_t     char_idx,
                                 const char *str)
{
    if (!str) return ESP_ERR_INVALID_ARG;
    return nimble_server_send_bytes(conn_handle, svc_idx, char_idx,
                                    (const uint8_t *)str, strlen(str));
}

bool    nimble_server_is_connected(void)    { return s_conn_count > 0; }
uint8_t nimble_server_connected_count(void) { return s_conn_count; }

esp_err_t nimble_server_get_conn_info(uint16_t conn_handle,
                                      nimble_conn_info_t *info)
{
    if (!info) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    conn_entry_t *ce = find_conn(conn_handle);
    if (ce) *info = ce->info;
    xSemaphoreGive(s_mutex);
    return ce ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t nimble_server_disconnect(uint16_t conn_handle)
{
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t nimble_server_start_advertising(void)
{
    return start_advertising();
}

void nimble_server_update_rssi(void)
{
    if (!s_cfg) return;

    uint16_t handles[MAX_CONNECTIONS];
    uint8_t  count = 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    count = s_conn_count;
    for (int i = 0; i < count; i++) handles[i] = s_conns[i].handle;
    xSemaphoreGive(s_mutex);

    for (int i = 0; i < count; i++) {
        int8_t rssi = 0;
        if (ble_gap_conn_rssi(handles[i], &rssi) != 0) continue;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        conn_entry_t *ce = find_conn(handles[i]);
        nimble_conn_info_t info_copy = {0};
        if (ce) { ce->info.rssi = rssi; info_copy = ce->info; }
        xSemaphoreGive(s_mutex);

        if (ce && s_cfg->on_rssi) s_cfg->on_rssi(&info_copy, rssi);
    }
}
