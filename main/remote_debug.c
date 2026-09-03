#include "remote_debug.h"
#include <inttypes.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "mbedtls/sha256.h"
#include "usb_midi.h"
#include "ble_midi.h"

static const char *TAG = "remote_debug";
#define LOG_CAPACITY 80
#define LOG_LINE_SIZE 192
#define LOG_BATCH 32
#define OTA_PREFIX_SIZE (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))

static char log_lines[LOG_CAPACITY][LOG_LINE_SIZE];
static uint32_t log_sequence;
static portMUX_TYPE log_lock = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t serial_vprintf;
static atomic_uint counters[DIAG_COUNTER_COUNT];
static atomic_bool trace_enabled, wifi_connected, app_ready, ota_busy;
static atomic_uint wifi_disconnects;
static char management_password[129];
static httpd_handle_t server;
static esp_netif_t *station;
static esp_timer_handle_t reconnect_timer, reboot_timer;
static const char *counter_names[DIAG_COUNTER_COUNT] = {
    "usb_rx_events", "ble_tx_accepted", "ble_tx_not_ready", "ble_tx_errors",
    "ble_rx_packets", "usb_tx_queued", "ble_parse_errors", "usb_queue_errors",
};

static int capture_vprintf(const char *format, va_list args)
{
    char line[LOG_LINE_SIZE] = {0};
    va_list copy;
    va_copy(copy, args);
    int size = vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    if (size > 0) {
        portENTER_CRITICAL(&log_lock);
        memcpy(log_lines[log_sequence % LOG_CAPACITY], line, sizeof(line));
        log_sequence++;
        portEXIT_CRITICAL(&log_lock);
    }
    return serial_vprintf ? serial_vprintf(format, args) : size;
}

void remote_debug_capture_logs(void)
{
    if (!serial_vprintf) serial_vprintf = esp_log_set_vprintf(capture_vprintf);
}

void remote_debug_count(diag_counter_t counter, uint32_t amount)
{
    if (counter < DIAG_COUNTER_COUNT) atomic_fetch_add(&counters[counter], amount);
}

void remote_debug_trace(const char *direction, const uint8_t *data, size_t len)
{
    if (!trace_enabled) return;
    char hex[49];
    size_t count = len < 16 ? len : 16;
    for (size_t i = 0; i < count; i++) snprintf(hex + i * 3, 4, "%02x ", data[i]);
    hex[count * 3] = 0;
    ESP_LOGI("midi_trace", "%s bytes=%u: %s", direction, (unsigned)len, hex);
}

static esp_err_t reject(httpd_req_t *req, const char *status, const char *message)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    // Close to discard any unread request body, especially rejected OTA uploads.
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_sendstr(req, message);
    return ESP_FAIL;
}

static bool authorized(httpd_req_t *req)
{
    char header[144];
    size_t password_len = strlen(management_password);
    size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (!password_len || len != password_len + 7 || len >= sizeof(header) ||
        httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) != ESP_OK ||
        memcmp(header, "Bearer ", 7) != 0) return false;
    unsigned mismatch = 0;
    for (size_t i = 0; i < password_len; i++) mismatch |= header[i + 7] ^ management_password[i];
    return mismatch == 0;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *json)
{
    char *text = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    if (!text) return reject(req, "500 Internal Server Error", "Out of memory");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, text);
    free(text);
    return err;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    extern const char html_start[] asm("_binary_debug_html_start");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
    return httpd_resp_sendstr(req, html_start);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    if (!authorized(req)) return reject(req, "401 Unauthorized", "Management password required");
    usb_midi_status_t usb;
    ble_midi_status_t ble;
    usb_midi_get_status(&usb);
    ble_midi_get_status(&ble);
    cJSON *root = cJSON_CreateObject();
    cJSON *u = cJSON_AddObjectToObject(root, "usb");
    cJSON *b = cJSON_AddObjectToObject(root, "ble");
    cJSON *bridge = cJSON_AddObjectToObject(root, "bridge");
    cJSON *wifi = cJSON_AddObjectToObject(root, "wifi");
    cJSON *ota = cJSON_AddObjectToObject(root, "ota");
    if (!root || !u || !b || !bridge || !wifi || !ota) {
        cJSON_Delete(root);
        return reject(req, "500 Internal Server Error", "Out of memory");
    }
    const esp_app_desc_t *app = esp_app_get_description();
    cJSON_AddStringToObject(root, "version", app->version);
    cJSON_AddStringToObject(root, "build_date", app->date);
    cJSON_AddStringToObject(root, "build_time", app->time);
    cJSON_AddStringToObject(root, "idf", app->idf_ver);
    char sha[65];
    for (size_t i = 0; i < 32; i++) snprintf(sha + i * 2, 3, "%02x", app->app_elf_sha256[i]);
    cJSON_AddStringToObject(root, "elf_sha256", sha);
    cJSON_AddNumberToObject(root, "uptime_ms", esp_timer_get_time() / 1000);
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "minimum_free_heap", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "reset_reason", esp_reset_reason());
    cJSON_AddBoolToObject(root, "trace", trace_enabled);
    cJSON_AddBoolToObject(u, "connected", usb.connected);
    cJSON_AddBoolToObject(u, "rx_active", usb.rx_active);
    cJSON_AddBoolToObject(u, "tx_active", usb.tx_active);
    cJSON_AddBoolToObject(u, "tx_failed", usb.tx_failed);
#define USB_NUMBER(field) cJSON_AddNumberToObject(u, #field, usb.field)
    USB_NUMBER(in_endpoint); USB_NUMBER(out_endpoint); USB_NUMBER(session);
    USB_NUMBER(queued_events); USB_NUMBER(rx_transfers); USB_NUMBER(rx_bytes);
    USB_NUMBER(tx_transfers); USB_NUMBER(tx_bytes); USB_NUMBER(rx_errors); USB_NUMBER(tx_errors);
#undef USB_NUMBER
    cJSON_AddBoolToObject(b, "connected", ble.connected);
    cJSON_AddBoolToObject(b, "notifications_enabled", ble.notifications_enabled);
    cJSON_AddBoolToObject(b, "advertising", ble.advertising);
#define BLE_NUMBER(field) cJSON_AddNumberToObject(b, #field, ble.field)
    BLE_NUMBER(payload_mtu); BLE_NUMBER(notify_submitted); BLE_NUMBER(notify_errors);
    BLE_NUMBER(writes_received); BLE_NUMBER(disconnects);
#undef BLE_NUMBER
    for (size_t i = 0; i < DIAG_COUNTER_COUNT; i++)
        cJSON_AddNumberToObject(bridge, counter_names[i], atomic_load(&counters[i]));
    cJSON_AddBoolToObject(wifi, "connected", wifi_connected);
    cJSON_AddNumberToObject(wifi, "disconnects", wifi_disconnects);
    wifi_ap_record_t access_point;
    if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK)
        cJSON_AddNumberToObject(wifi, "rssi", access_point.rssi);
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(station, &ip) == ESP_OK) {
        char address[16];
        snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));
        cJSON_AddStringToObject(wifi, "ip", address);
    }
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    cJSON_AddStringToObject(ota, "running_partition", running ? running->label : "unknown");
    cJSON_AddStringToObject(ota, "next_partition", next ? next->label : "unavailable");
    cJSON_AddBoolToObject(ota, "busy", ota_busy);
    esp_ota_img_states_t state;
    if (running && esp_ota_get_state_partition(running, &state) == ESP_OK)
        cJSON_AddNumberToObject(ota, "image_state", state);
    return send_json(req, root);
}

static esp_err_t logs_handler(httpd_req_t *req)
{
    if (!authorized(req)) return reject(req, "401 Unauthorized", "Management password required");
    char query[64], value[16];
    uint32_t after = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "after", value, sizeof(value)) == ESP_OK)
        after = strtoul(value, NULL, 10);
    portENTER_CRITICAL(&log_lock);
    uint32_t newest = log_sequence;
    uint32_t oldest = newest >= LOG_CAPACITY ? newest - LOG_CAPACITY + 1 : 1;
    portEXIT_CRITICAL(&log_lock);
    uint32_t start = after ? after + 1 : (newest >= LOG_BATCH ? newest - LOG_BATCH + 1 : 1);
    if (start < oldest) start = oldest;
    // A client cursor from before a device reboot must start reading again.
    if (after > newest) start = oldest;
    cJSON *root = cJSON_CreateObject();
    cJSON *entries = cJSON_AddArrayToObject(root, "entries");
    if (!root || !entries) {
        cJSON_Delete(root);
        return reject(req, "500 Internal Server Error", "Out of memory");
    }
    uint32_t next = start - 1;
    for (uint32_t seq = start; seq <= newest && seq - start < LOG_BATCH; seq++) {
        char line[LOG_LINE_SIZE];
        portENTER_CRITICAL(&log_lock);
        bool present = log_sequence - seq < LOG_CAPACITY;
        if (present) memcpy(line, log_lines[(seq - 1) % LOG_CAPACITY], sizeof(line));
        portEXIT_CRITICAL(&log_lock);
        if (!present) continue;
        cJSON *entry = cJSON_CreateObject();
        if (!entry) break;
        cJSON_AddNumberToObject(entry, "seq", seq);
        cJSON_AddStringToObject(entry, "text", line);
        cJSON_AddItemToArray(entries, entry);
        next = seq;
    }
    cJSON_AddNumberToObject(root, "next", next);
    cJSON_AddNumberToObject(root, "oldest", oldest);
    cJSON_AddBoolToObject(root, "dropped", after && after + 1 < oldest);
    return send_json(req, root);
}

static esp_err_t trace_handler(httpd_req_t *req)
{
    if (!authorized(req)) return reject(req, "401 Unauthorized", "Management password required");
    char value;
    if (req->content_len != 1 || httpd_req_recv(req, &value, 1) != 1 || (value != '0' && value != '1'))
        return reject(req, "400 Bad Request", "Body must be 0 or 1");
    trace_enabled = value == '1';
    esp_log_level_set("USB_MIDI", trace_enabled ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    ESP_LOGI(TAG, "MIDI trace %s", trace_enabled ? "enabled" : "disabled");
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t reconnect_handler(httpd_req_t *req)
{
    if (!authorized(req)) return reject(req, "401 Unauthorized", "Management password required");
    if (req->content_len) return reject(req, "400 Bad Request", "No body expected");
    esp_err_t err = usb_midi_reconnect();
    if (err != ESP_OK) return reject(req, "503 Service Unavailable", "USB driver unavailable");
    ESP_LOGI(TAG, "USB reopen requested via network");
    return httpd_resp_sendstr(req, "USB reopen queued");
}

static bool receive_exact(httpd_req_t *req, uint8_t *data, size_t len)
{
    size_t done = 0;
    while (done < len) {
        int count = httpd_req_recv(req, (char *)data + done, len - done);
        if (count <= 0) return false;
        done += count;
    }
    return true;
}

static esp_err_t ota_handler(httpd_req_t *req)
{
    if (!authorized(req)) return reject(req, "401 Unauthorized", "Management password required");
    if (ota_busy) return reject(req, "409 Conflict", "Update already pending");
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (!partition) return reject(req, "503 Service Unavailable", "OTA partition unavailable");
    if (req->content_len < OTA_PREFIX_SIZE || req->content_len > partition->size)
        return reject(req, "400 Bad Request", "Upload the application .bin, within the OTA slot size");
    char expected_sha[65];
    if (httpd_req_get_hdr_value_len(req, "X-Firmware-SHA256") != 64 ||
        httpd_req_get_hdr_value_str(req, "X-Firmware-SHA256", expected_sha, sizeof(expected_sha)) != ESP_OK)
        return reject(req, "400 Bad Request", "X-Firmware-SHA256 header required");
    uint8_t buffer[1024];
    if (!receive_exact(req, buffer, OTA_PREFIX_SIZE))
        return reject(req, "408 Request Timeout", "Incomplete firmware header");
    esp_image_header_t image;
    esp_app_desc_t app;
    memcpy(&image, buffer, sizeof(image));
    memcpy(&app, buffer + sizeof(image) + sizeof(esp_image_segment_header_t), sizeof(app));
    if (image.magic != ESP_IMAGE_HEADER_MAGIC || image.chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID ||
        app.magic_word != ESP_APP_DESC_MAGIC_WORD ||
        memcmp(app.project_name, esp_app_get_description()->project_name, sizeof(app.project_name)) != 0)
        return reject(req, "400 Bad Request", "Firmware chip/project mismatch; do not upload a merged image");

    ota_busy = true;
    usb_midi_reset_output();
    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(partition, req->content_len, &handle);
    if (err != ESP_OK) {
        ota_busy = false;
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        return reject(req, "500 Internal Server Error", "Cannot start OTA");
    }
    ESP_LOGI(TAG, "OTA writing %u bytes to %s", (unsigned)req->content_len, partition->label);
    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);
    size_t total = 0;
    size_t count = OTA_PREFIX_SIZE;
    while (true) {
        mbedtls_sha256_update(&sha, buffer, count);
        err = esp_ota_write(handle, buffer, count);
        if (err != ESP_OK) break;
        total += count;
        if (total == req->content_len) break;
        count = req->content_len - total;
        if (count > sizeof(buffer)) count = sizeof(buffer);
        if (!receive_exact(req, buffer, count)) { err = ESP_ERR_TIMEOUT; break; }
    }
    uint8_t digest[32];
    char actual_sha[65];
    mbedtls_sha256_finish(&sha, digest);
    mbedtls_sha256_free(&sha);
    for (size_t i = 0; i < sizeof(digest); i++) snprintf(actual_sha + i * 2, 3, "%02x", digest[i]);
    if (err == ESP_OK && strcmp(actual_sha, expected_sha) != 0) err = ESP_ERR_INVALID_CRC;
    if (err != ESP_OK) {
        esp_ota_abort(handle);
        ota_busy = false;
        ESP_LOGW(TAG, "OTA aborted: %s", esp_err_to_name(err));
        return reject(req, "400 Bad Request", "Firmware incomplete, corrupt, or flash write failed; current app retained");
    }
    err = esp_ota_end(handle); // Always consumes handle, including validation failure.
    if (err == ESP_OK) err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        ota_busy = false;
        ESP_LOGE(TAG, "OTA validation/boot selection failed: %s", esp_err_to_name(err));
        return reject(req, "400 Bad Request", "Firmware validation failed; current app retained");
    }
    ESP_LOGI(TAG, "OTA verified; restarting into %s", partition->label);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"rebooting\":true}");
    esp_timer_start_once(reboot_timer, 1000000);
    return ESP_OK;
}

static esp_err_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.task_priority = 3;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 8;
    config.send_wait_timeout = 8;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) return err;
    const httpd_uri_t routes[] = {
        {.uri="/", .method=HTTP_GET, .handler=index_handler},
        {.uri="/api/status", .method=HTTP_GET, .handler=status_handler},
        {.uri="/api/logs", .method=HTTP_GET, .handler=logs_handler},
        {.uri="/api/trace", .method=HTTP_POST, .handler=trace_handler},
        {.uri="/api/usb/reconnect", .method=HTTP_POST, .handler=reconnect_handler},
        {.uri="/api/ota", .method=HTTP_POST, .handler=ota_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        err = httpd_register_uri_handler(server, &routes[i]);
        if (err != ESP_OK) { httpd_stop(server); server = NULL; return err; }
    }
    return ESP_OK;
}

static void connect_again(void *arg)
{
    (void)arg;
    if (!wifi_connected) esp_wifi_connect();
}

static void reboot(void *arg)
{
    (void)arg;
    esp_restart();
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        atomic_fetch_add(&wifi_disconnects, 1);
        wifi_event_sta_disconnected_t *event = data;
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%u; retrying in 3 seconds", event->reason);
        esp_timer_stop(reconnect_timer);
        esp_timer_start_once(reconnect_timer, 3000000);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = true;
        esp_timer_stop(reconnect_timer);
        ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "Remote debug: http://" IPSTR "/", IP2STR(&event->ip_info.ip));
    }
}

static void health_task(void *arg)
{
    (void)arg;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    bool pending = esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
    int64_t start = esp_timer_get_time();
    while (pending) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int64_t elapsed = esp_timer_get_time() - start;
        if (elapsed >= 10000000 && app_ready && wifi_connected && server) {
            esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA startup health check: %s", esp_err_to_name(err));
            if (err == ESP_OK) break;
        }
        if (elapsed > 90000000) {
            ESP_LOGE(TAG, "OTA health check failed; rolling back");
            esp_ota_mark_app_invalid_rollback_and_reboot();
            break;
        }
    }
    vTaskDelete(NULL);
}

void remote_debug_app_ready(void)
{
    app_ready = true;
}

esp_err_t remote_debug_init(void)
{
    esp_err_t err = nvs_flash_init_partition("netcfg");
    if (err != ESP_OK) return err;
    nvs_handle_t nvs;
    err = nvs_open_from_partition("netcfg", "bridge_net", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi is not provisioned; run tools/provision_wifi.py and flash netcfg.bin");
        return err;
    }
    char ssid[33] = {0}, password[65] = {0};
    size_t len = sizeof(ssid);
    err = nvs_get_str(nvs, "ssid", ssid, &len);
    len = sizeof(password);
    if (err == ESP_OK) err = nvs_get_str(nvs, "password", password, &len);
    len = sizeof(management_password);
    if (err == ESP_OK) err = nvs_get_str(nvs, "token", management_password, &len);
    nvs_close(nvs);
    if (err != ESP_OK || !ssid[0] || !management_password[0])
        return err == ESP_OK ? ESP_ERR_INVALID_ARG : err;
    err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    station = esp_netif_create_default_wifi_sta();
    if (!station) return ESP_ERR_NO_MEM;
    esp_netif_set_hostname(station, "usb2ble-midi");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) return err;
    // Keep credentials in the dedicated partition, out of ordinary Wi-Fi NVS.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_timer_create_args_t reconnect = {.callback=connect_again, .name="wifi_retry"};
    esp_timer_create_args_t reset = {.callback=reboot, .name="ota_restart"};
    err = esp_timer_create(&reconnect, &reconnect_timer);
    if (err == ESP_OK) err = esp_timer_create(&reset, &reboot_timer);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL);
    if (err == ESP_OK) err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL);
    if (err != ESP_OK) return err;
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, ssid, strlen(ssid));
    memcpy(config.sta.password, password, strlen(password));
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &config);
    memset(&config, 0, sizeof(config));
    memset(password, 0, sizeof(password));
    if (err == ESP_OK) err = start_server();
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) return err;
    // Keep the driver's default power saving for Wi-Fi/BLE radio coexistence.
    if (xTaskCreate(health_task, "ota_health", 3072, NULL, 2, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "Wi-Fi debug started; waiting for DHCP");
    return ESP_OK;
}
