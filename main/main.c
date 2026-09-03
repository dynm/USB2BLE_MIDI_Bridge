#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "usb_midi.h"
#include "ble_midi.h"
#include "ble_to_usb_midi.h"
#include "esp_timer.h"
#include "remote_debug.h"
#include "usb_to_ble_midi.h"

static const char *TAG = "midi_bridge";
static ble_to_usb_midi_t ble_decoder;
static uint32_t output_session;

static void ble_midi_data_callback(uint8_t *data, size_t len)
{
    remote_debug_count(DIAG_BLE_RX_PACKETS, 1);
    remote_debug_trace("BLE RX", data, len);
    uint32_t session = usb_midi_output_session();
    if (session != output_session || session == 0) {
        memset(&ble_decoder, 0, sizeof(ble_decoder));
        output_session = session;
    }
    if (session == 0) {
        return;
    }
    // One USB event per input byte is a conservative bound, including buffered
    // SysEx bytes. The characteristic currently accepts at most 100 bytes.
    uint8_t events[4 * (100 + 2)];
    size_t events_len;
    if (!ble_to_usb_midi_decode(&ble_decoder, data, len,
                                events, sizeof(events), &events_len)) {
        remote_debug_count(DIAG_BLE_PARSE_ERRORS, 1);
        ESP_LOGW(TAG, "Discarding malformed or oversized BLE MIDI packet");
        usb_midi_reset_output();
        return;
    }
    if (events_len == 0) {
        return;
    }
    esp_err_t err = usb_midi_send_data(events, events_len, session);
    if (err == ESP_OK) {
        remote_debug_count(DIAG_USB_TX_QUEUED, events_len / 4);
    } else {
        remote_debug_count(DIAG_USB_QUEUE_ERRORS, 1);
        memset(&ble_decoder, 0, sizeof(ble_decoder));
        if (err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "BLE to USB MIDI queue failed: %s", esp_err_to_name(err));
            usb_midi_reset_output();
        }
    }
}

static void ble_midi_connection_callback(bool connected)
{
    memset(&ble_decoder, 0, sizeof(ble_decoder));
    if (!connected) {
        usb_midi_reset_output();
    }
}

static void forward_usb_midi_event(const uint8_t *event)
{
    remote_debug_count(DIAG_USB_RX_EVENTS, 1);
    remote_debug_trace("USB RX", event, 4);
    // BLE-MIDI timestamps are absolute milliseconds modulo 8192, not 320 us.
    uint16_t timestamp = (esp_timer_get_time() / 1000) & 0x1FFF;
    uint8_t ble_data[7];
    size_t len = usb_to_ble_midi_encode(event, timestamp, ble_data);
    if (!len) return;
    esp_err_t err = ble_midi_send_data(ble_data, len);
    if (err == ESP_OK) {
        remote_debug_count(DIAG_BLE_TX_ACCEPTED, 1);
        remote_debug_trace("BLE TX", ble_data, len);
    } else if (err == ESP_ERR_INVALID_STATE) {
        remote_debug_count(DIAG_BLE_TX_NOT_READY, 1);
    } else {
        remote_debug_count(DIAG_BLE_TX_ERRORS, 1);
        ESP_LOGW(TAG, "USB to BLE send failed: %s", esp_err_to_name(err));
    }
}

// Modified USB MIDI callback function
static void usb_midi_data_callback(const uint8_t* data, size_t len) {
    if (data == NULL || len < 4) {
        ESP_LOGW(TAG, "Ignoring short USB MIDI transfer: %u bytes", (unsigned)len);
        return;
    }

    size_t packet_count = len / 4;
    for (size_t i = 0; i < packet_count; i++) {
        forward_usb_midi_event(&data[i * 4]);
    }

    if ((len % 4) != 0) {
        ESP_LOGW(TAG, "USB MIDI transfer had %u trailing byte(s)", (unsigned)(len % 4));
    }
}

void app_main(void)
{
    remote_debug_capture_logs();
    ESP_LOGI(TAG, "MIDI Bridge Starting");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ble_midi_set_callback(ble_midi_data_callback);
    ble_midi_set_connection_callback(ble_midi_connection_callback);

    // Initialize BLE MIDI
    ESP_ERROR_CHECK(ble_midi_init());
    ESP_LOGI(TAG, "BLE MIDI Initialization Completed");
    
    // Initialize USB MIDI
    usb_midi_config_t usb_config = {
        .data_callback = usb_midi_data_callback,
        .task_priority = 5
    };
    ESP_ERROR_CHECK(usb_midi_init(&usb_config));
    ESP_LOGI(TAG, "USB MIDI Initialization Completed");

    // MIDI keeps working if network provisioning or Wi-Fi is unavailable.
    ret = remote_debug_init();
    if (ret != ESP_OK) ESP_LOGW(TAG, "Network debug unavailable: %s", esp_err_to_name(ret));
    remote_debug_app_ready();
    ESP_LOGI(TAG, "Waiting for device connection...");
}
