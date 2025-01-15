#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "usb_midi.h"
#include "ble_midi.h"
#include "esp_timer.h"

static const char *TAG = "midi_bridge";

// Timestamp related definitions
#define MIDI_TIMESTAMP_PERIOD_US 320   // BLE MIDI timestamp unit (microseconds)
#define MAX_TIMESTAMP_VALUE 0x1FFF     // Maximum value for 13-bit timestamp

// Timestamp control structure
typedef struct {
    int64_t last_time_us;     // Last timestamp point
    uint16_t last_timestamp;   // Last sent timestamp value
    bool is_initialized;       // Whether initialized
} timestamp_control_t;

static timestamp_control_t ts_control = {
    .last_time_us = 0,
    .last_timestamp = 0,
    .is_initialized = false
};

// Get current timestamp
static uint16_t get_current_timestamp(void) {
    int64_t current_time = esp_timer_get_time();
    uint16_t new_timestamp;

    if (!ts_control.is_initialized) {
        ts_control.last_time_us = current_time;
        ts_control.last_timestamp = 0;
        ts_control.is_initialized = true;
        return 0;
    }

    // Calculate time difference (microseconds)
    int64_t time_diff = current_time - ts_control.last_time_us;
    
    // Convert time difference to timestamp increment
    uint32_t timestamp_diff = time_diff / MIDI_TIMESTAMP_PERIOD_US;
    
    // Calculate new timestamp value
    new_timestamp = (ts_control.last_timestamp + timestamp_diff) & MAX_TIMESTAMP_VALUE;
    
    // Update status
    ts_control.last_timestamp = new_timestamp;
    ts_control.last_time_us = current_time;
    
    return new_timestamp;
}

// Pack timestamp into BLE MIDI message
static void pack_timestamp(uint16_t timestamp, uint8_t* ble_data) {
    ble_data[0] = 0x80 | ((timestamp >> 7) & 0x3F);  // High 7 bits
    ble_data[1] = 0x80 | (timestamp & 0x7F);         // Low 7 bits
}

// Modified USB MIDI callback function
static void usb_midi_data_callback(const uint8_t* data, size_t len) {
    ESP_LOGI(TAG, "USB MIDI packet parsing:");
    ESP_LOGI(TAG, "Cable Number: 0x%01x", (data[0] >> 4) & 0x0F);
    ESP_LOGI(TAG, "Code Index: 0x%01x", data[0] & 0x0F);
    ESP_LOGI(TAG, "MIDI Event: %02x %02x %02x", data[1], data[2], data[3]);
    
    // Get current timestamp
    uint16_t current_ts = get_current_timestamp();
    
    // Prepare BLE MIDI packet
    uint8_t ble_data[5];
    pack_timestamp(current_ts, ble_data);
    memcpy(&ble_data[2], &data[1], 3);  // Copy MIDI event data
    
    // Send data
    esp_err_t send_success = ble_midi_send_data(ble_data, 5);
    if (send_success == ESP_OK) {
        ESP_LOGI(TAG, "Forwarded to BLE MIDI (timestamp: 0x%04x)", current_ts);
    } else {
        ESP_LOGE(TAG, "Send failed");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "MIDI Bridge Starting");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
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
    
    ESP_LOGI(TAG, "Waiting for device connection...");
} 