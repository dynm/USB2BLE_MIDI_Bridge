#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    DIAG_USB_RX_EVENTS,
    DIAG_BLE_TX_ACCEPTED,
    DIAG_BLE_TX_NOT_READY,
    DIAG_BLE_TX_ERRORS,
    DIAG_BLE_RX_PACKETS,
    DIAG_USB_TX_QUEUED,
    DIAG_BLE_PARSE_ERRORS,
    DIAG_USB_QUEUE_ERRORS,
    DIAG_COUNTER_COUNT,
} diag_counter_t;

// Install bounded in-memory log capture before initializing other components.
void remote_debug_capture_logs(void);
// Credentials are read from the private netcfg NVS partition, never the app.
esp_err_t remote_debug_init(void);
void remote_debug_app_ready(void);
void remote_debug_count(diag_counter_t counter, uint32_t amount);
void remote_debug_trace(const char *direction, const uint8_t *data, size_t len);
