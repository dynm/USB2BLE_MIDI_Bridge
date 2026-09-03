#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief USB MIDI data callback function type
 * @param data MIDI data buffer
 * @param len Data length
 */
typedef void (*usb_midi_callback_t)(const uint8_t* data, size_t len);

/**
 * @brief USB MIDI configuration structure
 */
typedef struct {
    usb_midi_callback_t data_callback;  ///< MIDI data callback function
    uint8_t task_priority;              ///< USB MIDI task priority
} usb_midi_config_t;

/**
 * @brief Initialize USB MIDI
 * @param config USB MIDI configuration
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_STATE Already initialized
 *     - ESP_ERR_NO_MEM Out of memory
 */
esp_err_t usb_midi_init(usb_midi_config_t *config);

/**
 * @brief Deinitialize USB MIDI
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_STATE Not initialized
 */
esp_err_t usb_midi_deinit(void);

/**
 * @brief Check if USB MIDI device is connected
 * @return true if connected, false if not connected
 */
bool usb_midi_device_connected(void);

/**
 * @brief Queue USB-MIDI 1.0 event packets for the discovered OUT endpoint.
 * Copies data before returning; len must be a nonzero multiple of 4.
 * ESP_OK means queued, not yet delivered. Returns ESP_ERR_INVALID_STATE if no
 * writable device is connected, or ESP_ERR_NO_MEM if the entire batch will not
 * fit in the bounded queue (no partial batch is queued).
 * session must match usb_midi_output_session(), preventing events decoded for
 * a removed piano from being sent to a newly connected device.
 */
esp_err_t usb_midi_send_data(const uint8_t *data, size_t len, uint32_t session);

/** Current writable connection ID, or zero when output is unavailable. */
uint32_t usb_midi_output_session(void);

/** Drop queued output and send sustain-off/all-sound-off/all-notes-off on all
 * channels. Used on BLE disconnect or dropped output to prevent stuck notes. */
void usb_midi_reset_output(void);

typedef struct {
    bool connected;
    bool rx_active;
    bool tx_active;
    bool tx_failed;
    uint8_t in_endpoint;
    uint8_t out_endpoint;
    uint32_t session;
    uint32_t queued_events;
    uint32_t rx_transfers;
    uint32_t rx_bytes;
    uint32_t tx_transfers;
    uint32_t tx_bytes;
    uint32_t rx_errors;
    uint32_t tx_errors;
} usb_midi_status_t;

void usb_midi_get_status(usb_midi_status_t *status);
// Reopen the attached device from the USB client task (safe from HTTP context).
esp_err_t usb_midi_reconnect(void);

#ifdef __cplusplus
}
#endif
