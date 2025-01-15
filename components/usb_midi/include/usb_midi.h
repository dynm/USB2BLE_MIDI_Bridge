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

#ifdef __cplusplus
}
#endif
