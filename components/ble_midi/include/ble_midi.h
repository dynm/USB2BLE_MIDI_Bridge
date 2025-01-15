#ifndef BLE_MIDI_H
#define BLE_MIDI_H

#include "esp_err.h"
#include <stdint.h>

#define MIDI_NUM_HANDLE 4

// MIDI message callback function type
typedef void (*midi_callback_t)(uint8_t *data, size_t len);

// Initialize BLE MIDI service
esp_err_t ble_midi_init(void);

// Send MIDI message
esp_err_t ble_midi_send_message(uint8_t *data, size_t len);
esp_err_t ble_midi_send_data(uint8_t* data, uint16_t length);

// Set MIDI message receive callback function
void ble_midi_set_callback(midi_callback_t callback);

#endif 