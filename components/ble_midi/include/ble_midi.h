#ifndef BLE_MIDI_H
#define BLE_MIDI_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MIDI_NUM_HANDLE 4

// Raw BLE-MIDI characteristic value, including header and timestamps.
typedef void (*midi_callback_t)(uint8_t *data, size_t len);
typedef void (*ble_midi_connection_callback_t)(bool connected);

// Initialize BLE MIDI service
esp_err_t ble_midi_init(void);

// Send MIDI message
esp_err_t ble_midi_send_message(uint8_t *data, size_t len);
esp_err_t ble_midi_send_data(uint8_t* data, uint16_t length);

// Set MIDI message receive callback function
void ble_midi_set_callback(midi_callback_t callback);
void ble_midi_set_connection_callback(ble_midi_connection_callback_t callback);

typedef struct {
    bool connected;
    bool notifications_enabled;
    bool advertising;
    uint16_t payload_mtu;
    uint32_t notify_submitted;
    uint32_t notify_errors;
    uint32_t writes_received;
    uint32_t disconnects;
} ble_midi_status_t;

void ble_midi_get_status(ble_midi_status_t *status);

#endif
