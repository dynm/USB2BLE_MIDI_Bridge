#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    bool in_sysex;
    uint8_t sysex[3];
    uint8_t sysex_len;
} ble_to_usb_midi_t;

// Decode one BLE-MIDI characteristic value into USB-MIDI 1.0 cable-0 events.
// Only SysEx state survives packet boundaries. Timestamps are consumed; events
// are forwarded immediately in arrival order. On failure, discard all output
// and reset the parser so malformed/truncated input cannot affect later notes.
bool ble_to_usb_midi_decode(ble_to_usb_midi_t *state,
                            const uint8_t *data, size_t len,
                            uint8_t *output, size_t capacity, size_t *output_len);
