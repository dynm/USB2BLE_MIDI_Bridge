#pragma once
#include <stddef.h>
#include <stdint.h>

// One USB-MIDI 1.0 event to one BLE packet, including SysEx continuation/EOX.
// output must have room for 7 bytes. Returns zero for reserved CINs.
size_t usb_to_ble_midi_encode(const uint8_t event[4], uint16_t timestamp_ms,
                              uint8_t output[7]);
