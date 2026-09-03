#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t interface_num;
    uint8_t interface_alt;
    uint8_t in_addr;
    uint8_t out_addr;
    uint16_t in_packet_size;
    uint16_t out_packet_size;
} usb_midi_interface_t;

// Prefer a bidirectional MIDI 1.0 interface; otherwise retain a one-way port.
bool usb_midi_find_interface(const uint8_t *desc, size_t len,
                             usb_midi_interface_t *result);
