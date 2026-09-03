#include "usb_to_ble_midi.h"

size_t usb_to_ble_midi_encode(const uint8_t event[4], uint16_t timestamp_ms,
                              uint8_t output[7])
{
    static const uint8_t lengths[16] = {0,0,2,3,3,1,2,3,3,3,3,3,2,2,3,1};
    size_t midi_len = lengths[event[0] & 0xF];
    if (!midi_len) return 0;
    size_t len = 0;
    output[len++] = 0x80 | ((timestamp_ms >> 7) & 0x3F);
    for (size_t i = 1; i <= midi_len; i++) {
        // Continuation data follows the header directly. F7, like every MIDI
        // status byte, needs its own timestamp even at the end of a USB event.
        if (event[i] & 0x80) output[len++] = 0x80 | (timestamp_ms & 0x7F);
        output[len++] = event[i];
    }
    return len;
}
