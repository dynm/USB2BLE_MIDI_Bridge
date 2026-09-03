#include "ble_to_usb_midi.h"
#include <string.h>

static bool append_event(uint8_t *output, size_t capacity, size_t *len,
                         uint8_t cin, const uint8_t *midi, size_t midi_len)
{
    if (*len > capacity || capacity - *len < 4) {
        return false;
    }
    uint8_t *event = output + *len;
    memset(event, 0, 4);
    event[0] = cin;
    memcpy(event + 1, midi, midi_len);
    *len += 4;
    return true;
}

bool ble_to_usb_midi_decode(ble_to_usb_midi_t *state,
                            const uint8_t *data, size_t len,
                            uint8_t *output, size_t capacity, size_t *output_len)
{
    if (state == NULL || output_len == NULL) {
        return false;
    }
    *output_len = 0;
    if (data == NULL || output == NULL || len < 2 || (data[0] & 0xC0) != 0x80) {
        goto invalid;
    }

    uint8_t running_status = 0;
    bool timestamp_required = true;
    size_t written = 0;
    size_t pos = 1;
    while (pos < len) {
        // SysEx continuation data has no timestamp, including after realtime.
        if (state->in_sysex && data[pos] < 0x80) {
            state->sysex[state->sysex_len++] = data[pos++];
            if (state->sysex_len == 3) {
                if (!append_event(output, capacity, &written, 0x4, state->sysex, 3)) {
                    goto invalid;
                }
                state->sysex_len = 0;
            }
            continue;
        }

        bool has_timestamp = (data[pos] & 0x80) != 0;
        if (has_timestamp) {
            // Timestamp values may equal MIDI status bytes (including F0/F7).
            if (++pos == len) {
                goto invalid;
            }
        } else if (timestamp_required) {
            goto invalid;
        }

        uint8_t status = data[pos];
        if (status >= 0x80) {
            if (!has_timestamp) {
                goto invalid;
            }
            pos++;
        } else {
            status = running_status;
            if (status == 0) {
                goto invalid;
            }
        }

        if (state->in_sysex) {
            if (status == 0xF7) {
                state->sysex[state->sysex_len++] = status;
                if (!append_event(output, capacity, &written,
                                  0x4 + state->sysex_len, state->sysex, state->sysex_len)) {
                    goto invalid;
                }
                state->in_sysex = false;
                state->sysex_len = 0;
                timestamp_required = true;
                continue;
            }
            if (status < 0xF8) {
                goto invalid;
            }
            // Preserve byte order when realtime interrupts a partial USB event.
            for (uint8_t i = 0; i < state->sysex_len; i++) {
                if (!append_event(output, capacity, &written, 0xF, &state->sysex[i], 1)) {
                    goto invalid;
                }
            }
            state->sysex_len = 0;
        } else if (status == 0xF0) {
            state->in_sysex = true;
            state->sysex[0] = status;
            state->sysex_len = 1;
            timestamp_required = true;
            continue;
        }

        uint8_t midi[3] = {status, 0, 0};
        uint8_t cin;
        size_t midi_len;
        if (status < 0xF0) {
            cin = status >> 4;
            midi_len = (cin == 0xC || cin == 0xD) ? 2 : 3;
            running_status = status;
            timestamp_required = false;
        } else {
            // BLE-MIDI preserves running status across both common and realtime
            // messages, but requires a timestamp before the next running message.
            timestamp_required = true;
            switch (status) {
            case 0xF1:
            case 0xF3:
                cin = 0x2;
                midi_len = 2;
                break;
            case 0xF2:
                cin = 0x3;
                midi_len = 3;
                break;
            case 0xF6:
                cin = 0x5;
                midi_len = 1;
                break;
            default:
                if (status < 0xF8) {
                    goto invalid;
                }
                cin = 0xF;
                midi_len = 1;
                break;
            }
        }
        if (len - pos < midi_len - 1) {
            goto invalid;
        }
        for (size_t i = 1; i < midi_len; i++) {
            if (data[pos] >= 0x80) {
                goto invalid;
            }
            midi[i] = data[pos++];
        }
        if (!append_event(output, capacity, &written, cin, midi, midi_len)) {
            goto invalid;
        }
    }
    *output_len = written;
    return true;

invalid:
    memset(state, 0, sizeof(*state));
    return false;
}
