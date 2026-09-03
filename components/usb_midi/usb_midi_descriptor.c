#include "usb_midi_descriptor.h"
#include <string.h>

static void select_interface(const usb_midi_interface_t *candidate,
                              usb_midi_interface_t *result)
{
    if ((!result->in_addr && !result->out_addr) ||
        (candidate->in_addr && candidate->out_addr &&
         !(result->in_addr && result->out_addr))) {
        *result = *candidate;
    }
}

bool usb_midi_find_interface(const uint8_t *desc, size_t len,
                             usb_midi_interface_t *result)
{
    if (!desc || !result || len < 9 || desc[0] < 9 || desc[1] != 2) {
        return false;
    }
    size_t total_len = desc[2] | ((size_t)desc[3] << 8);
    if (total_len > len || total_len < 9) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    usb_midi_interface_t candidate = {0};
    bool in_midi_interface = false;
    for (size_t pos = 0; pos < total_len;) {
        if (total_len - pos < 2 || desc[pos] < 2 || desc[pos] > total_len - pos) {
            return false;
        }
        const uint8_t *item = desc + pos;
        if (item[1] == 4) {
            if (item[0] < 9) {
                return false;
            }
            select_interface(&candidate, result);
            memset(&candidate, 0, sizeof(candidate));
            candidate.interface_num = item[2];
            candidate.interface_alt = item[3];
            in_midi_interface = item[5] == 1 && item[6] == 3 && item[7] == 0;
        } else if (item[1] == 5 && in_midi_interface) {
            if (item[0] < 7) {
                return false;
            }
            uint8_t type = item[3] & 3;
            uint16_t mps = (item[4] | ((uint16_t)item[5] << 8)) & 0x7FF;
            if ((type == 2 || type == 3) && (item[2] & 0x0F) && mps >= 4) {
                if ((item[2] & 0x80) && !candidate.in_addr) {
                    candidate.in_addr = item[2];
                    candidate.in_packet_size = mps;
                } else if (!(item[2] & 0x80) && !candidate.out_addr) {
                    candidate.out_addr = item[2];
                    candidate.out_packet_size = mps;
                }
            }
        }
        pos += item[0];
    }
    select_interface(&candidate, result);
    return result->in_addr || result->out_addr;
}
