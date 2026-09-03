#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ble_to_usb_midi.h"
#include "usb_midi_descriptor.h"
#include "usb_to_ble_midi.h"

static unsigned checks;

static size_t unhex(const char *hex, uint8_t *bytes)
{
    size_t len = 0;
    unsigned byte;
    int consumed;
    while (sscanf(hex, " %2x%n", &byte, &consumed) == 1) {
        bytes[len++] = byte;
        hex += consumed;
    }
    return len;
}

static void packet(ble_to_usb_midi_t *state, const char *input, const char *expected)
{
    uint8_t data[512], result[2048], want[2048];
    size_t len = unhex(input, data);
    size_t want_len = unhex(expected, want);
    size_t result_len = 0;
    assert(ble_to_usb_midi_decode(state, data, len, result, sizeof(result), &result_len));
    assert(result_len == want_len);
    assert(memcmp(result, want, want_len) == 0);
    checks++;
}

static void reject(ble_to_usb_midi_t *state, const char *input)
{
    uint8_t data[512], result[2048];
    size_t len = unhex(input, data), result_len = 123;
    assert(!ble_to_usb_midi_decode(state, data, len, result, sizeof(result), &result_len));
    assert(result_len == 0 && !state->in_sysex && state->sysex_len == 0);
    checks++;
}

static void test_midi(void)
{
    ble_to_usb_midi_t state = {0};
    packet(&state, "80 80 90 3c 64", "09 90 3c 64"); // Middle C on
    packet(&state, "80 81 80 3c 40", "08 80 3c 40"); // Middle C off
    packet(&state, "80 80 9f 3c 00", "09 9f 3c 00"); // Velocity-zero note-off, ch 16
    packet(&state, "80 80 b0 40 7f 81 b0 40 00", "0b b0 40 7f 0b b0 40 00");
    packet(&state, "80 80 a1 3c 40 81 c2 05 82 d3 50 83 e4 00 40",
                   "0a a1 3c 40 0c c2 05 00 0d d3 50 00 0e e4 00 40");
    // Timestamp bytes that look like MIDI status, including wrap to 00.
    packet(&state, "bf f8 90 3c 64 ff 90 40 64 80 90 43 64",
                   "09 90 3c 64 09 90 40 64 09 90 43 64");
    packet(&state, "80 80 90 3c 64 40 65 82 43 66",
                   "09 90 3c 64 09 90 40 65 09 90 43 66");
    packet(&state, "80 80 c0 02 03 81 04", "0c c0 02 00 0c c0 03 00 0c c0 04 00");
    packet(&state, "80 80 f1 12 81 f2 01 02 82 f3 03 83 f6",
                   "02 f1 12 00 03 f2 01 02 02 f3 03 00 05 f6 00 00");
    packet(&state, "80 80 f8 81 fa 82 fb 83 fc 84 fe 85 ff",
                   "0f f8 00 00 0f fa 00 00 0f fb 00 00 0f fc 00 00 0f fe 00 00 0f ff 00 00");
    // Unlike serial MIDI, BLE retains running status across System Common.
    packet(&state, "80 80 90 3c 64 81 f1 00 82 40 64 83 f8 84 43 64",
                   "09 90 3c 64 02 f1 00 00 09 90 40 64 0f f8 00 00 09 90 43 64");
    // USB SysEx endings with 1, 2 and 3 meaningful bytes.
    packet(&state, "80 80 f0 7e 00 81 f7", "04 f0 7e 00 05 f7 00 00");
    packet(&state, "80 80 f0 81 f7", "06 f0 f7 00");
    packet(&state, "80 80 f0 7d 81 f7", "07 f0 7d f7");
    packet(&state, "80 80 f0 7e", "");
    packet(&state, "81 7f 09 01", "04 f0 7e 7f");
    packet(&state, "82 80 f7 81 90 3c 64", "07 09 01 f7 09 90 3c 64");
    // Realtime interrupts SysEx, including partial USB events and packet edges.
    packet(&state, "80 80 f0 7d 81 f8 01 02", "0f f0 00 00 0f 7d 00 00 0f f8 00 00");
    packet(&state, "81 03 80 fa 04 81 f7", "04 01 02 03 0f fa 00 00 06 04 f7 00");
    packet(&state, "80 80 f0 01 02", "04 f0 01 02");
    packet(&state, "80 81 f8", "0f f8 00 00");
    packet(&state, "80 01 82 f7", "06 01 f7 00");

    reject(&state, "");
    reject(&state, "80");
    reject(&state, "40 80 90 3c 64");
    reject(&state, "c0 80 90 3c 64");
    reject(&state, "80 90 3c 64"); // Missing timestamp/status
    reject(&state, "80 80 90 3c");
    reject(&state, "80 80 90 3c 64 81"); // Reject whole packet, no partial output
    reject(&state, "80 80 90 3c 81 f8 64"); // Realtime must be deinterleaved
    reject(&state, "80 80 f4");
    reject(&state, "80 80 f7");
    reject(&state, "80 01 02"); // Orphan SysEx continuation
    packet(&state, "80 80 90 3c 64", "09 90 3c 64");
    reject(&state, "80 80 40 64"); // Running status cannot cross BLE packets
    reject(&state, "80 80 90 3c 64 81 f8 40 64");
    packet(&state, "80 80 f0 7d", "");
    reject(&state, "80 80 90 3c 64"); // Interrupted SysEx clears saved state
    packet(&state, "80 80 90 3c 64", "09 90 3c 64");

    uint8_t data[] = {0x80, 0x80, 0x90, 60, 100};
    uint8_t result[4] = {0};
    size_t result_len = 123;
    assert(!ble_to_usb_midi_decode(&state, data, sizeof(data), result, 3, &result_len));
    assert(result_len == 0);
    assert(!ble_to_usb_midi_decode(&state, NULL, 5, result, 4, &result_len));
    checks += 2;
}

static void test_long_sysex(void)
{
    // Exercise every BLE split boundary against the exact recovered MIDI stream.
    for (size_t split = 1; split <= 31; split++) {
        ble_to_usb_midi_t state = {0};
        uint8_t original[131] = {0xF0};
        for (size_t i = 1; i < sizeof(original) - 1; i++) original[i] = (i - 1) & 0x7F;
        original[sizeof(original) - 1] = 0xF7;
        uint8_t recovered[256];
        size_t recovered_len = 0, consumed = 0;
        while (consumed < sizeof(original)) {
            uint8_t input[64] = {0x80}, events[256];
            size_t input_len = 1, count = 0, events_len;
            while (count++ < split && consumed < sizeof(original)) {
                uint8_t b = original[consumed++];
                if (b >= 0x80) input[input_len++] = 0x81;
                input[input_len++] = b;
            }
            assert(ble_to_usb_midi_decode(&state, input, input_len, events, sizeof(events), &events_len));
            for (size_t i = 0; i < events_len; i += 4) {
                size_t n = events[i] == 4 ? 3 : events[i] - 4;
                assert(n >= 1 && n <= 3);
                memcpy(recovered + recovered_len, events + i + 1, n);
                recovered_len += n;
            }
        }
        assert(!state.in_sysex);
        assert(recovered_len == sizeof(original));
        assert(memcmp(recovered, original, sizeof(original)) == 0);
        checks++;
    }
}

static void test_descriptors(void)
{
    uint8_t desc[512];
    usb_midi_interface_t port;
    const char *config = "09 02 29 00 01 01 00 80 32 "
                         "09 04 02 00 02 01 03 00 00 "
                         "09 05 02 02 40 00 00 00 00 "
                         "05 25 01 01 01 "
                         "09 05 83 02 40 00 00 00 00";
    size_t len = unhex(config, desc);
    assert(usb_midi_find_interface(desc, len, &port));
    assert(port.interface_num == 2 && port.out_addr == 2 && port.in_addr == 0x83);
    assert(port.in_packet_size == 64 && port.out_packet_size == 64);
    checks++;
    // IN-first ordering with a 7-byte standard endpoint descriptor.
    len = unhex("09 02 20 00 01 01 00 80 32 09 04 01 00 02 01 03 00 00 "
                "07 05 81 02 40 00 00 07 05 02 02 20 00 00", desc);
    assert(usb_midi_find_interface(desc, len, &port));
    assert(port.in_addr == 0x81 && port.out_addr == 2 && port.out_packet_size == 32);
    checks++;
    // Never mix endpoints from different interfaces or alternate settings.
    len = unhex("09 02 29 00 01 01 00 80 32 09 04 01 00 01 01 03 00 00 "
                "07 05 81 02 40 00 00 09 04 01 01 01 01 03 00 00 "
                "07 05 02 02 40 00 00", desc);
    assert(usb_midi_find_interface(desc, len, &port));
    assert(port.in_addr == 0x81 && port.out_addr == 0 && port.interface_alt == 0);
    checks++;
    // Skip audio/vendor/MIDI 2.0 interfaces and support OUT-only instruments.
    len = unhex("09 02 29 00 02 01 00 80 32 09 04 00 00 01 01 03 02 00 "
                "07 05 81 02 40 00 00 09 04 01 00 01 01 03 00 00 "
                "07 05 02 02 40 00 00", desc);
    assert(usb_midi_find_interface(desc, len, &port));
    assert(port.interface_num == 1 && port.in_addr == 0 && port.out_addr == 2);
    checks++;
    // A later bidirectional interface is preferred to a one-way interface.
    len = unhex("09 02 30 00 02 01 00 80 32 09 04 00 00 01 01 03 00 00 "
                "07 05 81 02 40 00 00 09 04 01 00 02 01 03 00 00 "
                "07 05 82 02 40 00 00 07 05 03 02 40 00 00", desc);
    assert(usb_midi_find_interface(desc, len, &port));
    assert(port.interface_num == 1 && port.in_addr == 0x82 && port.out_addr == 3);
    checks++;
    assert(!usb_midi_find_interface(desc, len - 1, &port));
    desc[9] = 0;
    assert(!usb_midi_find_interface(desc, len, &port));
    len = unhex(config, desc);
    desc[18] = 6;
    assert(!usb_midi_find_interface(desc, len, &port));
    checks += 3;
}

static void test_usb_to_ble(void)
{
    const uint8_t events[][4] = {
        {9,0x90,60,100}, {8,0x80,60,0}, {0xC,0xC0,5,0},
        {4,0xF0,0x7E,0x7F}, {4,9,1,0}, {6,0,0xF7,0},
        {7,0xF0,0x7D,0xF7}, {0xF,0xF8,0,0},
    };
    const char *expected[] = {
        "83 f4 90 3c 64", "83 f4 80 3c 00", "83 f4 c0 05",
        "83 f4 f0 7e 7f", "83 09 01 00", "83 00 f4 f7",
        "83 f4 f0 7d f4 f7", "83 f4 f8",
    };
    for (size_t i = 0; i < sizeof(events)/sizeof(events[0]); i++) {
        uint8_t output[7], want[7];
        size_t len = usb_to_ble_midi_encode(events[i], 500, output);
        assert(len == unhex(expected[i], want));
        assert(memcmp(output, want, len) == 0);
        checks++;
    }
    // Round-trip complete SysEx through the two transport codecs.
    ble_to_usb_midi_t state = {0};
    uint8_t output[7], roundtrip[12];
    for (size_t i = 3; i <= 5; i++) {
        size_t len = usb_to_ble_midi_encode(events[i], 8191, output), recovered;
        assert(ble_to_usb_midi_decode(&state, output, len, roundtrip, sizeof(roundtrip), &recovered));
        assert(recovered == 4 && memcmp(roundtrip, events[i], 4) == 0);
    }
    assert(!state.in_sysex);
    assert(usb_to_ble_midi_encode((uint8_t[]){0,0,0,0}, 0, output) == 0);
    checks += 2;
}

int main(void)
{
    test_midi();
    test_long_sysex();
    test_descriptors();
    test_usb_to_ble();
    printf("Passed %u MIDI packet, SysEx split, and USB descriptor checks.\n", checks);
    return 0;
}
