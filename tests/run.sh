#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")/.."
binary=$(mktemp "${TMPDIR:-/tmp}/usb2ble-midi-test.XXXXXX")
trap 'rm -f "$binary"' EXIT HUP INT TERM
${CC:-cc} -std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined \
  -I main -I components/usb_midi \
  tests/test_midi.c main/ble_to_usb_midi.c main/usb_to_ble_midi.c components/usb_midi/usb_midi_descriptor.c \
  -o "$binary"
"$binary"
