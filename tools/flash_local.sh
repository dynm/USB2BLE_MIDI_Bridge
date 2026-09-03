#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")/.."
port=${1:?Usage: sh tools/flash_local.sh SERIAL_PORT}
python tools/provision_wifi.py
python -m esptool --chip esp32s3 -p "$port" -b 460800 \
  --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_size 8MB --flash_freq 80m \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/usb2ble_midi_bridge.bin \
  0x610000 build/ota_data_initial.bin \
  0x612000 build/netcfg.bin
