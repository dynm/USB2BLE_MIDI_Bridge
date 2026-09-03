# USB2BLE MIDI Bridge

This project bridges USB MIDI and Bluetooth MIDI (BLE MIDI) in both directions:

- USB → BLE: play the piano to send MIDI to a phone, tablet, or computer.
- BLE → USB: send MIDI from an app or DAW to play the piano's built-in sounds.

Tested on Yamaha YDP-144. Both MIDI directions, Wi-Fi diagnostics, and an OTA
update with reboot and startup validation have been confirmed on hardware.

## Hardware Requirements

- ESP32-S3 DevKitC board
- USB Type A breakout board
- Yamaha Digital Piano (with USB MIDI support)

## Wiring Instructions

Connect the USB Type A breakout board to ESP32-S3 as follows:

- USB D+ → GPIO20
- USB D- → GPIO19
- USB VCC → 5V
- USB GND → GND

## Usage

1. Connect the hardware according to the wiring instructions
2. Flash the program to ESP32-S3
3. Connect a USB cable between the piano and the USB breakout board
   - Piano side: Standard USB-B port
   - Breakout board side: USB-A port
4. Turn on Bluetooth on your mobile device/tablet and search for the MIDI device
5. Select `USB2BLE MIDI Bridge` as the MIDI input and/or output in your MIDI app

### Play the piano from BLE MIDI

Connect to `USB2BLE MIDI Bridge` using your app's Bluetooth MIDI connection screen
(on macOS, use Audio MIDI Setup → MIDI Studio → Bluetooth). Choose the bridge as
the app/DAW's **MIDI output**, then send notes on a channel the piano receives
(start with channel 1). The piano generates the sound; this connection carries
MIDI performance commands, not recorded audio.

For a quick test, send `90 3C 64` (channel 1, middle C, velocity 100), then
`80 3C 00` about half a second later. These are ordinary MIDI bytes for a MIDI
app/API. A BLE GATT test tool must include BLE-MIDI framing; example characteristic
writes are `80 80 90 3C 64` and, 500 ms later, `83 F4 80 3C 00` to the MIDI I/O
characteristic `7772E5DB-3868-4112-A1A9-F2669D106BF3`.

The reverse path supports all 16 MIDI channels, Note On/Off, control changes
(including sustain), program changes, pressure, pitch bend, system common and
realtime messages, running status within a BLE packet, and SysEx across packets.
It uses USB-MIDI 1.0 cable 0 on the first OUT endpoint of the selected MIDI
interface. The piano must expose a MIDI OUT endpoint and support receiving MIDI.
Devices with only an IN endpoint continue to support USB → BLE.

Events are forwarded immediately in received order; BLE timestamps are decoded
as framing but are not used to schedule playback. The bridge requests a
7.5–15 ms BLE connection interval; the phone/computer decides the final interval.
On BLE disconnect or invalid/overflowed input, pending output is cleared and
sustain-off, All Sound Off, and All Notes Off are sent on all channels. USB
disconnect discards pending output and retires transfers before releasing the
device. USB output errors are logged and leave the input path running. Reopen
the USB device from the debug page or reconnect its cable to recover.

If there is no sound, verify the MIDI output selection, piano volume/receive
channel, and the startup log's `OUT=0x..` endpoint. Disable app/DAW MIDI Thru if
it echoes received piano notes back to the bridge and causes doubled notes.

### Development checks

```bash
sh tests/run.sh
idf.py build
```

The host tests exercise both MIDI conversion directions, SysEx framing, and USB
endpoint discovery with address and undefined-behavior sanitizers. Hardware
validation should cover receiving notes and sustain from BLE while playing the
piano, BLE disconnect with a held note,
and unplugging/reconnecting USB during playback.

Protocol references: [BLE-MIDI specification](https://midi.org/midi-over-bluetooth-low-energy-ble-midi)
and [USB-MIDI 1.0 specification](https://www.usb.org/sites/default/files/midi10.pdf).

## Build and Flash

Use ESP-IDF 5.3.x with the ESP32-S3 target and an 8 MB flash board. For the first
installation with Wi-Fi diagnostics and OTA, follow
[First installation / private configuration](#first-installation--private-configuration)
below. A standard build and flash runs the MIDI bridge without provisioning Wi-Fi:

```bash
idf.py build
idf.py -p /dev/your_usb_to_serial_port flash
```

## Flash Prebuilt Release Firmware

Download firmware files from the latest GitHub Release.

For a device already configured for Wi-Fi and OTA, download the application file
`usb2ble_midi_bridge.bin` and follow [Update firmware over Wi-Fi](#update-firmware-over-wi-fi).
The serial flashing methods below install the bootloader and OTA partition table.
Public firmware contains no Wi-Fi or management credentials; configure a new
device using the [local provisioning instructions](#first-installation--private-configuration).

### Recommended: single merged binary

Download:

- `usb2ble_midi_bridge_merged.bin`

Install `esptool` if needed:

```bash
python -m pip install esptool
```

Flash it at offset `0x0`:

```bash
python -m esptool --chip esp32s3 -p /dev/your_usb_to_serial_port -b 460800 \
  --before default_reset --after hard_reset \
  write_flash --flash_mode dio --flash_size 8MB --flash_freq 80m \
  0x0 usb2ble_midi_bridge_merged.bin
```

On macOS the serial port is usually like `/dev/cu.usbserial-120` or `/dev/cu.usbmodem*`.

### Advanced: separate binaries

Download:

- `bootloader.bin`
- `partition-table.bin`
- `ota_data_initial.bin`
- `usb2ble_midi_bridge.bin`

Flash them with explicit offsets:

```bash
python -m esptool --chip esp32s3 -p /dev/your_usb_to_serial_port -b 460800 \
  --before default_reset --after hard_reset \
  write_flash --flash_mode dio --flash_size 8MB --flash_freq 80m \
  0x0 bootloader.bin \
  0x8000 partition-table.bin \
  0x10000 usb2ble_midi_bridge.bin \
  0x610000 ota_data_initial.bin
```

## Wi-Fi logs and OTA

The firmware joins the configured 2.4 GHz Wi-Fi network and serves a local debug
page at the DHCP address printed on the serial console: `http://DEVICE_IP/`.
The page displays USB endpoints, receive counters, BLE connection/subscription
state, forwarding/drop counters, heap, and the current OTA slot. MIDI tracing
can be enabled temporarily to inspect the actual USB and BLE packet bytes.
The log history is a bounded RAM buffer and resets at reboot.

Without local network configuration, the bridge continues to work over USB and
BLE, while the Wi-Fi debug server and OTA remain disabled.

### First installation / private configuration

Create `.wifi-pass` locally:

```ini
ssid=YOUR_NETWORK
password=YOUR_WIFI_PASSWORD
```

Put the OTA management password in `.ota-pass`, either as a single line or as
`password=YOUR_OTA_PASSWORD`. If absent, provisioning generates a random password.
Use 1–128 printable ASCII characters without spaces. Both files are ignored by
Git. The same management password protects status, logs, tracing, USB reconnect,
and OTA. It is never printed by the tools or sent in the URL.

With ESP-IDF activated:

```bash
idf.py build
sh tools/flash_local.sh /dev/your_usb_to_serial_port
```

The initial serial flash migrates to two 3 MiB OTA slots and provisions a separate
`netcfg` NVS partition. Existing ordinary NVS offsets are preserved. Credentials
are kept out of application binaries and public release artifacts. The generated
`build/netcfg.bin` contains private configuration: keep it local. An ordinary OTA
update preserves both configuration partitions. Changing either password or the
Wi-Fi network requires updating the local files, then regenerating and flashing
only `netcfg.bin` over the board's serial USB port:

```bash
python tools/provision_wifi.py
python -m esptool --chip esp32s3 -p /dev/your_usb_to_serial_port -b 460800 \
  --before default_reset --after hard_reset write_flash \
  0x612000 build/netcfg.bin
```

The credential files, generated private CSV, and `netcfg.bin` are ignored by Git.
Release packaging includes only public firmware and flashing metadata.

### English debug page

1. Open `http://DEVICE_IP/` from a device on the same network. Find the bridge's
   address in the serial console or your router's DHCP client list.
2. Enter the management password or select your local `.ota-pass` file, then
   click **Connect**. The file is read in the browser; API requests authenticate
   with the password.
3. **Connection Status** and **Recent Logs** refresh every second. Click
   **Enable MIDI Trace** to inspect packet bytes while testing, then
   **Disable MIDI Trace** when finished.
4. Use **Reconnect USB MIDI** to reopen the piano's USB connection after a USB
   error. This briefly interrupts MIDI traffic.

The **OTA Update** section shows the upload command with the current device
address. Firmware uploads are performed by the command-line tool.

### Remote commands

The tool reads `.ota-pass` automatically:

```bash
python tools/midi_remote.py --host DEVICE_IP status
python tools/midi_remote.py --host DEVICE_IP logs --follow
python tools/midi_remote.py --host DEVICE_IP trace on
python tools/midi_remote.py --host DEVICE_IP trace off
python tools/midi_remote.py --host DEVICE_IP usb-reconnect
```

Run these commands from the repository directory with Python 3.9 or newer; no
additional Python packages are needed for remote access. To use another password
file, add `--password-file PATH` before the subcommand.

For missing piano keys, check `usb.connected` and `usb.rx_active`, then play a
note and watch `usb.rx_bytes` / `bridge.usb_rx_events`. If these increase but
`bridge.ble_tx_not_ready` increases, check `ble.connected` and
`ble.notifications_enabled`: the MIDI client must subscribe to notifications.
`ble.notify_submitted` means the Bluetooth stack accepted a notification, not
confirmation that the receiving application processed it.

### Update firmware over Wi-Fi

Complete the initial serial installation above once. For later updates, keep the
bridge powered and connected to Wi-Fi, and pause MIDI playback while updating.
Flash operations can delay MIDI and the final reboot disconnects BLE.

1. Build the new application with ESP-IDF activated:

   ```bash
   idf.py build
   ```

2. Upload from the repository directory. Replace `DEVICE_IP` with the bridge's
   current address; the tool reads the local `.ota-pass` automatically:

   ```bash
   python tools/midi_remote.py --host DEVICE_IP ota build/usb2ble_midi_bridge.bin
   ```

   To install a prebuilt release, use the path to its downloaded
   `usb2ble_midi_bridge.bin` in the same command. OTA accepts the **application
   binary**; the merged image, bootloader, and partition table require serial
   flashing.

3. Keep power connected and wait for `OTA complete:`. The tool computes the
   upload's SHA-256 digest and checks that the restarted device runs the expected
   firmware with OTA image state `2` (valid). Wi-Fi settings and the management
   password are preserved.
4. Reconnect `USB2BLE MIDI Bridge` in your MIDI app and resume playback. Refresh
   the debug page after a firmware update to load its latest interface.

If the tool reports a timeout after uploading, query `status` and `logs` to check
the running version, OTA slot, and startup result. Use serial flashing if the
device cannot reconnect to Wi-Fi. Partition table changes always require serial
flashing.

### OTA validation and recovery

OTA accepts only an ESP32-S3 application for this project, requires a SHA-256
upload digest, validates the complete image, then switches slots and reboots.
Interrupted or invalid uploads leave the current slot selected. New OTA firmware
is marked valid once core initialization and Wi-Fi connectivity are available
at the startup check, at least 10 seconds after initialization. If that check
fails for 90 seconds, the firmware attempts rollback; crashes before confirmation
also trigger bootloader rollback.
The debug server uses HTTP on the local network; use a trusted LAN or VPN for
remote access.
