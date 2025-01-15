# USB2BLE MIDI Bridge

This project converts USB MIDI signals from Yamaha Digital Piano to Bluetooth MIDI (BLE MIDI), enabling wireless MIDI connectivity.

Tested on Yamaha YDP-144.

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
5. Start using - the piano's MIDI signals will now be transmitted via Bluetooth to the connected device

## Build and Flash

```bash
idf.py -p /dev/your_usb_to_serial_port flash
```
