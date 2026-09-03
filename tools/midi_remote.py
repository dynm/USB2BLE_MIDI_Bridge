#!/usr/bin/env python3
"""Read MIDI bridge status/logs, enable tracing, or upload application firmware."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='Device IP address or hostname')
    parser.add_argument('--password-file', type=Path, default=ROOT / '.ota-pass')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status')
    logs = commands.add_parser('logs')
    logs.add_argument('--follow', '-f', action='store_true')
    trace = commands.add_parser('trace')
    trace.add_argument('enabled', choices=['on', 'off'])
    commands.add_parser('usb-reconnect')
    ota = commands.add_parser('ota')
    ota.add_argument('firmware', type=Path)
    args = parser.parse_args()
    password = args.password_file.read_text().strip().removeprefix('password=')
    host = args.host.removeprefix('http://').rstrip('/')
    if any(c in host for c in '/?#@'):
        parser.error('--host must be an IP/hostname with optional port')
    # Avoid sending this device-local request through ambient HTTP proxy settings.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def request(path, data=None, headers=None, timeout=10):
        req = urllib.request.Request('http://' + host + path, data=data,
                                     headers={'Authorization': 'Bearer ' + password, **(headers or {})})
        with opener.open(req, timeout=timeout) as response:
            return response.read()
    if args.command == 'status':
        print(json.dumps(json.loads(request('/api/status')), indent=2))
    elif args.command == 'logs':
        cursor = 0
        while True:
            result = json.loads(request('/api/logs?after=' + str(cursor)))
            if result['dropped']: print('[older log entries were overwritten]')
            if result['next'] < cursor: print('[device restarted]')
            for entry in result['entries']:
                print(re.sub(r'\x1b\[[0-9;]*m', '', entry['text']), end='', flush=True)
            cursor = result['next']
            if not args.follow: break
            time.sleep(1)
    elif args.command == 'trace':
        print(request('/api/trace', b'1' if args.enabled == 'on' else b'0').decode())
    elif args.command == 'usb-reconnect':
        print(request('/api/usb/reconnect', b'').decode())
    elif args.command == 'ota':
        image = args.firmware.read_bytes()
        if len(image) < 288 or image[0] != 0xE9 or image[32:36] != bytes.fromhex('3254cdab'):
            parser.error('Use the application .bin, not the merged image or bootloader')
        # esp_app_desc_t.app_elf_sha256 sits at byte 144 of the application descriptor.
        expected_elf = image[32 + 144:32 + 176].hex()
        print(f'Uploading {len(image)} bytes to {host}...')
        result = request('/api/ota', image, {'Content-Type': 'application/octet-stream',
                         'X-Firmware-SHA256': hashlib.sha256(image).hexdigest()}, timeout=180)
        if not json.loads(result).get('ok'): raise RuntimeError('OTA was not accepted')
        print('Upload verified. Waiting for reboot and startup health check...')
        time.sleep(3)
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            try:
                status = json.loads(request('/api/status', timeout=3))
                if status['elf_sha256'] == expected_elf and status['uptime_ms'] < 90000:
                    if status['ota'].get('image_state') == 2:  # ESP_OTA_IMG_VALID
                        print('OTA complete:', status['version'], status['ota']['running_partition'])
                        return
            except (OSError, ValueError):
                pass
            time.sleep(2)
        raise RuntimeError('Upload succeeded, but the new firmware did not pass the network health check; inspect device/rollback status')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except urllib.error.HTTPError as error:
        print(f'HTTP {error.code}: {error.read().decode(errors="replace")}', file=sys.stderr)
        sys.exit(1)
    except (OSError, ValueError, RuntimeError) as error:
        print(f'Remote request failed: {error}', file=sys.stderr)
        sys.exit(1)
