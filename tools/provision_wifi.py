#!/usr/bin/env python3
"""Build a private NVS image; never compile credentials into release firmware."""
import argparse
import csv
import os
from pathlib import Path
import secrets
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def read_wifi(path):
    values = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        key, separator, value = line.partition('=')
        if not separator:
            raise ValueError('Use ssid=... and password=... in the Wi-Fi file')
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        values[key.strip().lower()] = value
    ssid = values.get('ssid', '')
    password = values.get('password', '')
    if not 1 <= len(ssid.encode()) <= 32:
        raise ValueError('SSID must be 1–32 UTF-8 bytes')
    if not (8 <= len(password.encode()) <= 63 or
            len(password) == 64 and all(c in '0123456789abcdefABCDEF' for c in password)):
        raise ValueError('Password must be 8–63 UTF-8 bytes, or a 64-digit hex PSK')
    return ssid, password

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wifi-file', type=Path, default=ROOT / '.wifi-pass')
    parser.add_argument('--output', type=Path, default=ROOT / 'build' / 'netcfg.bin')
    args = parser.parse_args()
    ssid, password = read_wifi(args.wifi_file)
    token_file = ROOT / '.ota-pass'
    if not token_file.exists():
        with os.fdopen(os.open(token_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as out:
            out.write('password=' + secrets.token_hex(32) + '\n')
    token = token_file.read_text().strip().removeprefix('password=')
    if not 1 <= len(token) <= 128 or any(ord(c) < 33 or ord(c) > 126 for c in token):
        raise ValueError('OTA password must be 1–128 printable ASCII characters without spaces')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.output.with_suffix('.private.csv')
    try:
        with os.fdopen(os.open(csv_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), 'w') as out:
            writer = csv.writer(out)
            writer.writerow(['key', 'type', 'encoding', 'value'])
            writer.writerow(['bridge_net', 'namespace', '', ''])
            for key, value in [('ssid', ssid), ('password', password), ('token', token)]:
                writer.writerow([key, 'data', 'string', value])
        idf = Path(os.environ.get('IDF_PATH', ROOT / '.esp-idf'))
        generator = idf / 'components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py'
        result = subprocess.run([sys.executable, str(generator), 'generate', str(csv_path),
                                 str(args.output), '0x6000'], capture_output=True)
        if result.returncode:
            raise RuntimeError('NVS generation failed; use the ESP-IDF Python environment')
        os.chmod(args.output, 0o600)
    finally:
        csv_path.unlink(missing_ok=True)
    print(f'Private network settings written to {args.output}. Flash at 0x612000.')
    print('Management password saved in .ota-pass; credentials were not printed.')

if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Provisioning failed: {error}', file=sys.stderr)
        sys.exit(1)
