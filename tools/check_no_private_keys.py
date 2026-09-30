#!/usr/bin/env python3
"""Reject private-key files/material in the Git index (also usable before commit).

Scans indexed blob contents, not working-tree contents, so a staged key cannot be
hidden by editing the working file. Prints only paths, never secret contents.
This is a narrow private-key guard, not a general-purpose secrets detector.
"""
from pathlib import PurePosixPath
import re
import subprocess
import sys

KEY_SUFFIXES = {'.key', '.pem', '.p12', '.pfx', '.pk8', '.ppk', '.jks', '.keystore'}
KEY_NAMES = {'id_rsa', 'id_dsa', 'id_ecdsa', 'id_ed25519'}
PEM = re.compile(rb'-----BEGIN (?:[A-Z0-9]+ )*PRIVATE KEY-----')
PUTTY = re.compile(rb'PuTTY-User-Key-File-[0-9]+:')


def main() -> int:
    try:
        entries = subprocess.check_output(['git', 'ls-files', '--stage', '-z']).split(b'\0')
        bad = []
        for entry in entries:
            if not entry:
                continue
            meta, raw_path = entry.split(b'\t', 1)
            mode, sha, stage = meta.split()
            path = raw_path.decode('utf-8', errors='replace')
            if stage != b'0':
                bad.append((path, 'unmerged index entry'))
                continue
            if mode == b'160000':
                continue
            name = PurePosixPath(path).name.lower()
            if PurePosixPath(name).suffix in KEY_SUFFIXES or name in KEY_NAMES:
                bad.append((path, 'private-key/signing-store filename'))
                continue
            data = subprocess.check_output(['git', 'cat-file', 'blob', sha.decode('ascii')])
            if PEM.search(data) or PUTTY.search(data):
                bad.append((path, 'private-key material'))
        for path, reason in bad:
            print(f'BLOCKED: {path}: {reason}', file=sys.stderr)
        if bad:
            return 1
        print('PASS: no private keys found in the Git index.')
        return 0
    except (OSError, subprocess.CalledProcessError, ValueError) as exc:
        print(f'Private-key scan could not complete: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
