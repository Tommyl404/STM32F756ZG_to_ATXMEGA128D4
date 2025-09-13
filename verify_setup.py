"""Verify environment for host_sanity serial tests."""

import os
import shutil
import subprocess
import sys
from typing import List, Optional


def check_modules() -> List[str]:
    """Return a list of missing-module errors."""
    errors: List[str] = []
    try:
        import serial  # noqa: F401
    except Exception as exc:  # pragma: no cover - import failure path
        errors.append(f"pyserial missing: {exc}")
    try:
        from Cryptodome.Cipher import AES  # noqa: F401
    except Exception as exc:  # pragma: no cover - import failure path
        errors.append(f"pycryptodomex missing: {exc}")
    return errors


def check_port_access(port: str) -> Optional[str]:
    """Return an error string if the serial port is inaccessible."""
    if not os.path.exists(port):
        return f"{port} not found"
    if not os.access(port, os.R_OK | os.W_OK):
        return f"no read/write access to {port}"
    return None


def check_port_in_use(port: str) -> Optional[str]:
    """Return lsof output if the port is busy, else ``None``."""
    if not shutil.which("lsof"):
        return None
    result = subprocess.run(["lsof", port], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    return result.stdout.strip() or None


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    errors = check_modules()
    err = check_port_access(port)
    if err:
        errors.append(err)
    busy = check_port_in_use(port)
    if busy:
        errors.append(f"{port} in use:\n{busy}")
    if errors:
        for e in errors:
            print("ERROR:", e)
        return 1
    print("Environment looks OK. Run host_sanity.py to test the device.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
