"""Simple host script for the CW-Lite XMEGA AES demo.

With a connected board this script writes a key ('k') and plaintext ('p')
commands using the SimpleSerial v1 protocol and validates the returned
ciphertext against Python's AES implementation. The script ignores
'z' acknowledgements and handles fragmented 'r' responses.

For environments without hardware, run with ``--self-test`` to validate
that the AES routine and parsing logic work correctly without hardware.
"""

import argparse
import binascii
import sys
import time

try:
    from Cryptodome.Cipher import AES  # type: ignore
except Exception as exc:  # pragma: no cover
    AES = None
    AES_IMPORT_ERROR = exc

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover
    serial = None
    SERIAL_IMPORT_ERROR = exc


def send_cmd(ser, cmd: str, data: bytes):
    """Send a command and payload, raising ``TimeoutError`` on write issues."""
    ser.reset_input_buffer()
    payload = cmd.encode() + binascii.hexlify(data) + b"\n"
    try:
        ser.write(payload)
    except serial.SerialTimeoutException as exc:
        raise TimeoutError(f"Timed out writing '{cmd}' command to device") from exc


def read_resp(ser, timeout: float = 5.0) -> bytes:
    """Return 16-byte ciphertext from successive ``r<hex>`` lines.

    Raises ``TimeoutError`` if no complete response is received within
    ``timeout`` seconds.
    """
    buf = ""
    # Use a monotonic clock so adjustments to the system time don't
    # cause the timeout loop to run indefinitely.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("z"):
            continue
        if line.startswith("r"):
            buf += line[1:]
            if len(buf) >= 32:
                return binascii.unhexlify(buf[:32])
    raise TimeoutError("Timed out waiting for 'r' response from device")


def check_echo(ser, timeout: float = 2.0) -> None:
    """Verify UART link by sending a test byte and expecting it back."""
    test = b"\xAA"
    send_cmd(ser, "t", test)
    # ``time.monotonic`` avoids issues if the system clock changes while
    # we're waiting for the echo response.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("t") and line[1:] == test.hex():
            return
    raise TimeoutError("No echo response from device")


def run_self_test():
    if AES is None:
        raise SystemExit(f"pycryptodomex required: {AES_IMPORT_ERROR}")
    key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    ref = AES.new(key, AES.MODE_ECB).encrypt(pt)
    print("AES test vector:", ref.hex())


def run_device(port: str):
    if AES is None:
        raise SystemExit(f"pycryptodomex required: {AES_IMPORT_ERROR}")
    if serial is None:
        raise SystemExit(f"pyserial required: {SERIAL_IMPORT_ERROR}")
    try:
        print(f"Connecting to {port}...", flush=True)
        ser = serial.Serial(port, 115200, timeout=0.5, write_timeout=0.5)
    except Exception as exc:
        raise SystemExit(f"Failed to open {port}: {exc}")
    key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    try:
        check_echo(ser)
        send_cmd(ser, "k", key)
        time.sleep(0.1)
        send_cmd(ser, "p", pt)
        ct = read_resp(ser)
    except TimeoutError as exc:
        ser.close()
        raise SystemExit(str(exc))
    ref = AES.new(key, AES.MODE_ECB).encrypt(pt)
    if ct != ref:
        ser.close()
        raise SystemExit(f"Mismatch: device {ct.hex()} != host {ref.hex()}")
    print("Device AES OK:", ct.hex())
    ser.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", nargs="?", help="serial port e.g. /dev/ttyACM0")
    parser.add_argument("--self-test", action="store_true", dest="selftest")
    args = parser.parse_args()
    if args.selftest:
        run_self_test()
    elif args.port:
        run_device(args.port)
    else:
        parser.print_help()
