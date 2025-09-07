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
    ser.write(cmd.encode())
    ser.write(binascii.hexlify(data))
    ser.write(b"\n")


def read_resp(ser) -> bytes:
    buf = ""
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        if line.startswith("z"):
            continue
        if line.startswith("r"):
            buf += line[1:]
            if len(buf) >= 32:
                return binascii.unhexlify(buf[:32])


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
    ser = serial.Serial(port, 115200, timeout=0.5)
    key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    send_cmd(ser, "k", key)
    time.sleep(0.1)
    send_cmd(ser, "p", pt)
    ct = read_resp(ser)
    ref = AES.new(key, AES.MODE_ECB).encrypt(pt)
    if ct != ref:
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

