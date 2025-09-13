#!/usr/bin/env python3
"""
ChipWhisperer-Lite/XMEGA diagnostic helper.

Usage:
    python cw_diag.py [SERIAL_PORT]
Example:
    python cw_diag.py /dev/ttyACM0
"""
import sys
import subprocess
import chipwhisperer as cw
import serial
import serial.tools.list_ports

def sh(cmd: str) -> None:
    """Run a shell command and print the output."""
    print(f"\n$ {cmd}")
    subprocess.run(cmd, shell=True, check=False)

def list_serial_ports() -> None:
    """List available serial ports."""
    print("\n# Detected serial ports")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device:12} {port.description}")

def check_scope() -> None:
    """Try to open the CW-Lite scope and verify firmware."""
    print("\n# Connecting to ChipWhisperer-Lite scope")
    scope = cw.scope()
    try:
        cw.check_firmware(scope)
        print("scope OK")
    finally:
        scope.dis()

def echo_test(port: str) -> None:
    """Send 't' to the target and print the echoed line."""
    print(f"\n# Echo test on {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        ser.reset_input_buffer()
        ser.write(b"t\n")
        ser.flush()
        resp = ser.readline().decode(errors="ignore").strip()
        print("echo response:", resp or "<no data>")
    finally:
        ser.close()

def host_sanity(port: str) -> None:
    """Run host_sanity.py against the specified port."""
    sh(f"python host_sanity.py {port}")

def main(port: str) -> None:
    sh("lsusb | grep -i newae || echo 'ChipWhisperer not found'")
    list_serial_ports()
    try:
        check_scope()
    except Exception as e:
        print("scope check failed:", e)
        return
    try:
        echo_test(port)
    except Exception as e:
        print("UART echo failed:", e)
        return
    host_sanity(port)

if __name__ == "__main__":
    serial_port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    main(serial_port)
