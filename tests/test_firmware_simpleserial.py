from pathlib import Path


def test_firmware_uses_simpleserial_only():
    """Firmware should rely on SimpleSerial handlers rather than raw UART routines."""
    src = Path("xmega/simpleserial-aes/simpleserial-aes.c").read_text()

    # Ensure SimpleSerial API is present
    for token in ["simpleserial_init", "simpleserial_addcmd", "simpleserial_get", "simpleserial_put"]:
        assert token in src

    # Disallow direct UART send/receive helpers
    disallowed = ["uart_get", "uart_put", "uart_read", "uart_write", "USART"]
    for token in disallowed:
        assert token not in src, f"unexpected UART usage: {token}"
