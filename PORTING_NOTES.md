# Porting Notes

This repository contains a minimal port of the original STM32F756ZG firmware to the ChipWhisperer-Lite platform based on the ATxmega128D4.

## Major changes vs. STM32 implementation

- **HAL Replacement** – all STM32 Cube/HAL calls were removed.  The firmware now uses the ChipWhisperer XMEGA HAL and the SimpleSerial v1 protocol.
- **Clocking** – the STM32 ran from a 216 MHz PLL.  The XMEGA runs from the
  7.3728 MHz crystal on the CW‑Lite board.
- **Trigger** – DMA/timer based triggering from the STM32 version was replaced with simple `trigger_high()` / `trigger_low()` macros driving `PORTA.0`.
- **AES Engine** – the STM32 used a hardware AES engine with DMA.  The XMEGA build links against the software TinyAES implementation and performs blocking encryption.
- **Memory limits** – global buffers are capped at 16‑byte blocks to fit in the ATxmega128D4’s 8 KB RAM.
- **Endianness** – STM32 is little‑endian; XMEGA is also little‑endian so no byte swapping is required.

## Build Notes

Build using the ChipWhisperer make system:

```
make PLATFORM=CWLITEXMEGA SS_VER=SS_VER_1_1 CRYPTO_TARGET=TINYAES128C
```

The resulting `.hex` file can be programmed with the ChipWhisperer API (see `flash_instructions.py`).
