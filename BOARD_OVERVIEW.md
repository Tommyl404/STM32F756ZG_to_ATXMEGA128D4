# Board Overview

This firmware targets the **STM32F756ZG** microcontroller. The board acts as the embedded side of an IKNP-style oblivious transfer extension. Its responsibilities are:

1. **Peripheral setup** – On boot, `main.c` initializes system clocks, enables cache, and configures UART2, RNG, HASH, and the CRYP (hardware AES) module. These peripherals are used throughout the protocol.
2. **Serial communication** – The board communicates with a host computer over USART2. It receives 128-bit blocks, drives a multi-stage state machine, and returns status or computed blocks back to the host.
3. **Pseudorandom generation** – AES is used as a pseudorandom generator. Arrays such as `arrayOfAesKeys` and `arrayOfBlocks` are expanded via AES rounds to produce large batches of random blocks needed by the OT protocol.
4. **Bit processing pipeline** – The code tracks progress through several `stages`:
   - stage 0/1: receive seeds and expand them with AES into block arrays.
   - stage 2: convert received bytes into boolean arrays for further processing.
   - stage 3: combine blocks and booleans, hash results (`mitccrh_hash`), and prepare final transfer values.
5. **Completion signal** – When all blocks are processed, the board transmits the string `"DONE"` over UART to notify the host that the computation finished.

Overall, the board acts as a cryptographic coprocessor: it performs high‑throughput AES operations and simple logic in response to data streamed from a host PC running `host_sanity.py` or a similar program.
