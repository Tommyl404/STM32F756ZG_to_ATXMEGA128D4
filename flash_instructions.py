"""Program the ChipWhisperer-Lite XMEGA target.

Usage:
    python flash_instructions.py <firmware.hex>
"""

import sys
import chipwhisperer as cw

def main(hexfile:str):
    scope = cw.scope()
    target = cw.target(scope)
    prog = cw.programmers.XMEGAProgrammer
    cw.program_target(scope, prog, hexfile)
    target.close()
    scope.dis()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python flash_instructions.py <firmware.hex>")
        sys.exit(1)
    main(sys.argv[1])