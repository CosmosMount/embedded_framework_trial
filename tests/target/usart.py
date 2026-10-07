"""Compare raw USART DMA echoes; target uses the production BSP."""
import argparse
import json
import time

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--phase", type=int, required=True)
    args = parser.parse_args()
    lengths = [1, 7, 31, 32, 33, 63, 64, 65, 255, 256, 257, 1024, 4096]
    if args.phase == 0:
        lengths += [4096] * 64
    total = 0
    with serial.Serial(args.port, 921600, timeout=0.1, write_timeout=3) as uart:
        for case, length in enumerate(lengths):
            expected = bytes((i * 73 + (i >> 8) * 19 + case * 31 + args.phase * 17) & 255
                             for i in range(length))
            if uart.write(expected) != length:
                raise RuntimeError("Incomplete host transmission")
            uart.flush()
            received = bytearray()
            deadline = time.monotonic() + 5
            while len(received) < length and time.monotonic() < deadline:
                received += uart.read(length - len(received))
            if received != expected:
                mismatch = next((i for i, (a, b) in enumerate(zip(received, expected)) if a != b),
                                min(len(received), length))
                raise RuntimeError(f"phase={args.phase} case={case} length={length} "
                                   f"received={len(received)} first_mismatch={mismatch}")
            total += length
        extra = uart.read(1)
        if extra:
            raise RuntimeError(f"Unexpected extra echo: {extra.hex()}")
    print(json.dumps({"phase": args.phase, "cases": len(lengths), "bytes": total}))


if __name__ == "__main__":
    main()
