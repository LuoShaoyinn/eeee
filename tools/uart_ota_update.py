#!/usr/bin/env python3
"""Install an ESP32 application image through its UART A/B OTA protocol."""

import argparse
import pathlib
import serial
import sys
import time
import zlib


def wait_for(port, marker, timeout):
    deadline = time.monotonic() + timeout
    received = bytearray()
    while time.monotonic() < deadline:
        received.extend(port.read(256))
        if marker in received:
            return bytes(received)
    raise TimeoutError(received.decode(errors="replace"))


def main():
    parser = argparse.ArgumentParser(
        description="Upload an ESP32 application image; robotd must be stopped first.")
    parser.add_argument("image", type=pathlib.Path)
    parser.add_argument("--port", default="/dev/ttyAS2")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    image = args.image.read_bytes()
    crc32 = zlib.crc32(image) & 0xFFFFFFFF
    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        port.reset_input_buffer()
        port.write(f"ota {len(image)} {crc32:08x}\n".encode())
        port.flush()
        wait_for(port, b"@ OTA READY\n", 5)
        for offset in range(0, len(image), 256):
            port.write(image[offset : offset + 256])
            port.flush()
            time.sleep(0.003)
        reply = wait_for(port, b"@ OTA OK REBOOT\n", 20)
    print(reply.decode(errors="replace").strip())


if __name__ == "__main__":
    try:
        main()
    except (OSError, TimeoutError) as error:
        print(f"update failed: {error}", file=sys.stderr)
        raise SystemExit(1)
