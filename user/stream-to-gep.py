#!/usr/bin/env python3

"""
Stream test text to the drone SoftSerial port.

Default target:
- Port: /dev/serial0
- Baud: 9600

This is intended as a simple transmit-only test. It writes one line at a
regular interval so the flight controller/SoftSerial input can be checked.
"""

from __future__ import annotations

import argparse
import itertools
import sys
import time
from pathlib import Path

import serial

PORT = "/dev/serial0"
BAUD = 9600
DEFAULT_MESSAGE = "SoftSerial test"
DEFAULT_INTERVAL = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream text to the drone SoftSerial serial port.",
    )
    parser.add_argument(
        "message",
        nargs="?",
        default=DEFAULT_MESSAGE,
        help=f"text to send repeatedly (default: {DEFAULT_MESSAGE!r})",
    )
    parser.add_argument(
        "--port",
        default=PORT,
        help=f"serial port to open (default: {PORT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=BAUD,
        help=f"serial baud rate (default: {BAUD})",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL,
        help=f"seconds between lines (default: {DEFAULT_INTERVAL})",
    )
    parser.add_argument(
        "--no-counter",
        action="store_true",
        help="send the same message each time instead of adding a counter",
    )
    parser.add_argument(
        "--newline",
        choices=("lf", "crlf", "none"),
        default="lf",
        help="line ending to append (default: lf)",
    )
    return parser.parse_args()


def line_ending(name: str) -> str:
    if name == "lf":
        return "\n"
    if name == "crlf":
        return "\r\n"
    return ""


def build_line(message: str, count: int, include_counter: bool) -> str:
    if include_counter:
        return f"{message} {count}"
    return message


def main() -> int:
    args = parse_args()

    if args.interval <= 0:
        print("ERROR: --interval must be greater than 0.")
        return 1

    if not Path(args.port).exists():
        print(f"ERROR: {args.port} does not exist.")
        return 2

    suffix = line_ending(args.newline)

    print("SoftSerial text streamer")
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Interval: {args.interval}s")
    print("Press Ctrl-C to stop.\n")

    try:
        with serial.Serial(
            args.port,
            baudrate=args.baud,
            timeout=0.5,
            write_timeout=2,
        ) as stream:
            stream.reset_input_buffer()
            stream.reset_output_buffer()

            for count in itertools.count(1):
                line = build_line(
                    args.message,
                    count,
                    include_counter=not args.no_counter,
                )
                payload = f"{line}{suffix}".encode("ascii", errors="replace")

                stream.write(payload)
                stream.flush()
                print(f"Sent: {line}")

                time.sleep(args.interval)

    except serial.SerialTimeoutException:
        print("\nERROR: timed out writing to the serial port.")
        return 3
    except serial.SerialException as exc:
        print(f"\nSerial error: {exc}")
        return 4
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
