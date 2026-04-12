#!/usr/bin/env python3
"""Quick test script to rotate two servos through ESP serial commands."""

import argparse
import time

import serial


def send_cmd(ser: serial.Serial, cmd: str, wait_s: float = 0.6) -> None:
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()
    print(f"TX: {cmd}")
    time.sleep(wait_s)

    # Read back any immediate response lines.
    end_t = time.time() + 0.2
    while time.time() < end_t:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"RX: {line}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Dual-servo serial test")
    parser.add_argument("--port", required=True, help="Serial port, example: /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--cycles", type=int, default=3, help="How many test cycles to run")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    try:
        print(f"Connected: {args.port} @ {args.baud}")
        time.sleep(2.0)  # Let ESP reboot after opening serial

        for i in range(args.cycles):
            print(f"--- Cycle {i + 1}/{args.cycles} ---")

            # Angle servo on pin 25
            send_cmd(ser, "A 0", 1.0)
            send_cmd(ser, "A 180", 1.0)
            send_cmd(ser, "A 90", 0.8)

            # Continuous servo on pin 33
            send_cmd(ser, "P 1600", 1.0)  # forward (typical)
            send_cmd(ser, "P 1500", 0.8)  # stop
            send_cmd(ser, "P 1400", 1.0)  # reverse (typical)
            send_cmd(ser, "P 1500", 0.8)  # stop

        print("Test complete")
    finally:
        ser.close()
        print("Disconnected")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
