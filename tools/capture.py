#!/usr/bin/env python3
"""
Capture pendulum telemetry from the Teensy and save to CSV.

Usage:
    python tools/capture.py                      # default port, save to data.csv
    python tools/capture.py -p /dev/ttyACM1 -o run1.csv
    python tools/capture.py -d 10               # stop after 10 s

Stop early at any time with Ctrl+C.

Requires pyserial (already in gui/.venv):
    source gui/.venv/bin/activate
"""

import argparse
import csv
import struct
import sys
import time
import serial

# ---- Packet format (must match firmware struct Sample) ----
# uint16 header | uint32 t_us | float angle_deg | float cart_mm | float cart_vel | uint8 feedback
PACKET_FMT  = "<H I f f f B"
PACKET_SIZE = struct.calcsize(PACKET_FMT)   # 19 bytes
HEADER      = bytes([0x55, 0xAA])           # 0xAA55 little-endian

CSV_HEADER  = ["t_us", "t_s", "angle_deg", "cart_mm", "cart_vel_mm_s", "feedback_on"]


def read_packet(ser: serial.Serial):
    """Scan the stream for the 2-byte header, then return one decoded packet."""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != 0x55:
            continue
        b2 = ser.read(1)
        if not b2:
            return None
        if b2[0] != 0xAA:
            continue
        # Header found — read the rest of the packet
        rest = ser.read(PACKET_SIZE - 2)
        if len(rest) != PACKET_SIZE - 2:
            return None
        return struct.unpack(PACKET_FMT, HEADER + rest)


def main():
    parser = argparse.ArgumentParser(description="Capture Teensy telemetry to CSV")
    parser.add_argument("-p", "--port",     default="/dev/ttyACM0")
    parser.add_argument("-b", "--baud",     default=115200, type=int)
    parser.add_argument("-o", "--output",   default="data.csv")
    parser.add_argument("-d", "--duration", default=None, type=float,
                        help="Stop after this many seconds (default: until Ctrl+C)")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud …")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.5)   # let the Teensy reset after DTR toggle
    ser.reset_input_buffer()

    rows = []
    t_start_host = time.monotonic()
    t_end = (t_start_host + args.duration) if args.duration else None

    print(f"Recording — press Ctrl+C to stop{f' (max {args.duration} s)' if t_end else ''}.")
    print(f"{'samples':>8}  {'t [s]':>8}  {'angle [deg]':>12}  {'cart [mm]':>10}")

    last_print = time.monotonic()

    try:
        while True:
            if t_end and time.monotonic() >= t_end:
                print("\nDuration reached.")
                break

            pkt = read_packet(ser)
            if pkt is None:
                continue

            _, t_us, angle_deg, cart_mm, cart_vel, feedback = pkt
            rows.append([t_us, t_us / 1e6, angle_deg, cart_mm, cart_vel, int(feedback)])

            # Print a summary line once per second
            now = time.monotonic()
            if now - last_print >= 1.0:
                print(f"{len(rows):>8}  {t_us/1e6:>8.2f}  {angle_deg:>12.3f}  {cart_mm:>10.2f}")
                last_print = now

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()

    if not rows:
        print("No data received — nothing saved.")
        sys.exit(1)

    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)

    print(f"Saved {len(rows)} samples to '{args.output}'.")


if __name__ == "__main__":
    main()
