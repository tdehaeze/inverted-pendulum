#!/usr/bin/env python3
"""
Record pendulum telemetry from the Teensy, save to CSV, and plot.

Usage:
    python tools/capture.py                        # 10 s → data/data.csv, figs/data.png
    python tools/capture.py -d 5 -o data/run1.csv  # → data/run1.csv, figs/run1.png
    python tools/capture.py -d 30 -o data/long.csv -p /dev/ttyACM1

Stop early at any time with Ctrl+C.
Requires: pyserial, matplotlib  (source gui/.venv/bin/activate)
"""

import argparse
import csv
import os
import struct
import sys
import time

import matplotlib
if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    matplotlib.use("Agg")   # no display available — save PNG only
import matplotlib.pyplot as plt
import serial

# ---- Packet format (must match firmware struct Sample) ----
# uint16 header | uint32 t_us | float angle_deg | float cart_mm | float cart_vel | uint8 feedback
PACKET_FMT  = "<H I f f f B"
PACKET_SIZE = struct.calcsize(PACKET_FMT)   # 19 bytes
HEADER      = bytes([0x55, 0xAA])           # 0xAA55 little-endian

CSV_COLUMNS = ["t_us", "t_s", "angle_deg", "cart_mm", "cart_vel_mm_s", "feedback_on"]


def read_packet(ser: serial.Serial):
    """Scan byte-by-byte for the header, then decode one packet."""
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
        rest = ser.read(PACKET_SIZE - 2)
        if len(rest) != PACKET_SIZE - 2:
            return None
        return struct.unpack(PACKET_FMT, HEADER + rest)


def capture(ser: serial.Serial, duration: float) -> list:
    rows = []
    t_end      = time.monotonic() + duration
    last_print = time.monotonic()

    print(f"  {'samples':>8}  {'t [s]':>8}  {'angle [deg]':>12}")

    try:
        while time.monotonic() < t_end:
            pkt = read_packet(ser)
            if pkt is None:
                continue
            _, t_us, angle_deg, cart_mm, cart_vel, feedback = pkt
            rows.append([t_us, t_us / 1e6, angle_deg, cart_mm, cart_vel, int(feedback)])

            now = time.monotonic()
            if now - last_print >= 1.0:
                print(f"  {len(rows):>8}  {t_us/1e6:>8.2f}  {angle_deg:>12.3f}")
                last_print = now

        print(f"\nDone — {len(rows)} samples.")
    except KeyboardInterrupt:
        print(f"\nStopped early — {len(rows)} samples.")

    return rows


def save_csv(rows: list, path: str):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        writer.writerows(rows)
    print(f"CSV  saved → {path}")


def plot_and_save(rows: list, csv_path: str):
    t0    = rows[0][1]
    t     = [r[1] - t0 for r in rows]
    angle = [r[2]       for r in rows]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, angle, lw=0.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.grid(True)
    fig.tight_layout()

    basename = os.path.splitext(os.path.basename(csv_path))[0]
    png_path = os.path.join("figs", basename + ".png")
    os.makedirs("figs", exist_ok=True)
    fig.savefig(png_path, dpi=150)
    print(f"Plot saved → {png_path}")
    try:
        plt.show()
    except Exception:
        pass  # no display available — PNG already saved


def main():
    parser = argparse.ArgumentParser(description="Record Teensy telemetry to CSV and plot")
    parser.add_argument("-p", "--port",     default="/dev/ttyACM0",  help="Serial port")
    parser.add_argument("-b", "--baud",     default=115200, type=int, help="Baud rate")
    parser.add_argument("-d", "--duration", default=10.0,   type=float, metavar="SECONDS")
    parser.add_argument("-o", "--output",   default="data/data.csv",   metavar="FILE")
    args = parser.parse_args()

    print(f"Connecting to {args.port} …")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"Recording for up to {args.duration} s (Ctrl+C to stop early):")
    rows = capture(ser, args.duration)
    ser.close()

    if not rows:
        print("No data received.")
        sys.exit(1)

    save_csv(rows, args.output)
    plot_and_save(rows, args.output)


if __name__ == "__main__":
    main()
