#!/usr/bin/env python3
"""
velocity_stream_test.py — Interactive test utility for VELOCITY_STREAM mode.

Sends continuous velocity commands to the AVR controller over USB serial
and prints incoming telemetry. Supports both interactive keyboard control
and scripted command sequences.

Usage:
    python velocity_stream_test.py              # auto-detect COM port
    python velocity_stream_test.py --port COM5  # specify port
    python velocity_stream_test.py --demo       # run a scripted demo sequence

Keyboard controls (interactive mode):
    W / S   — increase / decrease linear velocity
    A / D   — increase / decrease angular velocity
    SPACE   — immediate stop (zero both targets)
    Q       — quit

Requirements:
    pip install pyserial
"""

import argparse
import sys
import time
import threading
from typing import Optional
import serial
import serial.tools.list_ports

# ─── Default parameters ───────────────────────────────────────────────────────
DEFAULT_BAUD = 9600
SEND_RATE_HZ = 50          # command send rate (Hz)
DEBUG_MODE   = 0            # 0 = debug off

LIN_VEL_STEP = 25.0        # mm/s per key press
ANG_VEL_STEP = 5.0         # deg/s per key press
MAX_LIN_VEL  = 500.0       # mm/s  (must match VSTREAM_MAX_LIN_VEL)
MAX_ANG_VEL  = 120.0       # deg/s (must match VSTREAM_MAX_ANG_VEL)


# ─── Helpers ──────────────────────────────────────────────────────────────────
def find_avr_port() -> Optional[str]:
    """Try to auto-detect the AVR Leonardo CDC port."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "arduino" in desc or "leonardo" in desc or "usb serial" in desc:
            return p.device
    # fallback: return first available port
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def build_cmd(lin_vel: float, ang_vel: float, debug: int = DEBUG_MODE) -> str:
    """Build a VELOCITY_STREAM ASCII command string."""
    return f"2,{debug},{lin_vel:.1f},{ang_vel:.1f}\n"


# ─── Telemetry reader thread ─────────────────────────────────────────────────
def telemetry_reader(ser: serial.Serial, stop_event: threading.Event):
    """Background thread that reads and prints telemetry lines."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    print(f"  [TEL] {line}")
        except serial.SerialException:
            break
        except Exception:
            pass
        time.sleep(0.005)


# ─── Interactive mode ────────────────────────────────────────────────────────
def interactive_mode(ser: serial.Serial):
    """WASD keyboard control with continuous streaming."""
    try:
        import msvcrt  # Windows
        def get_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode("ascii", errors="replace").lower()
            return None
    except ImportError:
        # Unix / macOS
        import tty, termios, select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        def get_key():
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
            return None

    lin_vel = 0.0
    ang_vel = 0.0
    period  = 1.0 / SEND_RATE_HZ

    print("\n╔══════════════════════════════════════════════╗")
    print("║   VELOCITY_STREAM Interactive Controller     ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  W/S  — linear velocity  ↑/↓                ║")
    print("║  A/D  — angular velocity ←/→                ║")
    print("║  SPACE — stop (zero both)                   ║")
    print("║  Q     — quit                               ║")
    print("╚══════════════════════════════════════════════╝\n")

    stop_event = threading.Event()
    tel_thread = threading.Thread(target=telemetry_reader,
                                  args=(ser, stop_event), daemon=True)
    tel_thread.start()

    try:
        while True:
            t0 = time.monotonic()

            key = get_key()
            if key == "q":
                # send stop before exiting
                ser.write(build_cmd(0.0, 0.0).encode())
                print("\n[INFO] Sent stop command. Exiting.")
                break
            elif key == "w":
                lin_vel = clamp(lin_vel + LIN_VEL_STEP, -MAX_LIN_VEL, MAX_LIN_VEL)
            elif key == "s":
                lin_vel = clamp(lin_vel - LIN_VEL_STEP, -MAX_LIN_VEL, MAX_LIN_VEL)
            elif key == "a":
                ang_vel = clamp(ang_vel + ANG_VEL_STEP, -MAX_ANG_VEL, MAX_ANG_VEL)
            elif key == "d":
                ang_vel = clamp(ang_vel - ANG_VEL_STEP, -MAX_ANG_VEL, MAX_ANG_VEL)
            elif key == " ":
                lin_vel = 0.0
                ang_vel = 0.0

            cmd = build_cmd(lin_vel, ang_vel)
            ser.write(cmd.encode())

            # status line (overwrite in place)
            sys.stdout.write(
                f"\r  CMD → lin={lin_vel:+7.1f} mm/s  ang={ang_vel:+6.1f} deg/s   "
            )
            sys.stdout.flush()

            # maintain constant send rate
            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        ser.write(build_cmd(0.0, 0.0).encode())
        print("\n[INFO] Interrupted. Sent stop command.")
    finally:
        stop_event.set()
        tel_thread.join(timeout=1.0)
        # restore terminal on Unix
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


# ─── Demo / scripted sequence ────────────────────────────────────────────────
def demo_mode(ser: serial.Serial):
    """
    Run a scripted sequence of velocity commands demonstrating:
      1. Forward ramp-up
      2. Steady cruise
      3. In-place rotation
      4. Combined arc
      5. Smooth stop
      6. Timeout test (stop sending)
    """
    sequence = [
        # (duration_s, lin_vel, ang_vel, description)
        (2.0,  200.0,   0.0,  "Ramp up forward 200 mm/s"),
        (3.0,  200.0,   0.0,  "Cruise at 200 mm/s"),
        (2.0,    0.0,  45.0,  "Stop linear, spin 45 deg/s"),
        (3.0,  150.0,  20.0,  "Arc: 150 mm/s + 20 deg/s"),
        (2.0,    0.0,   0.0,  "Ramp to stop"),
        # timeout test: we stop sending for 1 second
    ]

    stop_event = threading.Event()
    tel_thread = threading.Thread(target=telemetry_reader,
                                  args=(ser, stop_event), daemon=True)
    tel_thread.start()

    period = 1.0 / SEND_RATE_HZ
    print("\n[DEMO] Starting scripted velocity-stream sequence...\n")

    try:
        for duration, lin, ang, desc in sequence:
            print(f"  ▸ {desc}  (lin={lin:+.0f}, ang={ang:+.0f}, {duration:.1f}s)")
            end_time = time.monotonic() + duration
            while time.monotonic() < end_time:
                cmd = build_cmd(lin, ang)
                ser.write(cmd.encode())
                time.sleep(period)

        # Timeout test: stop sending and let watchdog kick in
        print(f"\n  ▸ Timeout test: NOT sending for 1.5 s (watchdog should stop robot)")
        time.sleep(1.5)

        print("\n[DEMO] Sequence complete.")

    except KeyboardInterrupt:
        ser.write(build_cmd(0.0, 0.0).encode())
        print("\n[DEMO] Interrupted. Sent stop command.")
    finally:
        stop_event.set()
        tel_thread.join(timeout=1.0)


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Test utility for AVR VELOCITY_STREAM mode")
    parser.add_argument("--port", type=str, default=None,
                        help="Serial port (e.g. COM5 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--demo", action="store_true",
                        help="Run scripted demo sequence instead of interactive")
    args = parser.parse_args()

    port = args.port or find_avr_port()
    if not port:
        print("[ERROR] No serial port found. Use --port to specify one.")
        sys.exit(1)

    print(f"[INFO] Opening {port} at {args.baud} baud...")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        sys.exit(1)

    time.sleep(2.0)  # wait for AVR Leonardo USB CDC reset
    ser.reset_input_buffer()
    print(f"[INFO] Connected to {port}.\n")

    try:
        if args.demo:
            demo_mode(ser)
        else:
            interactive_mode(ser)
    finally:
        # always send stop on exit
        try:
            ser.write(build_cmd(0.0, 0.0).encode())
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print("[INFO] Serial port closed.")


if __name__ == "__main__":
    main()
