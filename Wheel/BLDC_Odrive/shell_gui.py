#!/usr/bin/env python3

import odrive
import time
import sys
import tty
import termios

from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

# ─────────────────────────────────────────────
# CONFIG — INSERT YOUR SERIALS HERE
# ─────────────────────────────────────────────
ODRV0_SERIAL = "336D33573235"
ODRV1_SERIAL = "335A33633235"

VEL = 1.0
VEL_MAX = 10.0


# ─────────────────────────────────────────────
# Keyboard input (single char, no enter)
# ─────────────────────────────────────────────
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ─────────────────────────────────────────────
# Setup axis for velocity control
# ─────────────────────────────────────────────
def setup_axis(ax, name=""):
    print(f"[{name}] → IDLE")
    ax.requested_state = AXIS_STATE_IDLE
    time.sleep(0.3)

    # Clear errors (important)
    try:
        ax.error = 0
        ax.motor.error = 0
        ax.encoder.error = 0
        ax.controller.error = 0
    except:
        pass

    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    ax.controller.config.vel_ramp_rate = 3.0

    ax.controller.input_vel = 0.0

    print(f"[{name}] → CLOSED LOOP")
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.6)

    print(f"[{name}] state={ax.current_state} error={ax.error}")


# ─────────────────────────────────────────────
# Connect BOTH ODrives (deterministic)
# ─────────────────────────────────────────────
def connect_odrives():
    print("Connecting odrv0...")
    odrv0 = odrive.find_any(serial_number=ODRV0_SERIAL)

    time.sleep(1)  # IMPORTANT: prevent fibre race

    print("Connecting odrv1...")
    odrv1 = odrive.find_any(serial_number=ODRV1_SERIAL)

    print("\nConnected:")
    print("  odrv0:", odrv0.serial_number)
    print("  odrv1:", odrv1.serial_number)

    # Setup all axes
    setup_axis(odrv0.axis0, "odrv0.axis0 (FL)")
    setup_axis(odrv0.axis1, "odrv0.axis1 (RL)")
    setup_axis(odrv1.axis0, "odrv1.axis0 (FR)")
    setup_axis(odrv1.axis1, "odrv1.axis1 (RR)")

    return odrv0, odrv1


# ─────────────────────────────────────────────
# Motor control helpers
# ─────────────────────────────────────────────
def set_all(odrv0, odrv1, fl, rl, fr, rr):
    odrv0.axis0.controller.input_vel = fl
    odrv0.axis1.controller.input_vel = rl
    odrv1.axis0.controller.input_vel = fr
    odrv1.axis1.controller.input_vel = rr


def stop_all(odrv0, odrv1):
    set_all(odrv0, odrv1, 0, 0, 0, 0)


def idle_all(odrv0, odrv1):
    for odrv in [odrv0, odrv1]:
        try:
            odrv.axis0.requested_state = AXIS_STATE_IDLE
            odrv.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass


# ─────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────
def main():
    global VEL

    odrv0, odrv1 = connect_odrives()

    print("\nControls:")
    print("  w → forward")
    print("  s → reverse")
    print("  a → turn left")
    print("  d → turn right")
    print("  space → stop")
    print("  + / - → adjust speed")
    print("  q → quit\n")

    try:
        while True:
            key = getch()

            if key == 'w':
                set_all(odrv0, odrv1, +VEL, +VEL, -VEL, -VEL)

            elif key == 's':
                set_all(odrv0, odrv1, -VEL, -VEL, +VEL, +VEL)

            elif key == 'a':  # left turn
                set_all(odrv0, odrv1, +VEL, +VEL, +VEL, +VEL)

            elif key == 'd':  # right turn
                set_all(odrv0, odrv1, -VEL, -VEL, -VEL, -VEL)

            elif key == ' ':
                stop_all(odrv0, odrv1)

            elif key == '+':
                VEL = min(VEL + 0.2, VEL_MAX)
                print(f"VEL = {VEL:.2f}")

            elif key == '-':
                VEL = max(VEL - 0.2, 0.1)
                print(f"VEL = {VEL:.2f}")

            elif key == 'q':
                break

    finally:
        print("\nStopping motors...")
        stop_all(odrv0, odrv1)
        time.sleep(0.2)

        print("Idling motors...")
        idle_all(odrv0, odrv1)


if __name__ == "__main__":
    main()