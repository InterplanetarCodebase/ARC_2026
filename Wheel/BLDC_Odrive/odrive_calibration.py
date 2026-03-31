"""
ODrive 3.6 56V Configuration Script
Supports --axis (0, 1, or both) and --device (odrv0 or dev0)
"""

import odrive
from odrive.enums import *
import time
import argparse
import sys


# ─────────────────────────────────────────────
#  CLI Arguments
# ─────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="ODrive 3.6 56V Configuration Script",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--axis",
        nargs="+",
        choices=["0", "1"],
        default=["0"],
        metavar="{0,1}",
        help="Axis/axes to configure: 0, 1, or both (e.g. --axis 0 1)"
    )
    parser.add_argument(
        "--device",
        choices=["odrv0", "dev0"],
        default="odrv0",
        help="Device name to use: odrv0 (default) or dev0"
    )
    args = parser.parse_args()
    args.axis = sorted(set(int(a) for a in args.axis))  # deduplicate and sort
    return args


# ─────────────────────────────────────────────
#  Connection
# ─────────────────────────────────────────────

def connect(device_name):
    print(f"Connecting to ODrive ({device_name})...")
    try:
        dev = odrive.find_any(timeout=10)
        print(f"  Connected: serial={dev.serial_number}")
        return dev
    except Exception as e:
        print(f"  ERROR: Could not connect to ODrive — {e}")
        sys.exit(1)


def safe_reboot(dev, device_name):
    try:
        dev.reboot()
    except Exception:
        pass  # Expected disconnect on reboot
    print("  Waiting for ODrive to reboot...")
    time.sleep(6)
    return connect(device_name)


# ─────────────────────────────────────────────
#  Axis Helpers
# ─────────────────────────────────────────────

def get_axis(dev, axis_num):
    return dev.axis0 if axis_num == 0 else dev.axis1


def decode_axis_error(err):
    known_errors = {
        0x00000001: "INVALID_STATE",
        0x00000002: "DC_BUS_UNDER_VOLTAGE",
        0x00000004: "DC_BUS_OVER_VOLTAGE",
        0x00000008: "CURRENT_MEASUREMENT_TIMEOUT",
        0x00000010: "BRAKE_RESISTOR_DISARMED",
        0x00000020: "MOTOR_DISARMED",
        0x00000040: "MOTOR_FAILED",
        0x00000080: "SENSORLESS_ESTIMATOR_FAILED",
        0x00000100: "ENCODER_FAILED",
        0x00000200: "CONTROLLER_FAILED",
        0x00000400: "POS_CTRL_DURING_SENSORLESS",
        0x00000800: "WATCHDOG_TIMER_EXPIRED",
        0x00001000: "MIN_ENDSTOP_PRESSED",
        0x00002000: "MAX_ENDSTOP_PRESSED",
        0x00004000: "ESTOP_REQUESTED",
        0x00020000: "HOMING_WITHOUT_ENDSTOP",
        0x00040000: "OVER_TEMP",
    }
    active = [msg for flag, msg in known_errors.items() if err & flag]
    return active if active else ["UNKNOWN_ERROR"]


def check_axis_error(dev, axis_num, stage):
    axis = get_axis(dev, axis_num)
    err = axis.error
    if err == 0:
        return True

    print(f"  [axis{axis_num}] ERROR at '{stage}': {hex(err)}")
    for msg in decode_axis_error(err):
        print(f"    → {msg}")

    if err & 0x00000100:
        print(f"    HINT: Encoder/Hall sensor issue — check wiring on axis{axis_num}.")
    if err & 0x00000040 or err & 0x00000020:
        print(f"    HINT: Motor not connected or phase wiring issue on axis{axis_num}.")
    if err & 0x00000002:
        print(f"    HINT: DC bus under-voltage — check power supply.")
    if err & 0x00000004:
        print(f"    HINT: DC bus over-voltage — check brake resistor or supply voltage.")

    return False


def is_axis_healthy(dev, axis_num):
    axis = get_axis(dev, axis_num)
    err = axis.error
    if err != 0:
        print(f"  [axis{axis_num}] Pre-existing error {hex(err)} detected — skipping axis.")
        for msg in decode_axis_error(err):
            print(f"    → {msg}")
        return False
    return True


# ─────────────────────────────────────────────
#  Per-axis Configuration
# ─────────────────────────────────────────────

def apply_axis_config(dev, axis_num):
    print(f"  Configuring axis{axis_num}...")
    axis = get_axis(dev, axis_num)

    # Motor
    axis.motor.config.pole_pairs = 15
    axis.motor.config.resistance_calib_max_voltage = 4
    axis.motor.config.requested_current_range = 25
    axis.motor.config.current_control_bandwidth = 100
    axis.motor.config.torque_constant = 0.5168750286102295

    # Encoder (Hall-effect)
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 90
    axis.encoder.config.calib_scan_distance = 150
    axis.encoder.config.bandwidth = 100

    # Controller
    tc  = axis.motor.config.torque_constant
    cpr = axis.encoder.config.cpr
    axis.controller.config.pos_gain = 1
    axis.controller.config.vel_gain = 0.02 * tc * cpr
    axis.controller.config.vel_integrator_gain = 0.1 * tc * cpr
    axis.controller.config.vel_limit = 10
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    print(f"  axis{axis_num} settings applied.")


def calibrate_axis(dev, axis_num):
    axis = get_axis(dev, axis_num)
    print(f"\n  [axis{axis_num}] Starting full calibration (~40s)...")
    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    for i in range(40, 0, -5):
        print(f"    axis{axis_num}: {i}s remaining...")
        time.sleep(5)

    # Poll until idle
    timeout = 30
    while axis.current_state != AXIS_STATE_IDLE and timeout > 0:
        time.sleep(1)
        timeout -= 1

    if timeout == 0:
        print(f"  [axis{axis_num}] WARNING: Timed out waiting for IDLE state.")

    if not check_axis_error(dev, axis_num, "calibration"):
        return False

    axis.motor.config.pre_calibrated = True
    axis.encoder.config.pre_calibrated = True
    offset = axis.encoder.config.offset_float
    print(f"  [axis{axis_num}] Calibration OK — encoder offset_float: {offset:.4f}")
    return True


def start_closed_loop(dev, axis_num):
    axis = get_axis(dev, axis_num)
    print(f"  [axis{axis_num}] Entering closed-loop control...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)

    if not check_axis_error(dev, axis_num, "closed-loop"):
        print(f"  [axis{axis_num}] Could not enter closed-loop. Check motor/encoder connections.")
        return False

    print(f"  [axis{axis_num}] Running in CLOSED_LOOP velocity control.")
    return True


# ─────────────────────────────────────────────
#  Main Flow
# ─────────────────────────────────────────────

def main():
    args = parse_args()
    device_name = args.device
    axes = args.axis

    print("=" * 48)
    print(f"  ODrive 3.6 56V — Configuration Script")
    print(f"  Device : {device_name}   |   Axes: {axes}")
    print("=" * 48)

    dev = connect(device_name)

    # ── Step 1: Erase ──
    print("\n" + "!" * 48)
    print("  !!         W A R N I N G          !!")
    print("!" * 48)
    print("  All existing ODrive configuration will be")
    print("  permanently erased and cannot be recovered.")
    print("!" * 48)
    try:
        input("\n  Press ENTER to continue, or Ctrl+C to abort...\n")
    except KeyboardInterrupt:
        print("\n\n  Aborted. No changes were made.")
        sys.exit(0)

    print("[Step 1/4] Erasing configuration...")
    try:
        dev.erase_configuration()
    except Exception:
        pass  # Immediate reboot/disconnect on erase is expected
    print("  Waiting for reboot after erase...")
    time.sleep(6)
    dev = connect(device_name)

    # ── Step 2: Apply settings ──
    print("\n[Step 2/4] Applying axis settings...")
    valid_axes = []
    for ax in axes:
        if not is_axis_healthy(dev, ax):
            print(f"  Skipping axis{ax}.")
            continue
        apply_axis_config(dev, ax)
        valid_axes.append(ax)

    if not valid_axes:
        print("\nERROR: No valid axes to configure. Check all motor and encoder connections.")
        sys.exit(1)

    dev.save_configuration()
    print("\n  Settings saved. Rebooting...")
    dev = safe_reboot(dev, device_name)

    # ── Step 3: Calibrate ──
    print("\n[Step 3/4] Calibrating axes...")
    calibrated_axes = []
    for ax in valid_axes:
        ok = calibrate_axis(dev, ax)
        if ok:
            calibrated_axes.append(ax)
        else:
            print(f"  axis{ax} calibration failed — skipping.")

    if not calibrated_axes:
        print("\nERROR: All axes failed calibration. Check motor and Hall sensor wiring.")
        sys.exit(1)

    dev.save_configuration()
    print("\n  Pre-calibrated config saved. Rebooting...")
    dev = safe_reboot(dev, device_name)

    # ── Step 4: Closed-loop ──
    print("\n[Step 4/4] Starting closed-loop control...")
    success_axes = []
    for ax in calibrated_axes:
        ok = start_closed_loop(dev, ax)
        if ok:
            success_axes.append(ax)

    # ── Summary ──
    skipped = [ax for ax in axes if ax not in success_axes]
    print("\n" + "=" * 48)
    print("  SETUP COMPLETE")
    if success_axes:
        print(f"  ✓ Active  : axis {success_axes}")
        for ax in success_axes:
            print(f"    {device_name}.axis{ax}.controller.input_vel = <turns/s>")
    if skipped:
        print(f"  ✗ Skipped : axis {skipped} (not connected or error)")
    print("=" * 48)


if __name__ == "__main__":
    main()