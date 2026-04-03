"""
ODrive 3.6 56V Configuration Script

Defaults  : both devices, both axes
Override  : --serials <S1> [S2]   --axis <0> [1]

Known devices:
  336D33573235  →  odrv0
  335A33633235  →  odrv1
"""

import odrive
from odrive.enums import *
import time
import argparse
import sys

# ─────────────────────────────────────────────
#  Known Device Registry
# ─────────────────────────────────────────────

KNOWN_DEVICES = {
    "336D33573235": "odrv0",
    "335A33633235": "odrv1",
}

DEFAULT_SERIALS = list(KNOWN_DEVICES.keys())   # both by default
DEFAULT_AXES    = [0, 1]                        # both axes by default


# ─────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="ODrive 3.6 56V Configuration Script",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--serials",
        nargs="+",
        default=DEFAULT_SERIALS,
        metavar="SERIAL",
        help=(
            "Serial number(s) of ODrive(s) to configure.\n"
            "Default: both known devices\n"
            + "\n".join(f"  {s} → {a}" for s, a in KNOWN_DEVICES.items())
        )
    )
    parser.add_argument(
        "--axis",
        nargs="+",
        choices=["0", "1"],
        default=[str(a) for a in DEFAULT_AXES],
        metavar="{0,1}",
        help="Axis/axes to configure. Default: 0 1 (both)"
    )
    args = parser.parse_args()
    args.serials = [s.upper() for s in args.serials]
    args.axis    = sorted(set(int(a) for a in args.axis))
    return args


# ─────────────────────────────────────────────
#  Error Decoding
# ─────────────────────────────────────────────

AXIS_ERRORS = {
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

AXIS_ERROR_HINTS = {
    0x00000100: "Encoder/Hall sensor issue — check wiring.",
    0x00000040: "Motor not connected or phase wiring issue.",
    0x00000020: "Motor not connected or phase wiring issue.",
    0x00000002: "DC bus under-voltage — check power supply.",
    0x00000004: "DC bus over-voltage — check brake resistor or supply voltage.",
}

def decode_error(err):
    active = [msg for flag, msg in AXIS_ERRORS.items() if err & flag]
    return active or ["UNKNOWN_ERROR"]

def check_error(axis, tag, stage):
    err = axis.error
    if err == 0:
        return True
    print(f"  {tag} ERROR at '{stage}': {hex(err)}")
    for msg in decode_error(err):
        print(f"    → {msg}")
    for flag, hint in AXIS_ERROR_HINTS.items():
        if err & flag:
            print(f"    HINT: {hint}")
    return False

def is_healthy(axis, tag):
    err = axis.error
    if err != 0:
        print(f"  {tag} Pre-existing error {hex(err)} — skipping.")
        for msg in decode_error(err):
            print(f"    → {msg}")
        return False
    return True


# ─────────────────────────────────────────────
#  Main Configuration Function
# ─────────────────────────────────────────────

def configure(serial, axes):
    """
    Full 4-step configuration for one ODrive identified by serial number.
    Steps: erase → apply settings → calibrate → closed-loop.
    Reconnects by serial after every reboot so the correct device is always targeted.
    """
    alias = KNOWN_DEVICES.get(serial, serial)

    def tag(ax):
        return f"[{alias}|axis{ax}]"

    def connect():
        print(f"  Connecting to {alias} (serial={serial})...")
        try:
            dev = odrive.find_any(serial_number=serial, timeout=10)
            print(f"  Connected.")
            return dev
        except Exception as e:
            print(f"  ERROR: Could not connect — {e}")
            sys.exit(1)

    def reboot_and_reconnect(dev):
        try:
            dev.reboot()
        except Exception:
            pass
        print(f"  Rebooting {alias}... waiting 6s")
        time.sleep(6)
        return connect()

    # ── Header ──────────────────────────────
    print("\n" + "=" * 52)
    print(f"  {alias}  |  serial: {serial}  |  axes: {axes}")
    known_tag = "✓ in registry" if serial in KNOWN_DEVICES else "✗ NOT in registry — proceed with caution"
    print(f"  {known_tag}")
    print("=" * 52)

    dev = connect()

    # ── Step 1: Erase ───────────────────────
    print(f"\n[{alias}] 1/4  Erasing configuration...")
    try:
        dev.erase_configuration()
    except Exception:
        pass  # disconnect on erase is expected
    print(f"  Waiting for reboot after erase...")
    time.sleep(6)
    dev = connect()

    # ── Step 2: Apply Settings ──────────────
    print(f"\n[{alias}] 2/4  Applying axis settings...")
    valid_axes = []
    for ax in axes:
        axis = dev.axis0 if ax == 0 else dev.axis1
        t = tag(ax)
        if not is_healthy(axis, t):
            continue

        print(f"  {t} Applying config...")

        # Motor
        axis.motor.config.pole_pairs                   = 15
        axis.motor.config.resistance_calib_max_voltage = 4
        axis.motor.config.requested_current_range      = 25
        axis.motor.config.current_control_bandwidth    = 100
        axis.motor.config.torque_constant              = 0.5168750286102295

        # Encoder (Hall-effect)
        axis.encoder.config.mode                = ENCODER_MODE_HALL
        axis.encoder.config.cpr                 = 90
        axis.encoder.config.calib_scan_distance = 150
        axis.encoder.config.bandwidth           = 100

        # Controller
        tc  = axis.motor.config.torque_constant
        cpr = axis.encoder.config.cpr
        axis.controller.config.pos_gain            = 1
        axis.controller.config.vel_gain            = 0.02 * tc * cpr
        axis.controller.config.vel_integrator_gain = 0.1  * tc * cpr
        axis.controller.config.vel_limit           = 10
        axis.controller.config.control_mode        = CONTROL_MODE_VELOCITY_CONTROL

        print(f"  {t} Settings applied.")
        valid_axes.append(ax)

    if not valid_axes:
        print(f"\n  ERROR [{alias}]: No valid axes. Check motor and encoder connections.")
        return []

    dev.save_configuration()
    dev = reboot_and_reconnect(dev)

    # ── Step 3: Calibrate ───────────────────
    print(f"\n[{alias}] 3/4  Calibrating axes...")
    calibrated_axes = []
    for ax in valid_axes:
        axis = dev.axis0 if ax == 0 else dev.axis1
        t = tag(ax)
        print(f"\n  {t} Starting full calibration (~40s)...")
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        for remaining in range(40, 0, -5):
            print(f"    {t}: {remaining}s remaining...")
            time.sleep(5)

        timeout = 30
        while axis.current_state != AXIS_STATE_IDLE and timeout > 0:
            time.sleep(1)
            timeout -= 1
        if timeout == 0:
            print(f"  {t} WARNING: Timed out waiting for IDLE.")

        if not check_error(axis, t, "calibration"):
            print(f"  {t} Calibration failed — skipping.")
            continue

        axis.motor.config.pre_calibrated   = True
        axis.encoder.config.pre_calibrated = True
        offset = axis.encoder.config.offset_float
        print(f"  {t} Calibration OK — offset_float: {offset:.4f}")
        calibrated_axes.append(ax)

    if not calibrated_axes:
        print(f"\n  ERROR [{alias}]: All axes failed calibration. Check motor and Hall sensor wiring.")
        return []

    dev.save_configuration()
    dev = reboot_and_reconnect(dev)

    # ── Step 4: Closed-loop ─────────────────
    print(f"\n[{alias}] 4/4  Starting closed-loop control...")
    success_axes = []
    for ax in calibrated_axes:
        axis = dev.axis0 if ax == 0 else dev.axis1
        t = tag(ax)
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        if not check_error(axis, t, "closed-loop"):
            print(f"  {t} Could not enter closed-loop. Check motor/encoder connections.")
            continue
        print(f"  {t} Running in CLOSED_LOOP velocity control.")
        success_axes.append(ax)

    return success_axes


# ─────────────────────────────────────────────
#  Entry Point
# ─────────────────────────────────────────────

def main():
    args = parse_args()
    serials = args.serials
    axes    = args.axis

    print("=" * 52)
    print("  ODrive 3.6 56V — Configuration Script")
    print(f"  Devices : {[KNOWN_DEVICES.get(s, s) for s in serials]}")
    print(f"  Axes    : {axes}")
    print("=" * 52)

    # Warn about unknown serials
    unknown = [s for s in serials if s not in KNOWN_DEVICES]
    if unknown:
        print(f"\n  WARNING: Unknown serial(s): {unknown}")
        print(f"  Known  : {list(KNOWN_DEVICES.keys())}")

    # Global confirmation before any changes
    print("\n" + "!" * 52)
    print("  !!          W A R N I N G           !!")
    print("!" * 52)
    print("  Configuration will be ERASED on:")
    for s in serials:
        print(f"    • {KNOWN_DEVICES.get(s, 'UNKNOWN')}  ({s})")
    print("!" * 52)
    try:
        input("\n  Press ENTER to continue, or Ctrl+C to abort...\n")
    except KeyboardInterrupt:
        print("\n\n  Aborted. No changes were made.")
        sys.exit(0)

    # Configure each device sequentially
    results = {}
    for serial in serials:
        results[serial] = configure(serial, axes)

    # Final summary
    print("\n" + "=" * 52)
    print("  SETUP COMPLETE — SUMMARY")
    print("=" * 52)
    for serial, success_axes in results.items():
        alias = KNOWN_DEVICES.get(serial, serial)
        if success_axes:
            print(f"\n  ✓ {alias} ({serial})")
            print(f"    Active axes : {success_axes}")
            for ax in success_axes:
                print(f"    Command     : {alias}.axis{ax}.controller.input_vel = <turns/s>")
        else:
            print(f"\n  ✗ {alias} ({serial}) — FAILED")
    print("=" * 52)


if __name__ == "__main__":
    main()