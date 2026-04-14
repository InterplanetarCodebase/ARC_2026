#!/usr/bin/env python3
"""
pwm_to_odrive.py  —  Runs on Jetson Xavier
===========================================
Reads CH1/CH2 PWM values from Arduino Nano over Serial,
maps them to differential drive velocities, and commands
two ODrive controllers directly.

Serial format from Arduino (20 Hz):
    CH1:XXXX CH2:XXXX
    (0 = signal lost for that channel)

Channel mapping (FlySky default):
    CH1 = steering (left/right yaw)
    CH2 = throttle (forward/back)   ← negated: high PWM = forward

Differential drive:
    left  = x - z
    right = x + z

Motor polarity (matched from ws_to_diffdrive.py):
    odrv0.axis0 = FL  (+left_vel)
    odrv0.axis1 = RL  (+left_vel)
    odrv1.axis0 = FR  (-right_vel)   <- physically inverted
    odrv1.axis1 = RR  (-right_vel)

ODrive reconnect:
    If either ODrive is not found at startup, or drops mid-run,
    the script retries every ODRIVE_RETRY_S seconds until it reconnects.
"""
from __future__ import annotations

import argparse
import logging
import signal
import time

import serial
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [PWM->ODRIVE] %(levelname)s %(message)s",
)
log = logging.getLogger(__name__)

# --- CONFIG -------------------------------------------------------
SERIAL_PORT    = "/dev/ttyUSB0"
BAUD_RATE      = 115200

ODRV0_SERIAL   = "336D33573235"
ODRV1_SERIAL   = "335A33633235"

PWM_MIN        = 1000     # us — full reverse / full left
PWM_MID        = 1500     # us — neutral
PWM_MAX        = 2000     # us — full forward / full right
PWM_DEADBAND   = 50       # us — ignore jitter around neutral
PWM_LOST       = 0        # Arduino sends 0 when signal is stale

VEL_MAX        = 5.0      # turns/s at full stick
CMD_TIMEOUT_S  = 0.5      # stop motors if no valid serial line arrives
ODRIVE_RETRY_S = 5.0      # seconds between ODrive reconnect attempts


# --- ARGS ---------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        description="Direct PWM -> ODrive differential drive bridge."
    )
    p.add_argument("--port",       default=SERIAL_PORT,
                   help="Serial port of Arduino Nano")
    p.add_argument("--max_vel",    type=float, default=VEL_MAX,
                   help="Max wheel velocity in turns/s (1.0-10.0)")
    p.add_argument("--invert_ch1", action="store_true",
                   help="Invert CH1 (steering) direction")
    p.add_argument("--invert_ch2", action="store_true",
                   help="Invert CH2 (throttle) direction")

    args = p.parse_args()
    if not (1.0 <= args.max_vel <= 10.0):
        p.error("--max_vel must be between 1.0 and 10.0")
    return args


# --- PWM MAPPING --------------------------------------------------
def pwm_to_normalized(pwm_us: int, invert: bool = False) -> float:
    """
    Map a PWM pulse width to [-1.0, +1.0].
    Applies deadband around neutral. Returns 0.0 if signal lost.
    """
    if pwm_us == PWM_LOST:
        return 0.0

    pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))

    if abs(pwm_us - PWM_MID) <= PWM_DEADBAND:
        return 0.0

    if pwm_us < PWM_MID:
        norm = (pwm_us - PWM_MID) / (PWM_MID - PWM_MIN)
    else:
        norm = (pwm_us - PWM_MID) / (PWM_MAX - PWM_MID)

    norm = max(-1.0, min(1.0, norm))
    return -norm if invert else norm


def parse_serial_line(line: str) -> tuple[int, int] | None:
    """Parse 'CH1:XXXX CH2:XXXX' -> (ch1_us, ch2_us). None on error."""
    try:
        parts = line.strip().split()
        if len(parts) != 2:
            return None
        ch1 = int(parts[0].split(":")[1])
        ch2 = int(parts[1].split(":")[1])
        return ch1, ch2
    except (IndexError, ValueError):
        return None


# --- ODRIVE SETUP -------------------------------------------------
def setup_axis(ax, name: str = ""):
    log.info(f"[{name}] -> IDLE")
    ax.requested_state = AXIS_STATE_IDLE
    time.sleep(0.3)

    try:
        ax.error = 0
        ax.motor.error = 0
        ax.encoder.error = 0
        ax.controller.error = 0
    except Exception:
        pass

    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode    = INPUT_MODE_VEL_RAMP
    ax.controller.config.vel_ramp_rate = 20.0
    ax.controller.input_vel            = 0.0

    log.info(f"[{name}] -> CLOSED LOOP")
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.6)
    log.info(f"[{name}] state={ax.current_state}  error={ax.error}")


def connect_one(serial_number: str, label: str, shutdown_flag: list):
    """
    Block until the ODrive with the given serial number is found.
    Retries every ODRIVE_RETRY_S seconds. Returns None if shutdown requested.
    """
    while not shutdown_flag[0]:
        try:
            log.info(f"[{label}] Searching for ODrive {serial_number}...")
            odrv = odrive.find_any(serial_number=serial_number, timeout=4)
            log.info(f"[{label}] Connected — serial: {odrv.serial_number}")
            return odrv
        except Exception as e:
            log.warning(
                f"[{label}] Not found ({e}). "
                f"Retrying in {ODRIVE_RETRY_S:.0f}s..."
            )
            time.sleep(ODRIVE_RETRY_S)
    return None


def connect_odrives(shutdown_flag: list) -> tuple:
    """Connect and arm both ODrives. Retries indefinitely until found or shutdown."""
    odrv0 = connect_one(ODRV0_SERIAL, "odrv0", shutdown_flag)
    if odrv0 is None:
        return None, None
    time.sleep(0.5)

    odrv1 = connect_one(ODRV1_SERIAL, "odrv1", shutdown_flag)
    if odrv1 is None:
        return None, None

    setup_axis(odrv0.axis0, "odrv0.axis0 FL")
    setup_axis(odrv0.axis1, "odrv0.axis1 RL")
    setup_axis(odrv1.axis0, "odrv1.axis0 FR")
    setup_axis(odrv1.axis1, "odrv1.axis1 RR")

    return odrv0, odrv1


def reconnect_odrive(serial_number: str, label: str,
                     axes: list[tuple[str, str]], shutdown_flag: list):
    """
    Re-find a single dropped ODrive and re-arm its axes.
      axes: list of (attr, display_name) e.g. [("axis0","FL"), ("axis1","RL")]
    Returns the new odrive object, or None on shutdown.
    """
    odrv = connect_one(serial_number, label, shutdown_flag)
    if odrv is None:
        return None
    for attr, name in axes:
        setup_axis(getattr(odrv, attr), f"{label}.{attr} {name}")
    return odrv


# --- MOTOR COMMANDS -----------------------------------------------
def drive(odrv0, odrv1, left_vel: float, right_vel: float):
    odrv0.axis0.controller.input_vel =  left_vel    # FL
    odrv0.axis1.controller.input_vel =  left_vel    # RL
    odrv1.axis0.controller.input_vel = -right_vel   # FR (inverted)
    odrv1.axis1.controller.input_vel = -right_vel   # RR (inverted)


def stop(odrv0, odrv1):
    drive(odrv0, odrv1, 0.0, 0.0)


def idle_all(odrv0, odrv1):
    for odrv in [odrv0, odrv1]:
        try:
            odrv.axis0.requested_state = AXIS_STATE_IDLE
            odrv.axis1.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass


# --- MAIN ---------------------------------------------------------
def main():
    args = parse_args()
    vel_max = args.max_vel

    # Use a list so nested functions can mutate it
    shutdown_flag = [False]

    def _signal_handler(sig, frame):
        shutdown_flag[0] = True

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Initial connect — retries until both ODrives are found or Ctrl+C
    odrv0, odrv1 = connect_odrives(shutdown_flag)
    if odrv0 is None or odrv1 is None:
        log.info("Shutdown requested before ODrives connected. Exiting.")
        return

    log.info(f"Opening serial port {args.port} at {BAUD_RATE} baud")
    ser = serial.Serial(args.port, BAUD_RATE, timeout=0.1)
    time.sleep(2.0)   # let Arduino reset after DTR toggle
    ser.reset_input_buffer()
    log.info("Serial open. Reading PWM...")

    last_valid_ts   = time.monotonic()
    failsafe_active = False

    try:
        while not shutdown_flag[0]:
            line = ser.readline().decode("ascii", errors="ignore")

            if not line.strip():
                age = time.monotonic() - last_valid_ts
                if age > CMD_TIMEOUT_S and not failsafe_active:
                    try:
                        stop(odrv0, odrv1)
                    except Exception:
                        pass
                    log.warning(f"Failsafe: no data for {age:.2f}s — motors stopped")
                    failsafe_active = True
                continue

            parsed = parse_serial_line(line)
            if parsed is None:
                log.debug(f"Unparseable line: {line.strip()!r}")
                continue

            ch1_us, ch2_us = parsed

            # Both channels lost → failsafe
            if ch1_us == PWM_LOST and ch2_us == PWM_LOST:
                age = time.monotonic() - last_valid_ts
                if age > CMD_TIMEOUT_S and not failsafe_active:
                    try:
                        stop(odrv0, odrv1)
                    except Exception:
                        pass
                    log.warning("Failsafe: both channels lost signal — motors stopped")
                    failsafe_active = True
                continue

            last_valid_ts = time.monotonic()
            if failsafe_active:
                log.info("Signal restored — leaving failsafe")
                failsafe_active = False

            # Map PWM -> normalized [-1, 1]
            # CH2 = throttle (forward/back). Negated: FlySky high PWM = back by default.
            # CH1 = steering (left/right turn).
            x = -pwm_to_normalized(ch2_us, invert=args.invert_ch2)  # forward/back
            z =  pwm_to_normalized(ch1_us, invert=args.invert_ch1)  # steering

            # Differential drive mix
            left_norm  = max(-1.0, min(1.0, x - z))
            right_norm = max(-1.0, min(1.0, x + z))

            left_vel  = left_norm  * vel_max
            right_vel = right_norm * vel_max

            # Drive — catch ODrive disconnect and attempt reconnect
            try:
                drive(odrv0, odrv1, left_vel, right_vel)
            except Exception as e:
                log.error(f"ODrive communication error: {e}")
                log.warning("Attempting to identify which ODrive dropped...")

                # Try to ping each one and reconnect the dead one
                odrv0_ok = True
                odrv1_ok = True
                try:
                    _ = odrv0.vbus_voltage
                except Exception:
                    odrv0_ok = False
                try:
                    _ = odrv1.vbus_voltage
                except Exception:
                    odrv1_ok = False

                if not odrv0_ok:
                    log.warning("odrv0 lost. Reconnecting...")
                    odrv0 = reconnect_odrive(
                        ODRV0_SERIAL, "odrv0",
                        [("axis0", "FL"), ("axis1", "RL")],
                        shutdown_flag,
                    )
                    if odrv0 is None:
                        break

                if not odrv1_ok:
                    log.warning("odrv1 lost. Reconnecting...")
                    odrv1 = reconnect_odrive(
                        ODRV1_SERIAL, "odrv1",
                        [("axis0", "FR"), ("axis1", "RR")],
                        shutdown_flag,
                    )
                    if odrv1 is None:
                        break

                continue  # skip log print for this cycle

            log.info(
                f"CH1={ch1_us:4d}us  CH2={ch2_us:4d}us  |  "
                f"x={x:+.2f}  z={z:+.2f}  |  "
                f"left={left_vel:+.2f}  right={right_vel:+.2f} t/s"
            )

    finally:
        log.info("Shutting down — stopping motors")
        try:
            stop(odrv0, odrv1)
        except Exception:
            pass
        time.sleep(0.5)
        try:
            idle_all(odrv0, odrv1)
        except Exception:
            pass
        time.sleep(0.3)
        ser.close()
        log.info("Done.")


if __name__ == "__main__":
    main()