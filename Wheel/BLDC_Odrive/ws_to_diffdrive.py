#!/usr/bin/env python3
"""
ws_to_diffdrive.py  —  Runs on Jetson Xavier
=============================================
WebSocket server that receives drive packets from Arduino Nano
and drives ODrive motors with differential drive.

Packet format:
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]

Differential drive:
  left  = x - z
  right = x + z

Polarity (matched from experimental odrive_test.py):
  odrv0.axis0 = FL (+left_vel)
  odrv0.axis1 = RL (+left_vel)
  odrv1.axis0 = FR (-right_vel)   ← right side physically inverted
  odrv1.axis1 = RR (-right_vel)
"""
from __future__ import annotations
import asyncio
import json
import websockets
import struct
import logging
import time
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [WS→DIFFDRIVE] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ────────────────────────────────────────────────────────
WS_HOST      = "0.0.0.0"
WS_PORT      = 8765
PACKET_LEN   = 8

ODRV0_SERIAL = "336D33573235"
ODRV1_SERIAL = "335A33633235"

VEL_MAX      = 4.0    # max turns/s sent to ODrive — tune this
CMD_TIMEOUT_S = 0.35   # safety timeout: stop if no fresh command arrives
WATCHDOG_PERIOD_S = 0.05

# ─── PROTOCOL ──────────────────────────────────────────────────────
SOF1 = 0xAA
SOF2 = 0xBB

# ─── CRC8 (poly 0x07) ──────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

# ─── DIFF DRIVE DECODE ─────────────────────────────────────────────
def decode_packet(pkt: bytes):
    if len(pkt) != PACKET_LEN:
        log.warning(f"Bad packet length: {len(pkt)}")
        return None
    if pkt[0] != SOF1 or pkt[1] != SOF2:
        log.warning("Bad SOF bytes")
        return None
    if crc8(pkt[:7]) != pkt[7]:
        log.warning("CRC mismatch")
        return None

    seq  = (pkt[2] << 8) | pkt[3]
    x_i8 = struct.unpack('b', bytes([pkt[4]]))[0]
    z_i8 = struct.unpack('b', bytes([pkt[5]]))[0]

    left  = max(-127, min(127, x_i8 - z_i8))
    right = max(-127, min(127, x_i8 + z_i8))

    return seq, x_i8, z_i8, left, right


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def build_telemetry(odrv0, odrv1, seq: int) -> dict:
    return {
        "type": "odrive_telemetry",
        "seq": seq,
        "t_monotonic": time.monotonic(),
        "odrv0": {
            "vbus_voltage": _safe_float(getattr(odrv0, "vbus_voltage", None)),
            "axes": {
                "axis0": {
                    "name": "FL",
                    "pos_estimate": _safe_float(odrv0.axis0.encoder.pos_estimate),
                    "iq_measured": _safe_float(odrv0.axis0.motor.current_control.Iq_measured),
                    "id_measured": _safe_float(odrv0.axis0.motor.current_control.Id_measured),
                    "ibus": _safe_float(odrv0.axis0.motor.current_control.Ibus),
                },
                "axis1": {
                    "name": "RL",
                    "pos_estimate": _safe_float(odrv0.axis1.encoder.pos_estimate),
                    "iq_measured": _safe_float(odrv0.axis1.motor.current_control.Iq_measured),
                    "id_measured": _safe_float(odrv0.axis1.motor.current_control.Id_measured),
                    "ibus": _safe_float(odrv0.axis1.motor.current_control.Ibus),
                },
            },
        },
        "odrv1": {
            "vbus_voltage": _safe_float(getattr(odrv1, "vbus_voltage", None)),
            "axes": {
                "axis0": {
                    "name": "FR",
                    "pos_estimate": _safe_float(odrv1.axis0.encoder.pos_estimate),
                    "iq_measured": _safe_float(odrv1.axis0.motor.current_control.Iq_measured),
                    "id_measured": _safe_float(odrv1.axis0.motor.current_control.Id_measured),
                    "ibus": _safe_float(odrv1.axis0.motor.current_control.Ibus),
                },
                "axis1": {
                    "name": "RR",
                    "pos_estimate": _safe_float(odrv1.axis1.encoder.pos_estimate),
                    "iq_measured": _safe_float(odrv1.axis1.motor.current_control.Iq_measured),
                    "id_measured": _safe_float(odrv1.axis1.motor.current_control.Id_measured),
                    "ibus": _safe_float(odrv1.axis1.motor.current_control.Ibus),
                },
            },
        },
    }

# ─── ODRIVE SETUP ──────────────────────────────────────────────────
def setup_axis(ax, name=""):
    log.info(f"[{name}] → IDLE")
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
    ax.controller.input_vel = 0.0

    log.info(f"[{name}] → CLOSED LOOP")
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.6)

    log.info(f"[{name}] state={ax.current_state}  error={ax.error}")


def connect_odrives():
    log.info("Connecting odrv0...")
    odrv0 = odrive.find_any(serial_number=ODRV0_SERIAL)
    time.sleep(1)   # prevent fibre race

    log.info("Connecting odrv1...")
    odrv1 = odrive.find_any(serial_number=ODRV1_SERIAL)

    log.info(f"odrv0 serial: {odrv0.serial_number}")
    log.info(f"odrv1 serial: {odrv1.serial_number}")

    setup_axis(odrv0.axis0, "odrv0.axis0 FL")
    setup_axis(odrv0.axis1, "odrv0.axis1 RL")
    setup_axis(odrv1.axis0, "odrv1.axis0 FR")
    setup_axis(odrv1.axis1, "odrv1.axis1 RR")

    return odrv0, odrv1

# ─── MOTOR HELPERS ─────────────────────────────────────────────────
def drive(odrv0, odrv1, left_vel: float, right_vel: float):
    """
    Apply velocities with experimental polarity from odrive_test.py.
    Right side motors are physically inverted → negate right_vel.
    """
    odrv0.axis0.controller.input_vel =  left_vel   # FL
    odrv0.axis1.controller.input_vel =  left_vel   # RL
    odrv1.axis0.controller.input_vel = -right_vel  # FR
    odrv1.axis1.controller.input_vel = -right_vel  # RR


def stop(odrv0, odrv1):
    drive(odrv0, odrv1, 0.0, 0.0)


def idle_all(odrv0, odrv1):
    for odrv in [odrv0, odrv1]:
        try:
            odrv.axis0.requested_state = AXIS_STATE_IDLE
            odrv.axis1.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

# ─── WEBSOCKET HANDLER ─────────────────────────────────────────────
last_seq: dict[str, int] = {}
connected_clients: dict[str, object] = {}
command_clients: set[str] = set()
last_command_ts: float = 0.0
failsafe_active: bool = False


async def broadcast_telemetry(payload: str):
    stale_clients = []
    for client_id, ws in list(connected_clients.items()):
        try:
            await ws.send(payload)
        except Exception:
            stale_clients.append(client_id)

    for client_id in stale_clients:
        connected_clients.pop(client_id, None)
        last_seq.pop(client_id, None)
        command_clients.discard(client_id)
        log.info(f"Client removed during broadcast: {client_id}")


async def command_watchdog(odrv0, odrv1):
    global failsafe_active
    while True:
        await asyncio.sleep(WATCHDOG_PERIOD_S)

        # Only enforce timeout when at least one client has issued commands.
        if not command_clients:
            continue

        age = time.monotonic() - last_command_ts
        if age > CMD_TIMEOUT_S:
            stop(odrv0, odrv1)
            if not failsafe_active:
                log.warning(
                    f"Failsafe: no command for {age:.3f}s (> {CMD_TIMEOUT_S:.3f}s). Motors set to zero."
                )
            failsafe_active = True

def make_handler(odrv0, odrv1):
    async def handle_client(websocket):
        global last_command_ts, failsafe_active
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        log.info(f"Client connected: {client_id}")
        last_seq[client_id] = -1
        connected_clients[client_id] = websocket

        try:
            async for message in websocket:
                if not isinstance(message, bytes):
                    log.warning(f"Non-binary from {client_id} — ignored")
                    continue

                result = decode_packet(message)
                if result is None:
                    continue

                seq, x_i8, z_i8, left, right = result

                # Deduplicate
                if seq == last_seq[client_id]:
                    log.debug(f"Duplicate seq {seq} — dropped")
                    continue
                last_seq[client_id] = seq

                # Scale [-127, 127] → [-VEL_MAX, +VEL_MAX]
                left_vel  = (left  / 127.0) * VEL_MAX
                right_vel = (right / 127.0) * VEL_MAX

                command_clients.add(client_id)
                drive(odrv0, odrv1, left_vel, right_vel)
                last_command_ts = time.monotonic()
                if failsafe_active:
                    log.info("Command stream restored. Leaving failsafe state.")
                    failsafe_active = False

                telemetry = build_telemetry(odrv0, odrv1, seq)
                await broadcast_telemetry(json.dumps(telemetry))

                log.info(
                    f"seq={seq:5d} | "
                    f"x={x_i8:+4d} z={z_i8:+4d} | "
                    f"left={left_vel:+6.2f}  right={right_vel:+6.2f} turns/s"
                )

        except websockets.exceptions.ConnectionClosed:
            log.info(f"Client disconnected: {client_id}")
        finally:
            connected_clients.pop(client_id, None)
            last_seq.pop(client_id, None)
            command_clients.discard(client_id)
            if not command_clients:
                log.info(f"Stopping motors (last command client gone: {client_id})")
                stop(odrv0, odrv1)   # safety stop only when no command clients remain
            else:
                log.info(f"Command client gone: {client_id} (others still active)")

    return handle_client

# ─── MAIN ──────────────────────────────────────────────────────────
async def main():
    odrv0, odrv1 = connect_odrives()
    watchdog_task = asyncio.create_task(command_watchdog(odrv0, odrv1))

    log.info(f"WebSocket server starting on {WS_HOST}:{WS_PORT}")
    async with websockets.serve(make_handler(odrv0, odrv1), WS_HOST, WS_PORT):
        log.info("Ready — waiting for drive packets")
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            pass
        finally:
            watchdog_task.cancel()
            try:
                await watchdog_task
            except asyncio.CancelledError:
                pass

            log.info("Shutting down — stopping and idling motors")
            stop(odrv0, odrv1)
            time.sleep(0.2)
            idle_all(odrv0, odrv1)

if __name__ == "__main__":
    asyncio.run(main())