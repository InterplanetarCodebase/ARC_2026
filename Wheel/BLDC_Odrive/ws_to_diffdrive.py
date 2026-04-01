#!/usr/bin/env python3
"""
ws_to_diffdrive.py  —  Runs on Jetson Xavier
=============================================
WebSocket server that receives drive packets and decodes them.
Accepts connections from two sources (abstracted via same WS interface):
  1. esp_serial_to_ws.py  — RC controller path
  2. Base station GUI     — websocket direct path (future / remote)

Packet format:
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]

Differential drive:
  left  = x - z
  right = x + z

Odrive integration: NOT YET IMPLEMENTED — outputs printed only.
"""

import asyncio
import websockets
import struct
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [WS→DIFFDRIVE] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ────────────────────────────────────────────────────────
WS_HOST     = "0.0.0.0"
WS_PORT     = 8765
PACKET_LEN  = 8

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
def decode_packet(pkt: bytes) -> tuple[int, int, int, int, int] | None:
    """
    Returns (seq, x_i8, z_i8, left, right) or None if invalid.
    left  = x - z
    right = x + z
    Values clamped to [-127, 127].
    """
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

# ─── WEBSOCKET HANDLER ─────────────────────────────────────────────
last_seq: dict[str, int] = {}   # per-client dedup

async def handle_client(websocket):
    client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
    log.info(f"Client connected: {client_id}")
    last_seq[client_id] = -1

    try:
        async for message in websocket:
            if not isinstance(message, bytes):
                log.warning(f"Non-binary message from {client_id} — ignored")
                continue

            result = decode_packet(message)
            if result is None:
                continue

            seq, x_i8, z_i8, left, right = result

            # Deduplicate per client
            if seq == last_seq[client_id]:
                log.debug(f"Duplicate seq {seq} from {client_id} — dropped")
                continue
            last_seq[client_id] = seq

            # ── OUTPUT ──────────────────────────────────────────
            # TODO: replace prints with Odrive commands
            print(
                f"seq={seq:5d} | "
                f"x={x_i8:+4d}  z={z_i8:+4d} | "
                f"left={left:+4d}  right={right:+4d}"
            )

    except websockets.exceptions.ConnectionClosed:
        log.info(f"Client disconnected: {client_id}")
    finally:
        last_seq.pop(client_id, None)

# ─── MAIN ──────────────────────────────────────────────────────────
async def main():
    log.info(f"diffdrive WebSocket server starting on {WS_HOST}:{WS_PORT}")
    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        log.info("Ready — waiting for drive packets")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())