#!/usr/bin/env python3
"""
esp_serial_to_ws.py  —  Runs on Jetson Xavier
==============================================
Reads validated drive packets from ESP32 (RC controller) over USB Serial,
then forwards them to the local WebSocket diffdrive node.

Packet format (from ESP32):
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]

This node is the RC path in the drive architecture:
  RC Transmitter → RC Receiver → ESP32 → [this node] → WebSocket → diffdrive
"""

import asyncio
import serial
import websockets
import logging
import time

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [ESP-SERIAL→WS] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ────────────────────────────────────────────────────────
SERIAL_PORT      = "/dev/ttyUSB0"   # adjust: check `ls /dev/ttyUSB*`
SERIAL_BAUD      = 115200
WS_URI           = "ws://localhost:8765"  # local diffdrive WebSocket node
RECONNECT_DELAY  = 2.0              # seconds between WS reconnect attempts

# ─── PROTOCOL ──────────────────────────────────────────────────────
SOF1        = 0xAA
SOF2        = 0xBB
PACKET_LEN  = 8

# ─── CRC8 (poly 0x07) ──────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

# ─── SERIAL READER ─────────────────────────────────────────────────
def open_serial() -> serial.Serial:
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)
            log.info(f"Opened serial {SERIAL_PORT} @ {SERIAL_BAUD}")
            return ser
        except serial.SerialException as e:
            log.error(f"Cannot open serial: {e} — retrying in {RECONNECT_DELAY}s")
            time.sleep(RECONNECT_DELAY)

def read_next_packet(ser: serial.Serial) -> bytes | None:
    """
    Sync to 0xAA 0xBB header, then read the rest of the packet.
    Returns a validated 8-byte packet, or None if CRC fails or timeout.
    """
    # Scan for SOF1
    b = ser.read(1)
    if not b or b[0] != SOF1:
        return None

    # Check SOF2
    b = ser.read(1)
    if not b or b[0] != SOF2:
        return None

    # Read remaining 6 bytes
    rest = ser.read(6)
    if len(rest) < 6:
        return None

    pkt = bytes([SOF1, SOF2]) + rest

    # Validate CRC
    if crc8(pkt[:7]) != pkt[7]:
        log.warning("CRC mismatch — packet dropped")
        return None

    return pkt

# ─── MAIN LOOP ─────────────────────────────────────────────────────
async def serial_to_ws():
    ser = open_serial()
    last_seq = -1

    while True:
        try:
            log.info(f"Connecting to WebSocket at {WS_URI} ...")
            async with websockets.connect(WS_URI) as ws:
                log.info("WebSocket connected")

                while True:
                    # Serial read is blocking — run in executor to avoid
                    # blocking the asyncio event loop
                    pkt = await asyncio.get_event_loop().run_in_executor(
                        None, read_next_packet, ser
                    )

                    if pkt is None:
                        continue

                    seq = (pkt[2] << 8) | pkt[3]

                    # Deduplicate
                    if seq == last_seq:
                        log.debug(f"Duplicate seq {seq} — dropped")
                        continue
                    last_seq = seq

                    await ws.send(pkt)
                    log.debug(f"Forwarded packet seq={seq}")

        except (websockets.exceptions.ConnectionClosed,
                ConnectionRefusedError,
                OSError) as e:
            log.warning(f"WebSocket error: {e} — reconnecting in {RECONNECT_DELAY}s")
            await asyncio.sleep(RECONNECT_DELAY)

        except serial.SerialException as e:
            log.error(f"Serial lost: {e} — reopening")
            ser = open_serial()

if __name__ == "__main__":
    asyncio.run(serial_to_ws())