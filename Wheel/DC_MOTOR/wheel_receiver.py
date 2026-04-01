#!/usr/bin/env python3
"""
wheel_receiver.py  —  Runs on Jetson Xavier
============================================
- Listens for UDP packets from base station on port 5761
- Validates CRC, deduplicates by sequence number
- Forwards drive commands to Arduino Nano via USB Serial
- Relays ACKs back to base station
- Watchdog: if no UDP from base for >2s, sends zero-drive to Arduino (stop)

Throttle architecture note:
  Throttle scaling is performed on the GUI (base station) side.
  x_i8 and z_i8 in the packet already encode the throttle-scaled velocity.
  Byte[6] (formerly throttle) arrives as 0xFF (reserved) and is ignored here.
  This node is a transparent relay — it does not interpret drive values.
"""

import socket
import serial
import threading
import time
import struct
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [WHEEL-RX] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ────────────────────────────────────────────────────────
UDP_LISTEN_IP    = "0.0.0.0"
UDP_LISTEN_PORT  = 5761          # different port from arm (5760)

SERIAL_PORT      = "/dev/ttyUSB0"  # adjust: check `ls /dev/ttyUSB*`
SERIAL_BAUD      = 115200

WATCHDOG_TIMEOUT  = 2.0    # seconds — no UDP → stop wheels
HEARTBEAT_INTERVAL = 0.1   # seconds — send stop packet to keep Arduino watchdog alive

# ─── PROTOCOL ──────────────────────────────────────────────────────
SOF1        = 0xAA
SOF2        = 0xBB
ACK_BYTE    = 0xAC
PACKET_LEN  = 8   # [SOF1][SOF2][SEQ_H][SEQ_L][x_i8][z_i8][throttle][CRC8]
ACK_LEN     = 4   # [ACK][SEQ_H][SEQ_L][STATUS]

# ─── CRC8 (poly 0x07) ──────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def make_drive_packet(seq: int, x_i8: int, z_i8: int) -> bytes:
    """
    x_i8, z_i8 : signed int8 (-127..127), already throttle-scaled by GUI.
    Byte[6] is sent as 0xFF (reserved).
    """
    x_byte = struct.pack('b', max(-127, min(127, x_i8)))[0]
    z_byte = struct.pack('b', max(-127, min(127, z_i8)))[0]
    body = bytes([SOF1, SOF2, (seq >> 8) & 0xFF, seq & 0xFF,
                  x_byte, z_byte, 0xFF])
    return body + bytes([crc8(body)])

def make_stop_packet(seq: int) -> bytes:
    return make_drive_packet(seq, 0, 0)

# ─── SHARED STATE ──────────────────────────────────────────────────
last_udp_time  = time.time()
last_seq_seen  = -1
state_lock     = threading.Lock()
base_addr      = None    # (ip, port) learned from first packet
hb_seq         = 0       # heartbeat/stop sequence counter

# ─── ACK RELAY THREAD ──────────────────────────────────────────────
ack_buf = bytearray()

def read_acks(ser: serial.Serial, udp_sock: socket.socket):
    """Forward Arduino ACKs back to base station."""
    global ack_buf
    while True:
        try:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                ack_buf.extend(chunk)
                while len(ack_buf) >= ACK_LEN:
                    if ack_buf[0] != ACK_BYTE:
                        ack_buf.pop(0)
                        continue
                    ack = bytes(ack_buf[:ACK_LEN])
                    ack_buf = ack_buf[ACK_LEN:]
                    with state_lock:
                        if base_addr:
                            udp_sock.sendto(ack, base_addr)
        except Exception as e:
            log.error(f"ACK reader error: {e}")
            time.sleep(0.01)

# ─── WATCHDOG / HEARTBEAT THREAD ───────────────────────────────────
def heartbeat_thread(ser: serial.Serial):
    """
    Periodically checks if base station link is alive.
    If dead, sends stop commands to keep Arduino watchdog satisfied
    (otherwise the Arduino's own watchdog fires independently, but
    this provides an extra safety layer from the Xavier side).
    """
    global hb_seq, last_udp_time
    while True:
        time.sleep(HEARTBEAT_INTERVAL)
        try:
            with state_lock:
                elapsed = time.time() - last_udp_time

            if elapsed > WATCHDOG_TIMEOUT:
                pkt = make_stop_packet(hb_seq)
                hb_seq = (hb_seq + 1) & 0xFFFF
                ser.write(pkt)
                log.warning(f"Base link lost ({elapsed:.1f}s) — sending STOP to Arduino")
        except Exception as e:
            log.error(f"Heartbeat error: {e}")

# ─── MAIN ──────────────────────────────────────────────────────────
def main():
    global last_udp_time, last_seq_seen, base_addr

    log.info(f"Opening serial {SERIAL_PORT} @ {SERIAL_BAUD}")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
    except serial.SerialException as e:
        log.error(f"Cannot open serial: {e}")
        return

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
    udp_sock.settimeout(0.5)
    log.info(f"Listening UDP on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")

    threading.Thread(target=read_acks,       args=(ser, udp_sock), daemon=True).start()
    threading.Thread(target=heartbeat_thread, args=(ser,),          daemon=True).start()

    log.info("Wheel receiver ready")

    while True:
        try:
            data, addr = udp_sock.recvfrom(64)
        except socket.timeout:
            continue
        except Exception as e:
            log.error(f"UDP recv error: {e}")
            continue

        with state_lock:
            base_addr = addr

        if len(data) != PACKET_LEN:
            log.warning(f"Bad packet length {len(data)}")
            continue

        if data[0] != SOF1 or data[1] != SOF2:
            log.warning("Bad SOF bytes")
            continue

        if crc8(data[:PACKET_LEN - 1]) != data[PACKET_LEN - 1]:
            log.warning(f"CRC mismatch from {addr}")
            continue

        seq = (data[2] << 8) | data[3]

        with state_lock:
            if seq == last_seq_seen:
                log.debug(f"Duplicate seq {seq} — dropped")
                continue
            last_seq_seen = seq
            last_udp_time = time.time()

        x_i8     = struct.unpack('b', bytes([data[4]]))[0]
        z_i8     = struct.unpack('b', bytes([data[5]]))[0]
        throttle = data[6]

        log.debug(f"Drive x={x_i8} z={z_i8} throttle={throttle} seq={seq}")

        try:
            ser.write(data)   # forward the validated packet as-is
        except serial.SerialException as e:
            log.error(f"Serial write error: {e}")

if __name__ == "__main__":
    main()