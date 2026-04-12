#!/usr/bin/env python3
"""
arm_receiver.py  —  Runs on Jetson Xavier
=========================================
- Listens for UDP packets from base station (arm_transmitter.py)
- Validates CRC, sequence, deduplication
- Forwards commands to ESP32 via USB Serial
- Reads ACKs from ESP32 and relays them back to base station
- Reads encoder/pot values from ESP32 serial and relays them to base station
- Sends heartbeat to ESP32 watchdog
- Watchdog: if no UDP from base for >2s, sends ESTOP to ESP32
"""

import socket
import serial
import threading
import time
import logging
import json
import re

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [RECEIVER] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ───────────────────────────────────────────────────────
UDP_LISTEN_IP   = "0.0.0.0"
UDP_LISTEN_PORT = 5760
BASE_IP         = None   # Learned from first packet
BASE_PORT       = None

SERIAL_PORT     = "/dev/ttyUSB0"   # Change to your ESP32 port
SERIAL_BAUD     = 921600

WATCHDOG_TIMEOUT = 4.0   # seconds — no UDP → ESTOP ESP32
HEARTBEAT_INTERVAL = 0.2 # seconds — keep ESP32 watchdog alive
TELEMETRY_MIN_INTERVAL = 0.05  # seconds — encoder relay throttle

# ─── PROTOCOL ─────────────────────────────────────────────────────
SOF1        = 0xAA
SOF2        = 0xBB
ACK_BYTE    = 0xAC
PACKET_LEN  = 7   # [SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
ACK_LEN     = 4   # [ACK][SEQ_H][SEQ_L][STATUS]

CMD_ESTOP     = 0xFF
CMD_HEARTBEAT = 0x00

# ─── CRC8 (poly 0x07) ─────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def make_packet(seq: int, cmd: int, val: int) -> bytes:
    body = bytes([SOF1, SOF2, (seq >> 8) & 0xFF, seq & 0xFF, cmd, val])
    return body + bytes([crc8(body)])

def make_estop(seq: int) -> bytes:
    return make_packet(seq, CMD_ESTOP, 0x00)

def make_heartbeat(seq: int) -> bytes:
    return make_packet(seq, CMD_HEARTBEAT, 0x00)

# ─── SHARED STATE ─────────────────────────────────────────────────
last_udp_time   = time.time()
last_seq_seen   = -1       # deduplication
state_lock      = threading.Lock()
esp_seq         = 0        # our own seq counter for heartbeats

# ─── SERIAL ACK READER ────────────────────────────────────────────
ack_buffer = bytearray()
text_buffer = bytearray()
last_telemetry_sent = 0.0

# Supported serial line formats from firmware:
# 1) ENC1 raw=123 V=0.991 | ENC2 raw=456 V=1.234
# 2) Pin 13 -> Raw: 123 Pin 33 -> Raw: 456
ENC_RE_FMT1 = re.compile(
    r"ENC1\s*raw\s*=\s*(\d+).*?V\s*=\s*([0-9]+(?:\.[0-9]+)?)"
    r".*?ENC2\s*raw\s*=\s*(\d+).*?V\s*=\s*([0-9]+(?:\.[0-9]+)?)",
    re.IGNORECASE,
)
ENC_RE_FMT2 = re.compile(
    r"Pin\s*13\s*->\s*Raw:\s*(\d+).*?Pin\s*33\s*->\s*Raw:\s*(\d+)",
    re.IGNORECASE,
)


def parse_encoder_line(line: str):
    """Return normalized encoder payload dict or None."""
    line = line.strip()
    if not line:
        return None

    m = ENC_RE_FMT1.search(line)
    if m:
        enc1_raw = int(m.group(1))
        enc1_v = float(m.group(2))
        enc2_raw = int(m.group(3))
        enc2_v = float(m.group(4))
        return {
            "type": "encoder",
            "enc1_raw": enc1_raw,
            "enc2_raw": enc2_raw,
            "enc1_v": enc1_v,
            "enc2_v": enc2_v,
            "ts": time.time(),
        }

    m = ENC_RE_FMT2.search(line)
    if m:
        enc1_raw = int(m.group(1))
        enc2_raw = int(m.group(2))
        return {
            "type": "encoder",
            "enc1_raw": enc1_raw,
            "enc2_raw": enc2_raw,
            "enc1_v": enc1_raw * (3.3 / 4095.0),
            "enc2_v": enc2_raw * (3.3 / 4095.0),
            "ts": time.time(),
        }

    return None


def relay_encoder_payload(udp_sock: socket.socket, payload: dict):
    global last_telemetry_sent

    now = time.time()
    if now - last_telemetry_sent < TELEMETRY_MIN_INTERVAL:
        return

    with state_lock:
        base_ip = BASE_IP
        base_port = BASE_PORT

    if not base_ip or not base_port:
        return

    try:
        # Keep telemetry plain and human-readable: raw values are forwarded as-is.
        udp_sock.sendto(json.dumps(payload).encode("utf-8"), (base_ip, base_port))
        last_telemetry_sent = now
    except Exception as e:
        log.error(f"Telemetry relay error: {e}")

def read_acks(ser: serial.Serial, udp_sock: socket.socket):
    """Reads serial stream from ESP32 and relays ACK + encoder telemetry."""
    global ack_buffer, text_buffer
    while True:
        try:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                ack_buffer.extend(chunk)

                # Build a printable-only stream to parse telemetry text lines.
                for b in chunk:
                    if b in (10, 13) or 32 <= b <= 126:
                        text_buffer.append(b)

                while True:
                    nl_idx = text_buffer.find(b"\n")
                    if nl_idx < 0:
                        break
                    line_bytes = bytes(text_buffer[:nl_idx]).strip(b"\r")
                    del text_buffer[:nl_idx + 1]

                    line = line_bytes.decode("utf-8", errors="ignore")
                    payload = parse_encoder_line(line)
                    if payload is not None:
                        relay_encoder_payload(udp_sock, payload)

                # Parse all complete ACKs
                while len(ack_buffer) >= ACK_LEN:
                    if ack_buffer[0] != ACK_BYTE:
                        ack_buffer.pop(0)  # Resync
                        continue
                    ack = bytes(ack_buffer[:ACK_LEN])
                    ack_buffer = ack_buffer[ACK_LEN:]
                    # Relay ACK to base station
                    with state_lock:
                        if BASE_IP and BASE_PORT:
                            udp_sock.sendto(ack, (BASE_IP, BASE_PORT))
        except Exception as e:
            log.error(f"ACK reader error: {e}")
            time.sleep(0.01)

# ─── HEARTBEAT THREAD ─────────────────────────────────────────────
def heartbeat_thread(ser: serial.Serial):
    """Keeps ESP32 watchdog alive. Sends ESTOP if base goes silent."""
    global esp_seq, last_udp_time
    while True:
        time.sleep(HEARTBEAT_INTERVAL)
        try:
            with state_lock:
                elapsed = time.time() - last_udp_time

            if elapsed > WATCHDOG_TIMEOUT:
                # Base station link lost — ESTOP the arm
                pkt = make_estop(esp_seq)
                esp_seq = (esp_seq + 1) & 0xFFFF
                ser.write(pkt)
                log.warning(f"Base link lost ({elapsed:.1f}s) — ESTOP sent to ESP32")
            else:
                # Normal heartbeat
                pkt = make_heartbeat(esp_seq)
                esp_seq = (esp_seq + 1) & 0xFFFF
                ser.write(pkt)
        except Exception as e:
            log.error(f"Heartbeat error: {e}")

# ─── MAIN ─────────────────────────────────────────────────────────
def main():
    global last_udp_time, last_seq_seen, BASE_IP, BASE_PORT

    # Open serial to ESP32
    log.info(f"Opening serial {SERIAL_PORT} @ {SERIAL_BAUD}")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
    except serial.SerialException as e:
        log.error(f"Cannot open serial: {e}")
        return

    # UDP socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
    udp_sock.settimeout(0.5)
    log.info(f"Listening UDP on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")

    # Start ACK relay thread
    t_ack = threading.Thread(target=read_acks, args=(ser, udp_sock), daemon=True)
    t_ack.start()

    # Start heartbeat/watchdog thread
    t_hb = threading.Thread(target=heartbeat_thread, args=(ser,), daemon=True)
    t_hb.start()

    log.info("Receiver ready — waiting for commands")

    while True:
        try:
            data, addr = udp_sock.recvfrom(64)
        except socket.timeout:
            continue
        except Exception as e:
            log.error(f"UDP recv error: {e}")
            continue

        # Learn base station address
        with state_lock:
            BASE_IP   = addr[0]
            BASE_PORT = addr[1]

        if len(data) != PACKET_LEN:
            log.warning(f"Bad packet length {len(data)} from {addr}")
            continue

        # Validate SOF
        if data[0] != SOF1 or data[1] != SOF2:
            log.warning("Bad SOF bytes")
            continue

        # Validate CRC
        expected_crc = crc8(data[:PACKET_LEN - 1])
        if expected_crc != data[PACKET_LEN - 1]:
            log.warning(f"CRC mismatch from {addr} — dropping")
            continue

        seq = (data[2] << 8) | data[3]
        cmd = data[4]
        val = data[5]

        # Deduplication: drop replayed packets
        with state_lock:
            if seq == last_seq_seen:
                log.debug(f"Duplicate seq {seq} — dropped")
                continue
            last_seq_seen = seq
            last_udp_time = time.time()

        log.debug(f"CMD {cmd:#04x} VAL {val} SEQ {seq} from {addr}")

        # Forward to ESP32
        try:
            ser.write(data)
        except serial.SerialException as e:
            log.error(f"Serial write error: {e}")

if __name__ == "__main__":
    main()
