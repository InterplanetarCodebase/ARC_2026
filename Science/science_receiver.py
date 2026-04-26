#!/usr/bin/env python3
"""
science_receiver.py  —  Runs on Jetson (or localhost for testing)
==================================================================
- Listens for UDP packets from base station / GUI
- Validates CRC, sequence, deduplication
- Forwards commands to Science ESP32 via USB Serial
- Reads ACKs from ESP32 and relays them back to base station
- Reads load cell telemetry from ESP32 serial and relays to base station
- Sends heartbeat to ESP32 watchdog
- Watchdog: if no UDP from base for >4s, sends ESTOP to ESP32

Localhost test (no ESP32):
  python3 science_receiver.py --mock
  Then send packets to 127.0.0.1:5761 from your GUI/test script.
"""

import argparse
import socket
import threading
import time
import logging
import json
import re

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [SCI-RECV] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ───────────────────────────────────────────────────────
UDP_LISTEN_IP   = "0.0.0.0"
UDP_LISTEN_PORT = 5761          # Different port from arm (5760)
BASE_IP         = None          # Learned from first packet
BASE_PORT       = None

SERIAL_PORT     = "/dev/science_esp"
SERIAL_BAUD     = 921600

WATCHDOG_TIMEOUT    = 4.0   # seconds — no UDP → ESTOP ESP32
HEARTBEAT_INTERVAL  = 0.2   # seconds — keep ESP32 watchdog alive
TELEMETRY_MIN_INTERVAL = 0.05  # seconds — load cell relay throttle

# ─── PROTOCOL ─────────────────────────────────────────────────────
SOF1        = 0xAA
SOF2        = 0xBB
ACK_BYTE    = 0xAC
PACKET_LEN  = 7   # [SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
ACK_LEN     = 4   # [ACK][SEQ_H][SEQ_L][STATUS]

CMD_ESTOP     = 0xFF
CMD_HEARTBEAT = 0x00

STATUS_OK       = 0x00
STATUS_CRC_ERR  = 0x01
STATUS_UNK_CMD  = 0x02
STATUS_ESTOP    = 0x03
STATUS_NAMES = {
    STATUS_OK:      "OK",
    STATUS_CRC_ERR: "CRC_ERR",
    STATUS_UNK_CMD: "UNK_CMD",
    STATUS_ESTOP:   "ESTOP",
}

CMD_NAMES = {
    0x11: "L298N_A_FWD",  0x12: "L298N_A_REV",  0x13: "L298N_A_STOP",
    0x21: "L298N_B_FWD",  0x22: "L298N_B_REV",  0x23: "L298N_B_STOP",
    0x31: "BTS_FWD",      0x32: "BTS_REV",       0x33: "BTS_STOP",
    0x41: "SERVO1_ANGLE", 0x42: "SERVO2_ANGLE",
    0x43: "SERVO3_ANGLE", 0x44: "SERVO4_ANGLE",
    0x45: "SERVO_CENTER",
    0x51: "LOADCELL_READ", 0x52: "LOADCELL_TARE",
    0x00: "HEARTBEAT",    0xFF: "ESTOP",
}

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
last_seq_seen   = -1
state_lock      = threading.Lock()
esp_seq         = 0

# ─── LOAD CELL TELEMETRY PARSER ───────────────────────────────────
LOAD_RE = re.compile(r"LOAD\s+raw\s*=\s*([+-]?[0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

def parse_load_cell_line(line: str):
    """Return telemetry dict if line is a load cell reading, else None."""
    m = LOAD_RE.search(line.strip())
    if m:
        return {
            "type": "load_cell",
            "raw": float(m.group(1)),
            "ts": time.time(),
        }
    if "LOAD err=" in line:
        return {
            "type": "load_cell_err",
            "msg": line.strip(),
            "ts": time.time(),
        }
    return None

# ─── SERIAL ACK + TELEMETRY READER ───────────────────────────────
ack_buffer          = bytearray()
text_buffer         = bytearray()
last_telemetry_sent = 0.0

def relay_telemetry(udp_sock: socket.socket, payload: dict):
    global last_telemetry_sent
    now = time.time()
    if now - last_telemetry_sent < TELEMETRY_MIN_INTERVAL:
        return
    with state_lock:
        base_ip, base_port = BASE_IP, BASE_PORT
    if not base_ip or not base_port:
        return
    try:
        udp_sock.sendto(json.dumps(payload).encode("utf-8"), (base_ip, base_port))
        last_telemetry_sent = now
    except Exception as e:
        log.error(f"Telemetry relay error: {e}")

def read_acks(ser, udp_sock: socket.socket):
    """Read serial stream from ESP32: relay binary ACKs + text telemetry."""
    global ack_buffer, text_buffer
    while True:
        try:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue

            ack_buffer.extend(chunk)

            # Build printable-only stream for telemetry text parsing
            for b in chunk:
                if b in (10, 13) or 32 <= b <= 126:
                    text_buffer.append(b)

            # Parse complete text lines
            while True:
                nl_idx = text_buffer.find(b"\n")
                if nl_idx < 0:
                    break
                line_bytes = bytes(text_buffer[:nl_idx]).strip(b"\r")
                del text_buffer[:nl_idx + 1]
                line = line_bytes.decode("utf-8", errors="ignore")
                payload = parse_load_cell_line(line)
                if payload is not None:
                    log.debug(f"Telemetry: {payload}")
                    relay_telemetry(udp_sock, payload)

            # Parse all complete binary ACKs
            while len(ack_buffer) >= ACK_LEN:
                if ack_buffer[0] != ACK_BYTE:
                    ack_buffer.pop(0)
                    continue
                ack = bytes(ack_buffer[:ACK_LEN])
                ack_buffer = ack_buffer[ACK_LEN:]
                seq    = (ack[1] << 8) | ack[2]
                status = ack[3]
                log.debug(f"ACK seq={seq} status={STATUS_NAMES.get(status, hex(status))}")
                with state_lock:
                    if BASE_IP and BASE_PORT:
                        udp_sock.sendto(ack, (BASE_IP, BASE_PORT))

        except Exception as e:
            log.error(f"ACK reader error: {e}")
            time.sleep(0.01)

# ─── MOCK SERIAL (localhost testing) ─────────────────────────────
class MockSerial:
    """Fake ESP32 serial — echoes a synthetic ACK for every packet received."""
    def __init__(self):
        self._lock = threading.Lock()
        self._rx   = bytearray()  # data to be read back (ACKs)
        self.in_waiting = 0

    def write(self, data: bytes):
        if len(data) != PACKET_LEN:
            return
        if data[0] != SOF1 or data[1] != SOF2:
            return
        calc = crc8(data[:PACKET_LEN - 1])
        seq  = (data[2] << 8) | data[3]
        cmd  = data[4]
        if calc != data[6]:
            status = STATUS_CRC_ERR
        elif cmd == CMD_ESTOP:
            status = STATUS_ESTOP
        elif cmd not in CMD_NAMES:
            status = STATUS_UNK_CMD
        else:
            status = STATUS_OK
        ack = bytes([ACK_BYTE, data[2], data[3], status])
        cmd_name = CMD_NAMES.get(cmd, hex(cmd))
        log.info(f"[MOCK ESP32] CMD={cmd_name} VAL={data[5]} → {STATUS_NAMES.get(status, hex(status))}")
        with self._lock:
            self._rx.extend(ack)
            # Simulate load cell text response
            if cmd == 0x51:
                self._rx.extend(b"LOAD raw=42.1234\n")
            self.in_waiting = len(self._rx)

    def read(self, n: int) -> bytes:
        with self._lock:
            chunk = bytes(self._rx[:n])
            del self._rx[:n]
            self.in_waiting = len(self._rx)
        return chunk

# ─── HEARTBEAT THREAD ─────────────────────────────────────────────
def heartbeat_thread(ser):
    global esp_seq, last_udp_time
    while True:
        time.sleep(HEARTBEAT_INTERVAL)
        try:
            with state_lock:
                elapsed = time.time() - last_udp_time
            if elapsed > WATCHDOG_TIMEOUT:
                pkt = make_estop(esp_seq)
                esp_seq = (esp_seq + 1) & 0xFFFF
                ser.write(pkt)
                log.warning(f"Base link lost ({elapsed:.1f}s) — ESTOP sent to ESP32")
            else:
                pkt = make_heartbeat(esp_seq)
                esp_seq = (esp_seq + 1) & 0xFFFF
                ser.write(pkt)
        except Exception as e:
            log.error(f"Heartbeat error: {e}")

# ─── MAIN ─────────────────────────────────────────────────────────
def main():
    global last_udp_time, last_seq_seen, BASE_IP, BASE_PORT

    parser = argparse.ArgumentParser(description="Science board receiver")
    parser.add_argument("--mock", action="store_true",
                        help="Use mock serial (localhost testing, no ESP32 needed)")
    parser.add_argument("--port", type=str, default=SERIAL_PORT,
                        help=f"Serial port (default: {SERIAL_PORT})")
    parser.add_argument("--udp-port", type=int, default=UDP_LISTEN_PORT,
                        help=f"UDP listen port (default: {UDP_LISTEN_PORT})")
    args = parser.parse_args()

    # Open serial or mock
    if args.mock:
        ser = MockSerial()
        log.info("Mock serial active — no ESP32 required")
    else:
        try:
            import serial as pyserial
            ser = pyserial.Serial(args.port, SERIAL_BAUD, timeout=0.01)
            log.info(f"Opened serial {args.port} @ {SERIAL_BAUD}")
        except Exception as e:
            log.error(f"Cannot open serial: {e}")
            return

    # UDP socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((UDP_LISTEN_IP, args.udp_port))
    udp_sock.settimeout(0.5)
    log.info(f"Listening UDP on {UDP_LISTEN_IP}:{args.udp_port}")

    t_ack = threading.Thread(target=read_acks, args=(ser, udp_sock), daemon=True)
    t_ack.start()

    t_hb = threading.Thread(target=heartbeat_thread, args=(ser,), daemon=True)
    t_hb.start()

    log.info("Science receiver ready — waiting for commands")

    while True:
        try:
            data, addr = udp_sock.recvfrom(64)
        except socket.timeout:
            continue
        except Exception as e:
            log.error(f"UDP recv error: {e}")
            continue

        with state_lock:
            BASE_IP   = addr[0]
            BASE_PORT = addr[1]

        if len(data) != PACKET_LEN:
            log.warning(f"Bad packet length {len(data)} from {addr}")
            continue

        if data[0] != SOF1 or data[1] != SOF2:
            log.warning(f"Bad SOF from {addr}")
            continue

        if crc8(data[:PACKET_LEN - 1]) != data[PACKET_LEN - 1]:
            log.warning(f"CRC mismatch from {addr} — dropping")
            continue

        seq = (data[2] << 8) | data[3]
        cmd = data[4]
        val = data[5]

        with state_lock:
            if seq == last_seq_seen:
                log.debug(f"Duplicate seq {seq} — dropped")
                continue
            last_seq_seen = seq
            last_udp_time = time.time()

        log.info(f"CMD {CMD_NAMES.get(cmd, hex(cmd))} VAL={val} SEQ={seq} from {addr}")

        try:
            ser.write(data)
        except Exception as e:
            log.error(f"Serial write error: {e}")

if __name__ == "__main__":
    main()
