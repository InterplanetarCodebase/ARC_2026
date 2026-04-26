#!/usr/bin/env python3
"""
misc_receiver.py  —  Runs on Jetson Xavier
==========================================
- Listens for UDP packets from base station (misc_transmitter.py)
- Validates CRC, sequence, deduplication
- Forwards commands to ESP32 via USB Serial
- Reads ACKs from ESP32 and relays them back to base station
- Sends heartbeat to ESP32 watchdog
- Watchdog: if no UDP from base for >2s, sends ESTOP to ESP32
"""

import logging
import socket
import threading
import time
import argparse

import serial

logging.basicConfig(
	level=logging.INFO,
	format="%(asctime)s [MISC_RECEIVER] %(levelname)s %(message)s",
)
log = logging.getLogger(__name__)

# --- CONFIG ------------------------------------------------------
UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 5763
BASE_IP = None
BASE_PORT = None

# Preferred serial port. Use --serial-port to override at runtime.
SERIAL_PORT = "/dev/misc_esp"
SERIAL_BAUD = 115200

SERIAL_CANDIDATES = [
	"/dev/ttyACM1",
	"/dev/ttyACM0",
	"/dev/ttyUSB0",
	"/dev/ttyUSB1",
]

WATCHDOG_TIMEOUT = 2.0
HEARTBEAT_INTERVAL = 0.2

# --- PROTOCOL ----------------------------------------------------
SOF1 = 0xAA
SOF2 = 0xBB
ACK_BYTE = 0xAC
PACKET_LEN = 7  # [SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
ACK_LEN = 4     # [ACK][SEQ_H][SEQ_L][STATUS]

CMD_ESTOP = 0xFF
CMD_HEARTBEAT = 0x00

CMD_NAMES = {
	0x00: "HEARTBEAT",
	0x10: "NIGHT_ON",
	0x11: "NIGHT_OFF",
	0x20: "IND_ON",
	0x21: "IND_OFF",
	0x22: "IND_PWM",
	0x31: "SERVO1",
	0x32: "SERVO2",
	0x33: "SERVO3",
	0x34: "SERVO4",
	0x41: "LASER1_ON",
	0x42: "LASER1_OFF",
	0x43: "LASER2_ON",
	0x44: "LASER2_OFF",
	0xFF: "ESTOP",
}

STATUS_NAMES = {
	0x00: "OK",
	0x01: "CRC_ERR",
	0x02: "UNK_CMD",
	0x03: "ESTOP",
}


def crc8(data: bytes) -> int:
	"""CRC8 poly 0x07 to match ESP32 firmware."""
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


# --- SHARED STATE ------------------------------------------------
last_udp_time = time.time()
last_seq_seen = -1
state_lock = threading.Lock()
esp_seq = 0
ack_buffer = bytearray()


def read_acks(ser: serial.Serial, udp_sock: socket.socket):
	"""Read ACK bytes from ESP32, reassemble, then relay to base."""
	global ack_buffer
	while True:
		try:
			chunk = ser.read(ser.in_waiting or 1)
			if chunk:
				ack_buffer.extend(chunk)
				while len(ack_buffer) >= ACK_LEN:
					if ack_buffer[0] != ACK_BYTE:
						ack_buffer.pop(0)
						continue
					ack = bytes(ack_buffer[:ACK_LEN])
					ack_buffer = ack_buffer[ACK_LEN:]
					seq = (ack[1] << 8) | ack[2]
					status = ack[3]
					log.info(
						"ACK from ESP seq=%d status=%s (0x%02X)",
						seq,
						STATUS_NAMES.get(status, "UNKNOWN"),
						status,
					)
					with state_lock:
						if BASE_IP and BASE_PORT:
							udp_sock.sendto(ack, (BASE_IP, BASE_PORT))
		except Exception as exc:  # pylint: disable=broad-except
			log.error("ACK reader error: %s", exc)
			time.sleep(0.01)


def heartbeat_thread(ser: serial.Serial):
	"""Keep ESP32 watchdog alive and force ESTOP if base link is lost."""
	global esp_seq
	while True:
		time.sleep(HEARTBEAT_INTERVAL)
		try:
			with state_lock:
				elapsed = time.time() - last_udp_time

			if elapsed > WATCHDOG_TIMEOUT:
				pkt = make_estop(esp_seq)
				esp_seq = (esp_seq + 1) & 0xFFFF
				ser.write(pkt)
				log.warning("Base link lost (%.1fs) - ESTOP sent to ESP32", elapsed)
			else:
				pkt = make_heartbeat(esp_seq)
				esp_seq = (esp_seq + 1) & 0xFFFF
				ser.write(pkt)
		except Exception as exc:  # pylint: disable=broad-except
			log.error("Heartbeat error: %s", exc)


def main():
	global BASE_IP, BASE_PORT, last_seq_seen, last_udp_time

	parser = argparse.ArgumentParser(description="Misc board UDP-to-serial receiver")
	parser.add_argument(
		"--serial-port",
		help="Explicit serial port for misc ESP32 (example: /dev/ttyACM1)",
	)
	parser.add_argument(
		"--baud",
		type=int,
		default=SERIAL_BAUD,
		help="Serial baud rate for ESP32 link (default: 115200)",
	)
	args = parser.parse_args()

	ports_to_try = []
	if args.serial_port:
		ports_to_try.append(args.serial_port)
	ports_to_try.append(SERIAL_PORT)
	for port in SERIAL_CANDIDATES:
		if port not in ports_to_try:
			ports_to_try.append(port)

	ser = None
	for port in ports_to_try:
		log.info("Opening serial %s @ %d", port, args.baud)
		try:
			ser = serial.Serial(port, args.baud, timeout=0.01)
			SERIAL_PORT_USED = port
			break
		except serial.SerialException as exc:
			log.warning("Serial open failed on %s: %s", port, exc)

	if ser is None:
		log.error("Cannot open any candidate serial port: %s", ports_to_try)
		return

	log.info("Using serial port %s", SERIAL_PORT_USED)

	udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	udp_sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
	udp_sock.settimeout(0.5)
	log.info("Listening UDP on %s:%d", UDP_LISTEN_IP, UDP_LISTEN_PORT)

	threading.Thread(target=read_acks, args=(ser, udp_sock), daemon=True).start()
	threading.Thread(target=heartbeat_thread, args=(ser,), daemon=True).start()

	log.info("Misc receiver ready - waiting for commands")

	while True:
		try:
			data, addr = udp_sock.recvfrom(64)
		except socket.timeout:
			continue
		except Exception as exc:  # pylint: disable=broad-except
			log.error("UDP recv error: %s", exc)
			continue

		with state_lock:
			BASE_IP = addr[0]
			BASE_PORT = addr[1]

		if len(data) != PACKET_LEN:
			log.warning("Bad packet length %d from %s", len(data), addr)
			continue

		if data[0] != SOF1 or data[1] != SOF2:
			log.warning("Bad SOF bytes")
			continue

		expected_crc = crc8(data[: PACKET_LEN - 1])
		if expected_crc != data[PACKET_LEN - 1]:
			log.warning("CRC mismatch from %s - dropping", addr)
			continue

		seq = (data[2] << 8) | data[3]
		cmd = data[4]
		val = data[5]

		with state_lock:
			if seq == last_seq_seen:
				log.debug("Duplicate seq %d - dropped", seq)
				continue
			last_seq_seen = seq
			last_udp_time = time.time()

		if cmd == CMD_HEARTBEAT:
			log.debug("CMD HEARTBEAT SEQ %d from %s", seq, addr)
		else:
			log.info(
				"CMD %s (0x%02X) VAL %d SEQ %d from %s",
				CMD_NAMES.get(cmd, "UNKNOWN"),
				cmd,
				val,
				seq,
				addr,
			)

		try:
			ser.write(data)
		except serial.SerialException as exc:
			log.error("Serial write error: %s", exc)


if __name__ == "__main__":
	main()
