#!/usr/bin/env python3
"""
YOU MUST SET THE ROVER_IP ADDRESS TO YOUR ROVER'S IP FOR THIS TO WORK.

Logitech Extreme 3D Pro - Rover Arm Control + BLDC Diff Drive
Primary GUI architecture follows working_gui.py.
BLDC drive packet format follows ws_to_diffdrive.py:
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]

Install dependencies:
  pip install pygame websockets
"""

import os
import sys
import socket
import time
import struct
import json

# Must be set before pygame.init() so joystick updates continue in background.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame

try:
	from websockets.sync.client import connect as ws_connect
	from websockets.exceptions import WebSocketException
	WEBSOCKETS_OK = True
except Exception:
	ws_connect = None
	WebSocketException = Exception
	WEBSOCKETS_OK = False


# -- Network -----------------------------------------------------------------
ROVER_IP = "192.168.10.176"              # CHANGE THIS to your rover's IP address
ROVER_PORT = 5760                      # UDP port for arm commands

DRIVE_WS_PORT = 8765                   # ws_to_diffdrive.py WebSocket port
DRIVE_SEND_RATE = 0.05                 # 20 Hz
DEADZONE = 0.08
DRIVE_SENSITIVITY_X = 0.55             # lower = less forward/back sensitivity
DRIVE_SENSITIVITY_Z = 0.50             # lower = less turning sensitivity
DRIVE_EXPO = 1.8                       # >1 gives finer control near center

DRIVE_SOF1 = 0xAA
DRIVE_SOF2 = 0xBB
DRIVE_RESERVED = 0xFF

SOF1 = 0xAA
SOF2 = 0xBB

CMD_MOTOR1_FWD = 0x11
CMD_MOTOR1_REV = 0x12
CMD_MOTOR1_STOP = 0x13
CMD_MOTOR2_FWD = 0x21
CMD_MOTOR2_REV = 0x22
CMD_MOTOR2_STOP = 0x23
CMD_MOTOR3_FWD = 0x31
CMD_MOTOR3_REV = 0x32
CMD_MOTOR3_STOP = 0x33
CMD_MOTOR4A_FWD = 0x41
CMD_MOTOR4A_REV = 0x42
CMD_MOTOR4A_STOP = 0x43
CMD_MOTOR4B_FWD = 0x51
CMD_MOTOR4B_REV = 0x52
CMD_MOTOR4B_STOP = 0x53
CMD_SERVO_ANGLE = 0x60

SERVO_MIN = 0
SERVO_MAX = 180
SERVO_STEP = 2
SEND_RATE = 0.05

# Match ws_to_diffdrive.py scaling for preview values.
VEL_MAX_TURNS_PER_S =  4.0

# -- Colour palette -----------------------------------------------------------
BG = (0, 0, 0)
ORANGE_HI = (255, 140, 0)
ORANGE_MID = (200, 90, 0)
ORANGE_DIM = (150, 75, 0)
ORANGE_GLOW = (255, 180, 60)
BORDER_LINE = (120, 55, 0)
RED_ERR = (255, 50, 20)

MOTOR_NAMES = ["M1 (HAT N/S)", "M2 (HAT W/E)", "M3 (B3/B4)",
			   "M4A (B5/B6)", "M4B (B7/B8)"]

BUTTON_LABELS = [
	"TRIGGER", "THUMB", "BTN 3", "BTN 4", "BTN 5", "BTN 6",
	"BTN 7", "BTN 8", "BTN 9", "BTN 10", "BTN 11", "BTN 12"
]

BASE_W, BASE_H = 960, 700


# -- Helpers -----------------------------------------------------------------
def crc8(data: bytes) -> int:
	crc = 0
	for byte in data:
		crc ^= byte
		for _ in range(8):
			crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
	return crc


def clamp_i8(v: int) -> int:
	return max(-127, min(127, v))


def shape_drive_axis(value: float, deadzone: float, sensitivity: float, expo: float) -> float:
	"""Apply deadzone + exponential curve to reduce joystick sensitivity."""
	a = abs(value)
	if a <= deadzone:
		return 0.0

	sign = 1.0 if value >= 0.0 else -1.0
	norm = (a - deadzone) / (1.0 - deadzone)
	shaped = (norm ** expo) * sensitivity
	return sign * max(-1.0, min(1.0, shaped))


# -- UDP arm sender -----------------------------------------------------------
class ArmSender:
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.addr = (ROVER_IP, ROVER_PORT)
		self.seq = 0
		self.last_error = ""

	def send(self, cmd, val=0):
		body = bytes([
			SOF1, SOF2,
			(self.seq >> 8) & 0xFF, self.seq & 0xFF,
			cmd, val & 0xFF
		])
		pkt = body + bytes([crc8(body)])
		try:
			self.sock.sendto(pkt, self.addr)
			self.last_error = ""
		except OSError as e:
			self.last_error = str(e)
		self.seq = (self.seq + 1) & 0xFFFF


# -- WebSocket BLDC drive sender ---------------------------------------------
class BldcDriveSender:
	def __init__(self):
		self.url = f"ws://{ROVER_IP}:{DRIVE_WS_PORT}"
		self.seq = 0
		self.ws = None
		self.last_error = ""
		self.telemetry = {}
		self.last_telemetry_at = 0.0
		self.connected = False
		self.next_reconnect_at = 0.0
		self.reconnect_interval = 1.0

		if not WEBSOCKETS_OK:
			self.last_error = "websockets package missing (pip install websockets)"

	def _try_connect(self):
		if not WEBSOCKETS_OK:
			self.connected = False
			return

		now = time.monotonic()
		if now < self.next_reconnect_at:
			return

		try:
			self.ws = ws_connect(self.url, open_timeout=0.25, close_timeout=0.25)
			self.connected = True
			self.last_error = ""
		except Exception as e:
			self.connected = False
			self.last_error = f"WS connect failed: {e}"
			self.next_reconnect_at = now + self.reconnect_interval

	def _disconnect(self, err_msg=""):
		if self.ws is not None:
			try:
				self.ws.close()
			except Exception:
				pass
		self.ws = None
		self.connected = False
		if err_msg:
			self.last_error = err_msg
		self.next_reconnect_at = time.monotonic() + self.reconnect_interval

	def _poll_telemetry(self, max_messages=3):
		if self.ws is None or not self.connected:
			return

		for _ in range(max_messages):
			try:
				msg = self.ws.recv(timeout=0.0)
			except TimeoutError:
				break
			except (OSError, WebSocketException, RuntimeError) as e:
				self._disconnect(f"WS recv failed: {e}")
				break

			if msg is None:
				break

			try:
				if isinstance(msg, bytes):
					msg = msg.decode("utf-8", errors="ignore")
				if not isinstance(msg, str):
					continue
				data = json.loads(msg)
				if isinstance(data, dict) and data.get("type") == "odrive_telemetry":
					self.telemetry = data
					self.last_telemetry_at = time.monotonic()
			except Exception:
				continue

	def send_drive(self, x: float, z: float):
		"""
		Build and send ws_to_diffdrive-compatible packet:
		[AA][BB][SEQ_H][SEQ_L][x_i8][z_i8][FF][CRC8]
		"""
		if self.ws is None or not self.connected:
			self._try_connect()
		if self.ws is None or not self.connected:
			self.seq = (self.seq + 1) & 0xFFFF
			return None

		x_i8 = clamp_i8(int(x * 127.0))
		z_i8 = clamp_i8(int(z * 127.0))
		x_b = struct.pack("b", x_i8)[0]
		z_b = struct.pack("b", z_i8)[0]

		body = bytes([
			DRIVE_SOF1,
			DRIVE_SOF2,
			(self.seq >> 8) & 0xFF,
			self.seq & 0xFF,
			x_b,
			z_b,
			DRIVE_RESERVED,
		])
		pkt = body + bytes([crc8(body)])

		try:
			self.ws.send(pkt)
			self.last_error = ""
			self._poll_telemetry()
		except (OSError, WebSocketException, RuntimeError) as e:
			self._disconnect(f"WS send failed: {e}")
		finally:
			self.seq = (self.seq + 1) & 0xFFFF

		left_cmd = clamp_i8(x_i8 - z_i8)
		right_cmd = clamp_i8(x_i8 + z_i8)
		left_vel = (left_cmd / 127.0) * VEL_MAX_TURNS_PER_S
		right_vel = (right_cmd / 127.0) * VEL_MAX_TURNS_PER_S
		return x_i8, z_i8, left_cmd, right_cmd, left_vel, right_vel

	def close(self):
		self._disconnect()


# -- Draw helpers -------------------------------------------------------------
def draw_panel(surf, rect, title=None):
	pygame.draw.rect(surf, (4, 4, 4), rect, border_radius=3)
	if title:
		pygame.draw.rect(surf, ORANGE_HI,
						 (rect.x + 10, rect.y + 10, 2, 14), border_radius=1)
		lbl = FONT_SM.render(title, True, ORANGE_HI)
		surf.blit(lbl, (rect.x + 18, rect.y + 10))


def draw_hline(surf, x, y, w):
	pass


def draw_axis_bar(surf, x, y, w, h, value, label="", show_val=True):
	pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
	pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)
	cx = x + w // 2
	pygame.draw.line(surf, ORANGE_DIM, (cx, y + 2), (cx, y + h - 2))

	pct = (value + 1) / 2.0
	if value >= 0:
		fill_x = cx
		fill_w = int((pct - 0.5) * w)
	else:
		fill_x = x + int(pct * w)
		fill_w = cx - fill_x

	if fill_w > 0:
		pygame.draw.rect(surf, ORANGE_MID,
						 pygame.Rect(fill_x, y + 2, fill_w, h - 4),
						 border_radius=2)

	if label:
		surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x, y - 15))
	if show_val:
		val_surf = FONT_XS.render(f"{value:+.3f}", True, ORANGE_HI)
		surf.blit(val_surf, (x + w - val_surf.get_width(), y - 15))


def draw_throttle_v(surf, x, y, w, h, value):
	pct = 1.0 - (value + 1) / 2.0
	pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
	pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)
	fill_h = int(pct * (h - 4))
	if fill_h > 0:
		pygame.draw.rect(surf, ORANGE_MID,
						 (x + 2, y + h - 2 - fill_h, w - 4, fill_h),
						 border_radius=2)
	lbl = FONT_XS.render("THR", True, ORANGE_DIM)
	surf.blit(lbl, (x + w // 2 - lbl.get_width() // 2, y - 15))
	val = FONT_XS.render(f"{int(pct * 100)}%", True, ORANGE_HI)
	surf.blit(val, (x + w // 2 - val.get_width() // 2, y + h + 4))


def draw_xy_pad(surf, x, y, size, ax, ay):
	pygame.draw.rect(surf, (8, 8, 8), (x, y, size, size), border_radius=3)
	pygame.draw.rect(surf, ORANGE_DIM, (x, y, size, size), width=1, border_radius=3)
	cx, cy = x + size // 2, y + size // 2
	for t in (0.25, 0.5, 0.75):
		gx = x + int(t * size)
		gy = y + int(t * size)
		pygame.draw.line(surf, (20, 8, 0), (gx, y + 1), (gx, y + size - 1))
		pygame.draw.line(surf, (20, 8, 0), (x + 1, gy), (x + size - 1, gy))
	pygame.draw.line(surf, ORANGE_DIM, (cx, y + 2), (cx, y + size - 2))
	pygame.draw.line(surf, ORANGE_DIM, (x + 2, cy), (x + size - 2, cy))

	dx = cx + int(ax * (size // 2 - 6))
	dy = cy + int(ay * (size // 2 - 6))
	pygame.draw.circle(surf, ORANGE_HI, (dx, dy), 6)
	pygame.draw.circle(surf, ORANGE_GLOW, (dx, dy), 2)
	surf.blit(FONT_XS.render("X/Y", True, ORANGE_DIM),
			  (x + size // 2 - FONT_XS.size("X/Y")[0] // 2, y - 15))


def draw_button_widget(surf, x, y, w, h, pressed, label):
	bg = ORANGE_DIM if pressed else (8, 8, 8)
	bc = ORANGE_HI if pressed else ORANGE_DIM
	pygame.draw.rect(surf, bg, (x, y, w, h), border_radius=3)
	pygame.draw.rect(surf, bc, (x, y, w, h), width=1, border_radius=3)
	dot_c = ORANGE_GLOW if pressed else (30, 12, 0)
	pygame.draw.circle(surf, dot_c, (x + w // 2, y + 9), 4)
	lbl = FONT_XS.render(label, True, ORANGE_HI if pressed else ORANGE_DIM)
	surf.blit(lbl, (x + w // 2 - lbl.get_width() // 2, y + 18))


def draw_pov(surf, cx, cy, size, hat_x, hat_y):
	active_dir = (
		1 if hat_x > 0 else (-1 if hat_x < 0 else 0),
		1 if hat_y > 0 else (-1 if hat_y < 0 else 0),
	)
	dirs = {
		(0, 0): ".", (0, -1): "N", (0, 1): "S", (-1, 0): "W", (1, 0): "E",
		(-1, -1): "NW", (1, -1): "NE", (-1, 1): "SW", (1, 1): "SE"
	}
	cell = size // 3
	for row in range(3):
		for col in range(3):
			dx = col - 1
			dy = row - 1
			is_center = (dx == 0 and dy == 0)
			is_active = (dx, dy) == (active_dir[0], -active_dir[1]) and active_dir != (0, 0)
			rx = cx - size // 2 + col * cell + 2
			ry = cy - size // 2 + row * cell + 2
			rw = cell - 4
			rh = cell - 4
			bg = ORANGE_DIM if is_active else (8, 8, 8)
			bc = ORANGE_HI if is_active else ORANGE_DIM
			pygame.draw.rect(surf, bg, (rx, ry, rw, rh), border_radius=2)
			pygame.draw.rect(surf, bc, (rx, ry, rw, rh), width=1, border_radius=2)
			if is_center:
				pygame.draw.circle(
					surf,
					ORANGE_HI if active_dir != (0, 0) else (30, 12, 0),
					(rx + rw // 2, ry + rh // 2),
					4,
				)
	lbl = FONT_XS.render("POV HAT", True, ORANGE_DIM)
	surf.blit(lbl, (cx - lbl.get_width() // 2, cy - size // 2 - 16))
	dir_label = dirs.get((active_dir[0], active_dir[1]), ".")
	d = FONT_MD.render(dir_label, True, ORANGE_HI)
	surf.blit(d, (cx - d.get_width() // 2, cy + size // 2 + 4))


def draw_trigger(surf, cx, cy, pressed):
	c = RED_ERR if pressed else (8, 8, 8)
	bc = RED_ERR if pressed else ORANGE_DIM
	pygame.draw.circle(surf, c, (cx, cy), 24)
	pygame.draw.circle(surf, bc, (cx, cy), 24, width=2)
	pygame.draw.circle(surf, ORANGE_GLOW if pressed else (30, 12, 0), (cx, cy), 8)
	surf.blit(FONT_XS.render("TRIGGER", True, ORANGE_DIM),
			  (cx - FONT_XS.size("TRIGGER")[0] // 2, cy - 42))
	state_lbl = "FIRE" if pressed else "SAFE"
	s = FONT_XS.render(state_lbl, True, RED_ERR if pressed else ORANGE_DIM)
	surf.blit(s, (cx - s.get_width() // 2, cy + 30))


def draw_status_badge(surf, connected, name=""):
	color = ORANGE_HI if connected else RED_ERR
	label = "CTRL CONNECTED" if connected else "NO CONTROLLER"
	badge = FONT_XS.render(label, True, color)
	bw = badge.get_width() + 26
	bx = BASE_W - bw - 18
	pygame.draw.rect(surf, (6, 2, 0), (bx, 12, bw, 22), border_radius=2)
	pygame.draw.rect(surf, color, (bx, 12, bw, 22), width=1, border_radius=2)
	pygame.draw.circle(surf, color, (bx + 10, 23), 3)
	surf.blit(badge, (bx + 18, 17))
	if name and connected:
		n = FONT_XS.render(name[:50], True, ORANGE_DIM)
		surf.blit(n, (bx, 38))


def draw_rover_panel(surf, rect, motor_states, servo_angle, net_ok, packets_sent):
	draw_panel(surf, rect, "ARM / MOTOR COMMANDS")
	x0, y0 = rect.x + 14, rect.y + 36

	col_w = (rect.w - 28) // len(MOTOR_NAMES)
	for i, (name, state) in enumerate(zip(MOTOR_NAMES, motor_states)):
		cx = x0 + i * col_w + col_w // 2
		lbl = FONT_XS.render(name, True, ORANGE_DIM)
		surf.blit(lbl, (cx - lbl.get_width() // 2, y0))

		sc = ORANGE_HI if state == "FWD" else (RED_ERR if state == "REV" else (25, 10, 0))
		bc = sc if state != "STOP" else ORANGE_DIM
		bx = cx - 24
		by = y0 + 16
		pygame.draw.rect(surf, (6, 2, 0), (bx, by, 48, 16), border_radius=2)
		pygame.draw.rect(surf, bc, (bx, by, 48, 16), width=1, border_radius=2)
		st = FONT_XS.render(state, True, sc)
		surf.blit(st, (cx - st.get_width() // 2, by + 2))

	sy = y0 + 48
	surf.blit(FONT_XS.render("SERVO ANGLE", True, ORANGE_DIM), (x0, sy))
	bar_x = x0 + 90
	bar_w = rect.w - 110
	pygame.draw.rect(surf, (8, 8, 8), (bar_x, sy, bar_w, 12), border_radius=2)
	pygame.draw.rect(surf, ORANGE_DIM, (bar_x, sy, bar_w, 12), width=1, border_radius=2)
	fill = int((servo_angle / 180) * (bar_w - 4))
	pygame.draw.rect(surf, ORANGE_MID, (bar_x + 2, sy + 2, fill, 8), border_radius=2)
	ang = FONT_XS.render(f"{servo_angle} deg", True, ORANGE_HI)
	surf.blit(ang, (bar_x + bar_w + 6, sy))

	ny = sy + 22
	nc = ORANGE_HI if net_ok else RED_ERR
	surf.blit(FONT_XS.render(
		f"UDP  {ROVER_IP}:{ROVER_PORT}   pkts:{packets_sent}",
		True, nc), (x0, ny))


def draw_drive_panel(
	surf,
	rect,
	x_val,
	z_val,
	x_i8,
	z_i8,
	left_cmd,
	right_cmd,
	left_vel,
	right_vel,
	net_ok,
	pkts,
):
	draw_panel(surf, rect, "BLDC DIFF DRIVE (WS TO ODRIVE)")
	x0, y0 = rect.x + 14, rect.y + 36
	bar_w = rect.w - 110

	def row(label, value, y):
		surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x0, y))
		draw_axis_bar(surf, x0 + 95, y, bar_w, 12, value, show_val=True)

	row("X CMD", x_val, y0)
	row("Z CMD", z_val, y0 + 18)
	row("LEFT", left_cmd / 127.0, y0 + 36)
	row("RIGHT", right_cmd / 127.0, y0 + 54)

	txt = FONT_XS.render(
		f"x_i8={x_i8:+4d}  z_i8={z_i8:+4d}  left={left_cmd:+4d}  right={right_cmd:+4d}",
		True,
		ORANGE_MID,
	)
	surf.blit(txt, (x0, y0 + 76))

	vel = FONT_XS.render(
		f"L={left_vel:+5.2f}  R={right_vel:+5.2f} turns/s   (VEL_MAX={VEL_MAX_TURNS_PER_S:.1f})",
		True,
		ORANGE_MID,
	)
	surf.blit(vel, (x0, y0 + 92))

	nc = ORANGE_HI if net_ok else RED_ERR
	surf.blit(
		FONT_XS.render(
			f"WS  ws://{ROVER_IP}:{DRIVE_WS_PORT}   fmt:[AA BB SEQ x z FF CRC]   pkts:{pkts}",
			True,
			nc,
		),
		(x0, y0 + 108),
	)


def fmt_value(value, fmt_spec):
	if value is None:
		return "--"
	try:
		return format(float(value), fmt_spec)
	except Exception:
		return "--"


def draw_odrive_panel(surf, rect, telemetry, telem_age_s):
	draw_panel(surf, rect, "ODRIVE TELEMETRY")

	odrv0 = telemetry.get("odrv0", {}) if isinstance(telemetry, dict) else {}
	odrv1 = telemetry.get("odrv1", {}) if isinstance(telemetry, dict) else {}

	vbus0 = odrv0.get("vbus_voltage") if isinstance(odrv0, dict) else None
	vbus1 = odrv1.get("vbus_voltage") if isinstance(odrv1, dict) else None

	fresh = telem_age_s is not None and telem_age_s <= 1.0
	status_color = ORANGE_HI if fresh else RED_ERR
	status_text = "LIVE" if fresh else "STALE/NO DATA"
	age_text = "--" if telem_age_s is None else f"{telem_age_s:.2f}s"

	x0 = rect.x + 12
	y0 = rect.y + 34
	line = FONT_XS.render(
		f"ODRV0 {fmt_value(vbus0, '.2f')}V    ODRV1 {fmt_value(vbus1, '.2f')}V",
		True,
		ORANGE_MID,
	)
	surf.blit(line, (x0, y0))
	surf.blit(FONT_XS.render(f"TELEM {status_text}  age:{age_text}", True, status_color),
			  (x0, y0 + 14))

	def get_axis(board, axis_name):
		if not isinstance(board, dict):
			return {}
		axes = board.get("axes", {})
		if not isinstance(axes, dict):
			return {}
		axis = axes.get(axis_name, {})
		return axis if isinstance(axis, dict) else {}

	motors = [
		("FL", get_axis(odrv0, "axis0")),
		("RL", get_axis(odrv0, "axis1")),
		("FR", get_axis(odrv1, "axis0")),
		("RR", get_axis(odrv1, "axis1")),
	]

	grid_x = rect.x + 12
	grid_y = rect.y + 60
	gap_x = 8
	gap_y = 8
	tile_w = (rect.w - 24 - gap_x) // 2
	tile_h = (rect.h - 70 - gap_y) // 2

	for i, (label, axis) in enumerate(motors):
		col = i % 2
		row = i // 2
		tx = grid_x + col * (tile_w + gap_x)
		ty = grid_y + row * (tile_h + gap_y)

		pygame.draw.rect(surf, (8, 8, 8), (tx, ty, tile_w, tile_h), border_radius=2)

		iq = axis.get("iq_measured") if isinstance(axis, dict) else None
		id_m = axis.get("id_measured") if isinstance(axis, dict) else None
		ibus = axis.get("ibus") if isinstance(axis, dict) else None
		pos = axis.get("pos_estimate") if isinstance(axis, dict) else None

		t1 = FONT_XS.render(
			f"{label} Iq {fmt_value(iq, '+.2f')} Id {fmt_value(id_m, '+.2f')}",
			True,
			ORANGE_HI,
		)
		t2 = FONT_XS.render(
			f"Ibus {fmt_value(ibus, '+.2f')}A  Pos {fmt_value(pos, '+.3f')}",
			True,
			ORANGE_MID,
		)
		surf.blit(t1, (tx + 6, ty + 5))
		surf.blit(t2, (tx + 6, ty + 18))


def main():
	global FONT_XS, FONT_SM, FONT_MD, FONT_LG

	pygame.init()

	screen = pygame.display.set_mode((BASE_W, BASE_H), pygame.RESIZABLE | pygame.DOUBLEBUF)
	pygame.display.set_caption("Extreme 3D Pro - Rover Arm + BLDC Drive")
	clock = pygame.time.Clock()

	canvas = pygame.Surface((BASE_W, BASE_H))

	def load_fonts():
		global FONT_XS, FONT_SM, FONT_MD, FONT_LG
		for name in ("couriernew", "courier new", "monospace"):
			try:
				FONT_XS = pygame.font.SysFont(name, 12, bold=True)
				FONT_SM = pygame.font.SysFont(name, 13, bold=True)
				FONT_MD = pygame.font.SysFont(name, 18, bold=True)
				FONT_LG = pygame.font.SysFont(name, 24, bold=True)
				break
			except Exception:
				continue

	load_fonts()

	pygame.joystick.init()
	pygame.event.set_grab(False)
	joy = None

	def try_connect():
		nonlocal joy
		if pygame.joystick.get_count() > 0:
			joy = pygame.joystick.Joystick(0)
			joy.init()

	try_connect()

	arm = ArmSender()
	last_cmd = {}
	servo_angle = 90
	motor_states = ["STOP"] * 5
	packets_sent = 0
	last_send = 0.0

	drive_ws = BldcDriveSender()
	drive_pkts_sent = 0
	last_drive_send = 0.0

	wheel_x = 0.0
	wheel_z = 0.0
	x_i8 = z_i8 = 0
	left_cmd = right_cmd = 0
	left_vel = right_vel = 0.0
	telemetry_age = None

	W, H = BASE_W, BASE_H
	is_minimized = False

	try:
		while True:
			now = time.monotonic()

			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()
				if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
					pygame.quit()
					sys.exit()
				if event.type == pygame.WINDOWMINIMIZED or event.type == pygame.WINDOWHIDDEN:
					is_minimized = True
				if event.type == pygame.WINDOWRESTORED or event.type == pygame.WINDOWSHOWN:
					is_minimized = False
				if event.type == pygame.JOYDEVICEADDED:
					try_connect()
				if event.type == pygame.JOYDEVICEREMOVED:
					joy = None
				if event.type == pygame.VIDEORESIZE:
					if event.w > 0 and event.h > 0:
						W, H = event.w, event.h
						screen = pygame.display.set_mode((W, H), pygame.RESIZABLE | pygame.DOUBLEBUF)
				if event.type == pygame.WINDOWMAXIMIZED:
					W, H = screen.get_size()

			W, H = screen.get_size()

			connected = joy is not None and pygame.joystick.get_count() > 0
			if connected:
				try:
					pygame.event.pump()
					num_axes = joy.get_numaxes()
					num_buttons = joy.get_numbuttons()
					num_hats = joy.get_numhats()
					axes = [joy.get_axis(i) for i in range(num_axes)]
					buttons = [joy.get_button(i) for i in range(num_buttons)]
					hats = [joy.get_hat(i) for i in range(num_hats)]
					ax_x = axes[0] if num_axes > 0 else 0.0
					ax_y = axes[1] if num_axes > 1 else 0.0
					ax_z = axes[2] if num_axes > 2 else 0.0
					ax_t = axes[3] if num_axes > 3 else -1.0
					hat_x, hat_y = hats[0] if num_hats > 0 else (0, 0)
					joy_name = joy.get_name()
				except Exception:
					connected = False

			if not connected:
				ax_x = ax_y = ax_z = 0.0
				ax_t = -1.0
				buttons = [False] * 16
				hat_x = hat_y = 0
				joy_name = ""

			# -- Arm commands ---------------------------------------------------
			if connected and (now - last_send) >= SEND_RATE:
				last_send = now

				def maybe_send(key, cmd, val=255):
					nonlocal packets_sent
					if last_cmd.get(key) != cmd:
						arm.send(cmd, val)
						last_cmd[key] = cmd
						packets_sent += 1

				if hat_y > 0:
					c1 = CMD_MOTOR1_FWD
					motor_states[0] = "FWD"
				elif hat_y < 0:
					c1 = CMD_MOTOR1_REV
					motor_states[0] = "REV"
				else:
					c1 = CMD_MOTOR1_STOP
					motor_states[0] = "STOP"
				maybe_send("m1", c1)

				if hat_x > 0:
					c2 = CMD_MOTOR2_FWD
					motor_states[1] = "FWD"
				elif hat_x < 0:
					c2 = CMD_MOTOR2_REV
					motor_states[1] = "REV"
				else:
					c2 = CMD_MOTOR2_STOP
					motor_states[1] = "STOP"
				maybe_send("m2", c2)

				if len(buttons) > 3:
					if buttons[2]:
						c3 = CMD_MOTOR3_FWD
						motor_states[2] = "FWD"
					elif buttons[3]:
						c3 = CMD_MOTOR3_REV
						motor_states[2] = "REV"
					else:
						c3 = CMD_MOTOR3_STOP
						motor_states[2] = "STOP"
					maybe_send("m3", c3)

				if len(buttons) > 5:
					if buttons[4]:
						c4 = CMD_MOTOR4A_FWD
						motor_states[3] = "FWD"
					elif buttons[5]:
						c4 = CMD_MOTOR4A_REV
						motor_states[3] = "REV"
					else:
						c4 = CMD_MOTOR4A_STOP
						motor_states[3] = "STOP"
					maybe_send("m4", c4)

				if len(buttons) > 7:
					if buttons[6]:
						c5 = CMD_MOTOR4B_FWD
						motor_states[4] = "FWD"
					elif buttons[7]:
						c5 = CMD_MOTOR4B_REV
						motor_states[4] = "REV"
					else:
						c5 = CMD_MOTOR4B_STOP
						motor_states[4] = "STOP"
					maybe_send("m5", c5)

				if len(buttons) > 9:
					if buttons[8]:
						servo_angle += SERVO_STEP
					if buttons[9]:
						servo_angle -= SERVO_STEP

				servo_angle = max(SERVO_MIN, min(SERVO_MAX, servo_angle))
				arm.send(CMD_SERVO_ANGLE, servo_angle)
				packets_sent += 1

			# -- BLDC diff-drive commands --------------------------------------
			if (now - last_drive_send) >= DRIVE_SEND_RATE:
				last_drive_send = now

				raw_x = ax_y
				raw_z = ax_z
				wheel_x = shape_drive_axis(raw_x, DEADZONE, DRIVE_SENSITIVITY_X, DRIVE_EXPO)
				wheel_z = shape_drive_axis(raw_z, DEADZONE, DRIVE_SENSITIVITY_Z, DRIVE_EXPO)

				sent = drive_ws.send_drive(wheel_x, wheel_z)
				if sent is not None:
					x_i8, z_i8, left_cmd, right_cmd, left_vel, right_vel = sent
				drive_pkts_sent += 1

			if is_minimized:
				clock.tick(60)
				continue

			# -- Draw on fixed canvas ------------------------------------------
			if drive_ws.last_telemetry_at > 0:
				telemetry_age = now - drive_ws.last_telemetry_at
			else:
				telemetry_age = None

			canvas.fill(BG)

			title = FONT_LG.render("EXTREME 3D PRO  //  ROVER ARM + BLDC DRIVE", True, ORANGE_HI)
			canvas.blit(title, (16, 12))
			draw_status_badge(canvas, connected, joy_name)
			draw_hline(canvas, 16, 44, BASE_W - 32)

			LP = pygame.Rect(14, 52, 440, 360)
			draw_panel(canvas, LP, "AXES & STICK")

			ay = LP.y + 40
			draw_axis_bar(canvas, LP.x + 14, ay, LP.w - 28, 14, ax_x, "X AXIS   L/R")
			draw_axis_bar(canvas, LP.x + 14, ay + 42, LP.w - 28, 14, ax_y, "Y AXIS   FWD/BACK")
			draw_axis_bar(canvas, LP.x + 14, ay + 84, LP.w - 28, 14, ax_z, "Z TWIST  RUDDER")

			draw_xy_pad(canvas, LP.x + 14, ay + 126, 140, ax_x, ax_y)
			draw_throttle_v(canvas, LP.x + 172, ay + 126, 34, 140, ax_t)

			raw_y = ay + 286
			rr = pygame.Rect(LP.x + 14, raw_y, LP.w - 28, 42)
			pygame.draw.rect(canvas, (6, 2, 0), rr, border_radius=2)
			pygame.draw.rect(canvas, ORANGE_DIM, rr, width=1, border_radius=2)
			canvas.blit(FONT_XS.render("RAW AXIS VALUES", True, ORANGE_MID),
						(rr.x + 8, rr.y + 4))
			for i, (n, v) in enumerate(zip(["AX0", "AX1", "AX2", "AX3"],
										   [ax_x, ax_y, ax_z, ax_t])):
				rx = rr.x + 10 + i * (rr.w // 4)
				canvas.blit(FONT_XS.render(n, True, ORANGE_DIM), (rx, rr.y + 17))
				canvas.blit(FONT_XS.render(f"{v:+.4f}", True, ORANGE_HI), (rx, rr.y + 28))

			RP = pygame.Rect(466, 52, 480, 360)
			draw_panel(canvas, RP, "BUTTONS")

			bw, bh = 98, 42
			bx0 = RP.x + 14
			by0 = RP.y + 40
			for i in range(min(12, len(BUTTON_LABELS))):
				col = i % 4
				row = i // 4
				bx = bx0 + col * (bw + 6)
				by = by0 + row * (bh + 8)
				pressed = buttons[i] if i < len(buttons) else False
				draw_button_widget(canvas, bx, by, bw, bh, pressed, BUTTON_LABELS[i])

			pov_cy = RP.y + 295
			draw_pov(canvas, RP.x + 110, pov_cy, 90, hat_x, hat_y)
			draw_trigger(canvas, RP.x + 340, pov_cy,
						 buttons[0] if len(buttons) > 0 else False)

			draw_hline(canvas, 14, 422, BASE_W - 28)

			CP = pygame.Rect(14, 428, 930, 112)
			arm_net_ok = not bool(arm.last_error)
			draw_rover_panel(canvas, CP, motor_states, servo_angle, arm_net_ok, packets_sent)

			draw_hline(canvas, 14, 548, BASE_W - 28)

			DP = pygame.Rect(14, 554, 530, 130)
			drive_net_ok = drive_ws.connected and not bool(drive_ws.last_error)
			draw_drive_panel(
				canvas,
				DP,
				wheel_x,
				wheel_z,
				x_i8,
				z_i8,
				left_cmd,
				right_cmd,
				left_vel,
				right_vel,
				drive_net_ok,
				drive_pkts_sent,
			)

			TP = pygame.Rect(552, 554, 392, 130)
			draw_odrive_panel(canvas, TP, drive_ws.telemetry, telemetry_age)

			if drive_ws.last_error:
				err = FONT_XS.render(drive_ws.last_error[:110], True, RED_ERR)
				canvas.blit(err, (18, BASE_H - 14))
			else:
				foot = FONT_XS.render(
					"ESC: quit   B9/B10: servo +/-   B3-B8: arm motors   HAT: motors 1/2",
					True,
					ORANGE_MID,
				)
				canvas.blit(foot, (BASE_W // 2 - foot.get_width() // 2, BASE_H - 14))

			if (W, H) != (BASE_W, BASE_H):
				scale = min(W / BASE_W, H / BASE_H)
				out_w = int(BASE_W * scale)
				out_h = int(BASE_H * scale)
				scaled = pygame.transform.smoothscale(canvas, (out_w, out_h))
				screen.fill(BG)
				ox = (W - out_w) // 2
				oy = (H - out_h) // 2
				screen.blit(scaled, (ox, oy))
			else:
				screen.blit(canvas, (0, 0))

			pygame.display.flip()
			clock.tick(60)

	finally:
		drive_ws.close()


if __name__ == "__main__":
	main()
