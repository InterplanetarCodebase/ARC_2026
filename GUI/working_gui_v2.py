#!/usr/bin/env python3
"""
ARC v2 Control GUI

Dark navy + orange themed control surface with three fixed tabs:
  - Arm & Wheel
  - Science
  - GPS

Each tab can be popped out into a separate window.
"""

import glob
import json
import os
import signal
import socket
import struct
import sys
import time
import array
import errno
import fcntl

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, GLib, Gtk

try:
	from websockets.sync.client import connect as ws_connect
	from websockets.exceptions import WebSocketException
	WEBSOCKETS_OK = True
except Exception:
	ws_connect = None
	WebSocketException = Exception
	WEBSOCKETS_OK = False


WINDOW_W = 1440
WINDOW_H = 900
MIN_W = 900
MIN_H = 560

# -- Network -----------------------------------------------------------------
ROVER_IP = "192.168.10.176"
ROVER_PORT = 5760

DRIVE_WS_PORT = 8765
DRIVE_SEND_RATE = 0.05
SEND_RATE = 0.05

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

ACK_BYTE = 0xAC
ACK_LEN = 4

VEL_MAX_TURNS_PER_S = 5.0

# Match BLDC GUI shaping so joystick movement feels consistent.
DEADZONE = 0.08
DRIVE_SENSITIVITY_X = 0.55
DRIVE_SENSITIVITY_Z = 0.50
DRIVE_EXPO = 1.8

# Higher refresh + light low-pass filtering for fluid indicators.
UI_TICK_MS = 16
AXIS_FILTER_ALPHA = 0.22
VALUE_FILTER_ALPHA = 0.16
SERVO_STEP_INTERVAL_S = 0.05


CSS = b"""
window          { background: #060a0f; }
#header         { background: #070d14; border-bottom: 1px solid #2a1800; }
#title          { font-family: "Courier New"; font-size: 16px; font-weight: bold;
				  color: #ff8800; letter-spacing: 3px; }
#subtitle       { font-family: "Courier New"; font-size: 10px; color: #ff8800;
				  letter-spacing: 2px; }
#active-badge   { font-family: "Courier New"; font-size: 11px; color: #ff8800;
				  background: #120800; border: 1px solid #3a1800;
				  border-radius: 4px; padding: 3px 10px; }
#statusbar      { background: #040608; border-top: 1px solid #1a0e00; padding: 0 12px; }
#status-lbl     { font-family: "Courier New"; font-size: 10px; color: #ff8800; }
#clock-lbl      { font-family: "Courier New"; font-size: 10px; color: #ff8800; }

notebook        { background: #060a0f; }
notebook header { background: #070d14; }
notebook tab    { background: #0a0e14; border: 1px solid #1a0e00;
				  padding: 5px 12px; font-family: "Courier New"; font-size: 11px;
				  color: #8a6020; min-height: 0; }
notebook tab:checked { background: #121820; border-bottom: 2px solid #ff8800;
					   color: #ff8800; }
notebook tab:hover   { background: #151a22; color: #cc7700; }

.action-btn     { font-family: "Courier New"; font-size: 11px; color: #ff8800;
				  background: transparent; border: 1px solid #3a1800;
				  border-radius: 4px; padding: 4px 10px; margin: 0 4px; }
.action-btn:hover { background: #1a0e00; border-color: #ff8800; }

.panel          { background: #0a111a; border: 1px solid #203040; border-radius: 6px; }
.panel-title    { font-family: "Courier New"; font-size: 12px; font-weight: bold;
				  color: #ff8800; letter-spacing: 1px; }
.label          { font-family: "Courier New"; font-size: 10px; color: #cc7700; }
.value          { font-family: "Courier New"; font-size: 11px; color: #ffb060; }

.ctrl-btn       { font-family: "Courier New"; font-size: 10px; color: #ff8800;
				  background: #0b141e; border: 1px solid #2a1800;
				  border-radius: 4px; padding: 8px 6px; }
.ctrl-btn:hover { border-color: #ff8800; background: #162130; }
.ctrl-btn:active { background: #221200; }
.ctrl-btn.active-press { background: #2a1500; border-color: #ffb060; color: #ffd39a; }

levelbar trough {
	min-height: 12px;
	border-radius: 3px;
	background: #091019;
	border: 1px solid #1b2c3d;
}
levelbar block.filled {
	border-radius: 2px;
	background: #ff8800;
}
"""


def style(widget, class_name):
	widget.get_style_context().add_class(class_name)
	return widget


def panel(title):
	frame = Gtk.Frame()
	style(frame, "panel")
	frame.set_shadow_type(Gtk.ShadowType.NONE)

	outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
	outer.set_margin_start(10)
	outer.set_margin_end(10)
	outer.set_margin_top(10)
	outer.set_margin_bottom(10)

	title_lbl = Gtk.Label(label=title)
	style(title_lbl, "panel-title")
	title_lbl.set_halign(Gtk.Align.START)
	outer.pack_start(title_lbl, False, False, 0)

	frame.add(outer)
	return frame, outer


def metric_row(name, value="--", width_chars=12):
	row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
	lbl = Gtk.Label(label=name)
	style(lbl, "label")
	lbl.set_halign(Gtk.Align.START)

	val = Gtk.Label(label=value)
	style(val, "value")
	val.set_width_chars(width_chars)
	val.set_xalign(1.0)

	row.pack_start(lbl, True, True, 0)
	row.pack_end(val, False, False, 0)
	return row, val


def identity_tab(text):
	root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
	root.set_hexpand(True)
	root.set_vexpand(True)

	container = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
	container.set_hexpand(True)
	container.set_vexpand(True)
	container.set_halign(Gtk.Align.CENTER)
	container.set_valign(Gtk.Align.CENTER)

	frame, inner = panel("TAB ID")
	frame.set_size_request(420, 180)

	entry = Gtk.Entry()
	entry.set_text(text)
	entry.set_editable(False)
	entry.set_can_focus(False)
	style(entry, "value")
	inner.pack_start(entry, False, False, 0)

	container.pack_start(frame, False, False, 0)
	root.pack_start(container, True, True, 0)
	return root


def clamp(v, lo, hi):
	return max(lo, min(hi, v))


def shape_drive_axis(value, deadzone, sensitivity, expo):
	"""Apply deadzone + exponential curve to reduce center sensitivity."""
	a = abs(value)
	if a <= deadzone:
		return 0.0
	sign = 1.0 if value >= 0.0 else -1.0
	norm = (a - deadzone) / (1.0 - deadzone)
	shaped = (norm ** expo) * sensitivity
	return sign * clamp(shaped, -1.0, 1.0)


def low_pass(prev, target, alpha):
	return prev + alpha * (target - prev)


def crc8(data):
	crc = 0
	for byte in data:
		crc ^= byte
		for _ in range(8):
			if crc & 0x80:
				crc = ((crc << 1) ^ 0x07) & 0xFF
			else:
				crc = (crc << 1) & 0xFF
	return crc


def clamp_i8(v):
	return max(-127, min(127, int(v)))


def fmt_value(value, fmt_spec):
	if value is None:
		return "--"
	try:
		return format(float(value), fmt_spec)
	except Exception:
		return "--"


class ArmSender:
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind(("0.0.0.0", 0))
		self.sock.setblocking(False)
		self.addr = (ROVER_IP, ROVER_PORT)
		self.local_port = self.sock.getsockname()[1]
		self.seq = 0
		self.last_error = ""
		self.last_ack_seq = None
		self.last_ack_status = None
		self.last_ack_at = 0.0
		self.last_encoder = {
			"enc1_raw": None,
			"enc2_raw": None,
			"enc1_v": None,
			"enc2_v": None,
		}
		self.last_encoder_at = 0.0

	def send(self, cmd, val=0):
		body = bytes([
			SOF1,
			SOF2,
			(self.seq >> 8) & 0xFF,
			self.seq & 0xFF,
			cmd,
			val & 0xFF,
		])
		pkt = body + bytes([crc8(body)])
		try:
			self.sock.sendto(pkt, self.addr)
			self.last_error = ""
		except OSError as e:
			self.last_error = str(e)
		self.seq = (self.seq + 1) & 0xFFFF

	def poll_feedback(self, max_packets=12):
		for _ in range(max_packets):
			try:
				data, _addr = self.sock.recvfrom(512)
			except BlockingIOError:
				break
			except OSError as e:
				if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
					break
				self.last_error = f"UDP recv failed: {e}"
				break

			if len(data) == ACK_LEN and data[0] == ACK_BYTE:
				self.last_ack_seq = (data[1] << 8) | data[2]
				self.last_ack_status = data[3]
				self.last_ack_at = time.monotonic()
				continue

			if data[:1] != b"{":
				continue

			try:
				payload = json.loads(data.decode("utf-8", errors="ignore"))
			except Exception:
				continue

			if not isinstance(payload, dict):
				continue

			if payload.get("type") not in ("encoder", "encoder_raw"):
				continue

			e1 = payload.get("enc1_raw")
			e2 = payload.get("enc2_raw")
			if e1 is not None:
				self.last_encoder["enc1_raw"] = int(e1)
			if e2 is not None:
				self.last_encoder["enc2_raw"] = int(e2)

			v1 = payload.get("enc1_v")
			v2 = payload.get("enc2_v")
			if v1 is None and self.last_encoder["enc1_raw"] is not None:
				v1 = (float(self.last_encoder["enc1_raw"]) * 3.3) / 4095.0
			if v2 is None and self.last_encoder["enc2_raw"] is not None:
				v2 = (float(self.last_encoder["enc2_raw"]) * 3.3) / 4095.0

			self.last_encoder["enc1_v"] = v1
			self.last_encoder["enc2_v"] = v2
			self.last_encoder_at = time.monotonic()


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

	def send_drive(self, x, z):
		if self.ws is None or not self.connected:
			self._try_connect()
		if self.ws is None or not self.connected:
			self.seq = (self.seq + 1) & 0xFFFF
			return None

		x_i8 = clamp_i8(x * 127.0)
		z_i8 = clamp_i8(z * 127.0)
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


class JoystickBridge:
	"""Reads Logitech Extreme 3D Pro values from Linux joystick device nodes."""

	JS_EVENT_BUTTON = 0x01
	JS_EVENT_AXIS = 0x02
	JS_EVENT_INIT = 0x80

	JSIOCGAXES = 0x80016A11
	JSIOCGBUTTONS = 0x80016A12

	@staticmethod
	def _jsiocgname(length):
		return 0x80006A13 + (length << 16)

	def __init__(self):
		self.connected = False
		self.name = ""
		self.last_error = ""
		self.axes = [0.0, 0.0, 0.0, -1.0]
		self.buttons = [0] * 12
		self.hat = (0, 0)
		self.servo_angle = 90
		self._dev = None
		self._dev_path = ""
		self._next_scan_at = 0.0
		self._last_servo_step_at = 0.0

	def axis(self, idx, default=0.0):
		if idx < len(self.axes):
			return float(self.axes[idx])
		return float(default)

	def button(self, idx):
		return bool(idx < len(self.buttons) and self.buttons[idx])

	def _disconnect(self, msg=""):
		self.connected = False
		self.name = ""
		if self._dev is not None:
			try:
				self._dev.close()
			except Exception:
				pass
		self._dev = None
		self._dev_path = ""
		if msg:
			self.last_error = msg

	def _connect_device(self, path):
		try:
			dev = open(path, "rb", buffering=0)
			os.set_blocking(dev.fileno(), False)

			axes_count = array.array("B", [0])
			btn_count = array.array("B", [0])
			fcntl.ioctl(dev, self.JSIOCGAXES, axes_count, True)
			fcntl.ioctl(dev, self.JSIOCGBUTTONS, btn_count, True)

			name_buf = array.array("B", [0] * 128)
			try:
				fcntl.ioctl(dev, self._jsiocgname(len(name_buf)), name_buf, True)
				name = name_buf.tobytes().split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
			except Exception:
				name = os.path.basename(path)

			self._dev = dev
			self._dev_path = path
			self.connected = True
			self.name = name or os.path.basename(path)
			self.axes = [0.0] * max(4, int(axes_count[0]))
			if len(self.axes) > 3:
				self.axes[3] = -1.0
			self.buttons = [0] * max(12, int(btn_count[0]))
			self.hat = (0, 0)
			self.last_error = ""
			return True
		except Exception as e:
			self._disconnect(f"joystick connect failed ({path}): {e}")
			return False

	def _connect_first(self):
		paths = sorted(glob.glob("/dev/input/js*"))
		if not paths:
			self._disconnect("no joystick device found in /dev/input/js*")
			return

		for path in paths:
			if self._connect_device(path):
				return

		if not self.connected and not self.last_error:
			self.last_error = "failed to open joystick device"

	def _ensure_connection(self, now):
		if self.connected and self._dev is not None:
			return
		if now < self._next_scan_at:
			return
		self._next_scan_at = now + 1.0
		self._connect_first()

	def poll(self):
		now = time.monotonic()
		self._ensure_connection(now)

		if self._dev is None or not self.connected:
			self.axes = [0.0, 0.0, 0.0, -1.0]
			self.buttons = [0] * 12
			self.hat = (0, 0)
			return

		try:
			while True:
				raw = self._dev.read(8)
				if not raw or len(raw) < 8:
					break

				_, value, ev_type, number = struct.unpack("IhBB", raw)
				ev_type = ev_type & ~self.JS_EVENT_INIT

				if ev_type == self.JS_EVENT_AXIS:
					while number >= len(self.axes):
						self.axes.append(0.0)
					norm = max(-1.0, min(1.0, value / 32767.0))
					self.axes[number] = norm
				elif ev_type == self.JS_EVENT_BUTTON:
					while number >= len(self.buttons):
						self.buttons.append(0)
					self.buttons[number] = 1 if value else 0

			# On Linux joystick driver, hat is often exposed as axes 4/5.
			hat_x = int(round(self.axis(4, 0.0))) if len(self.axes) > 4 else 0
			hat_y = int(round(-self.axis(5, 0.0))) if len(self.axes) > 5 else 0
			self.hat = (
				max(-1, min(1, hat_x)),
				max(-1, min(1, hat_y)),
			)

			now = time.monotonic()
			if now - self._last_servo_step_at >= SERVO_STEP_INTERVAL_S:
				if self.button(8):
					self.servo_angle = min(180, self.servo_angle + 2)
					self._last_servo_step_at = now
				elif self.button(9):
					self.servo_angle = max(0, self.servo_angle - 2)
					self._last_servo_step_at = now
		except BlockingIOError:
			pass
		except OSError as e:
			if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
				pass
			else:
				self._disconnect(f"joystick read failed: {e}")
		except Exception as e:
			self._disconnect(f"joystick read failed: {e}")


class ModuleView(Gtk.Box):
	"""Base 2-column layout: wheel/odrive left and module controls right."""

	def __init__(self, right_title, right_button_labels, right_level_labels):
		super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
		self.set_margin_start(10)
		self.set_margin_end(10)
		self.set_margin_top(10)
		self.set_margin_bottom(10)

		self.wheel_bars = {}
		self.odrive_values = {}
		self.right_bars = {}
		self.right_value_labels = {}
		self.right_buttons = {}
		self.wheel_nav_buttons = {}

		left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
		right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
		self._left_col = left
		self._right_col = right
		left.set_size_request(360, -1)
		right.set_size_request(420, -1)

		left.set_hexpand(True)
		right.set_hexpand(True)
		left.set_vexpand(True)
		right.set_vexpand(True)

		self._pane = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
		self._pane.set_wide_handle(True)
		self._pane.pack1(left, resize=True, shrink=True)
		self._pane.pack2(right, resize=True, shrink=True)
		self.pack_start(self._pane, True, True, 0)

		self._pane_initialized = False
		self.connect("size-allocate", self._on_size_allocate)

		left.pack_start(self._build_wheel_controls(), True, True, 0)
		left.pack_start(self._build_odrive_panel(), True, True, 0)
		right.pack_start(self._build_right_controls(right_title, right_button_labels, right_level_labels), True, True, 0)

	def _on_size_allocate(self, _widget, allocation):
		if self._pane_initialized:
			return
		if allocation.width > 0:
			self._pane.set_position(int(allocation.width * 0.42))
			self._pane_initialized = True

	def _build_wheel_controls(self):
		frame, root = panel("WHEEL CONTROLS")

		nav_grid = Gtk.Grid(column_spacing=8, row_spacing=8)
		nav_buttons = [
			(1, 0, "FWD"),
			(0, 1, "LEFT"),
			(1, 1, "STOP"),
			(2, 1, "RIGHT"),
			(1, 2, "REV"),
		]
		for c, r, txt in nav_buttons:
			b = Gtk.Button(label=txt)
			style(b, "ctrl-btn")
			nav_grid.attach(b, c, r, 1, 1)
			self.wheel_nav_buttons[txt] = b

		mode_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
		for txt in ("CRAWL", "NORMAL", "TURBO", "E-STOP"):
			b = Gtk.Button(label=txt)
			style(b, "ctrl-btn")
			mode_row.pack_start(b, True, True, 0)

		bars_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
		for label in ("Speed Cmd", "Turn Cmd", "Battery", "Motor Temp"):
			row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
			lbl = Gtk.Label(label=label)
			style(lbl, "label")
			lbl.set_width_chars(10)
			lbl.set_xalign(0.0)

			bar = Gtk.LevelBar()
			bar.set_min_value(0.0)
			bar.set_max_value(1.0)
			bar.set_value(0.0)
			bar.set_hexpand(True)

			val = Gtk.Label(label="0%")
			style(val, "value")
			val.set_width_chars(5)
			val.set_xalign(1.0)

			row.pack_start(lbl, False, False, 0)
			row.pack_start(bar, True, True, 0)
			row.pack_start(val, False, False, 0)
			bars_box.pack_start(row, False, False, 0)

			self.wheel_bars[label] = (bar, val)

		root.pack_start(nav_grid, False, False, 0)
		root.pack_start(mode_row, False, False, 0)
		root.pack_start(bars_box, False, False, 0)
		return frame

	def _build_odrive_panel(self):
		frame, root = panel("ODRIVE TELEMETRY")

		bus_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
		row0, self.odrive_values["Vbus ODrive0"] = metric_row("Vbus ODrive0", "-- V")
		row1, self.odrive_values["Vbus ODrive1"] = metric_row("Vbus ODrive1", "-- V")
		bus_row.pack_start(row0, True, True, 0)
		bus_row.pack_start(row1, True, True, 0)

		grid = Gtk.Grid(column_spacing=8, row_spacing=6)
		headers = ["Wheel", "Iq (A)", "Id (A)", "Ibus (A)", "Pos"]
		for col, name in enumerate(headers):
			h = Gtk.Label(label=name)
			style(h, "label")
			h.set_xalign(0.0)
			grid.attach(h, col, 0, 1, 1)

		wheel_rows = ["FL", "FR", "RL", "RR"]
		metrics = ["Iq", "Id", "Ibus", "Pos"]
		for i, wheel in enumerate(wheel_rows, start=1):
			wlbl = Gtk.Label(label=wheel)
			style(wlbl, "value")
			wlbl.set_xalign(0.0)
			grid.attach(wlbl, 0, i, 1, 1)

			for j, m in enumerate(metrics, start=1):
				v = Gtk.Label(label="--")
				style(v, "value")
				v.set_xalign(1.0)
				grid.attach(v, j, i, 1, 1)
				self.odrive_values[f"{wheel} {m}"] = v

		root.pack_start(bus_row, False, False, 0)
		root.pack_start(grid, True, True, 0)
		return frame

	def _build_right_controls(self, title, button_labels, level_labels):
		frame, root = panel(title)

		buttons_grid = Gtk.Grid(column_spacing=8, row_spacing=8)
		cols = 4
		for idx, name in enumerate(button_labels):
			c = idx % cols
			r = idx // cols
			b = Gtk.Button(label=name)
			style(b, "ctrl-btn")
			b.set_hexpand(True)
			buttons_grid.attach(b, c, r, 1, 1)
			self.right_buttons[name] = b

		levels = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
		for label in level_labels:
			row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
			lbl = Gtk.Label(label=label)
			style(lbl, "label")
			lbl.set_width_chars(16)
			lbl.set_xalign(0.0)

			bar = Gtk.LevelBar()
			bar.set_min_value(0.0)
			bar.set_max_value(1.0)
			bar.set_value(0.0)
			bar.set_hexpand(True)

			value = Gtk.Label(label="0%")
			style(value, "value")
			value.set_width_chars(5)
			value.set_xalign(1.0)

			row.pack_start(lbl, False, False, 0)
			row.pack_start(bar, True, True, 0)
			row.pack_start(value, False, False, 0)
			levels.pack_start(row, False, False, 0)

			self.right_bars[label] = bar
			self.right_value_labels[label] = value

		root.pack_start(buttons_grid, False, False, 0)
		root.pack_start(levels, True, True, 0)
		return frame

	def set_right_value(self, name, val):
		bar = self.right_bars.get(name)
		lbl = self.right_value_labels.get(name)
		if not bar or not lbl:
			return
		v = max(0.0, min(1.0, val))
		bar.set_value(v)
		lbl.set_text(f"{int(v * 100):d}%")

	def set_wheel_value(self, name, val):
		item = self.wheel_bars.get(name)
		if not item:
			return
		bar, lbl = item
		v = max(0.0, min(1.0, val))
		bar.set_value(v)
		lbl.set_text(f"{int(v * 100):d}%")

	def _set_button_active(self, button, pressed):
		ctx = button.get_style_context()
		if pressed:
			ctx.add_class("active-press")
		else:
			ctx.remove_class("active-press")

	def set_right_button_pressed(self, label, pressed):
		b = self.right_buttons.get(label)
		if b is None:
			return
		self._set_button_active(b, pressed)

	def set_wheel_nav_pressed(self, label, pressed):
		b = self.wheel_nav_buttons.get(label)
		if b is None:
			return
		self._set_button_active(b, pressed)


class ArmWheelView(ModuleView):
	def __init__(self):
		super().__init__(
			right_title="ARM CONTROLS",
			right_button_labels=[
				"ARM BTN 1", "ARM BTN 2", "ARM BTN 3", "ARM BTN 4",
				"ARM BTN 5", "ARM BTN 6", "ARM BTN 7", "ARM BTN 8",
				"ARM BTN 9", "ARM BTN 10", "ARM BTN 11", "ARM BTN 12",
			],
			right_level_labels=[
				"Joint 1", "Joint 2", "Joint 3", "Joint 4",
				"Joint 5", "Joint 6", "Grip Force", "Arm Load",
			],
		)
		self.arm_feedback_labels = {}
		self._build_arm_feedback_panel()

	def _build_arm_feedback_panel(self):
		frame, root = panel("ARM LINK / ENCODER FEEDBACK")

		for key, label in [
			("ack", "ACK"),
			("ack_age", "ACK Age"),
			("enc1_raw", "ENC1 Raw"),
			("enc1_v", "ENC1 V"),
			("enc2_raw", "ENC2 Raw"),
			("enc2_v", "ENC2 V"),
		]:
			row, val = metric_row(label, "--")
			root.pack_start(row, False, False, 0)
			self.arm_feedback_labels[key] = val

		self._right_col.pack_end(frame, False, False, 0)

	def update_arm_feedback(self, arm_sender):
		status_map = {
			0x00: "OK",
			0x01: "CRC_ERR",
			0x02: "UNK_CMD",
			0x03: "ESTOP",
		}

		if arm_sender.last_ack_seq is None:
			self.arm_feedback_labels["ack"].set_text("--")
			self.arm_feedback_labels["ack_age"].set_text("--")
		else:
			code = arm_sender.last_ack_status if arm_sender.last_ack_status is not None else -1
			status = status_map.get(code, f"0x{code:02X}" if code >= 0 else "--")
			self.arm_feedback_labels["ack"].set_text(f"{arm_sender.last_ack_seq} / {status}")
			age = time.monotonic() - arm_sender.last_ack_at if arm_sender.last_ack_at > 0 else None
			self.arm_feedback_labels["ack_age"].set_text(f"{age:.2f}s" if age is not None else "--")

		enc = arm_sender.last_encoder
		self.arm_feedback_labels["enc1_raw"].set_text(str(enc.get("enc1_raw", "--")))
		self.arm_feedback_labels["enc2_raw"].set_text(str(enc.get("enc2_raw", "--")))

		v1 = enc.get("enc1_v")
		v2 = enc.get("enc2_v")
		self.arm_feedback_labels["enc1_v"].set_text("--" if v1 is None else f"{float(v1):.3f} V")
		self.arm_feedback_labels["enc2_v"].set_text("--" if v2 is None else f"{float(v2):.3f} V")


class ScienceView(Gtk.Box):
	def __init__(self):
		super().__init__(orientation=Gtk.Orientation.VERTICAL)
		self.pack_start(identity_tab("This is science"), True, True, 0)


class GPSView(Gtk.Box):
	def __init__(self):
		super().__init__(orientation=Gtk.Orientation.VERTICAL)
		self.pack_start(identity_tab("This is GPS"), True, True, 0)


class PopOutWindow(Gtk.Window):
	def __init__(self, title, factory, on_close):
		super().__init__(title=f"POP OUT  -  {title}")
		self._on_close = on_close

		self.set_default_size(980, 720)
		self.set_size_request(720, 480)
		self.add(factory())
		self.connect("destroy", self._destroyed)
		self.show_all()

	def _destroyed(self, *_args):
		if callable(self._on_close):
			self._on_close(self)


class App(Gtk.Window):
	def __init__(self):
		super().__init__(title="ARC GUI V2")
		self.set_default_size(WINDOW_W, WINDOW_H)
		self.set_size_request(MIN_W, MIN_H)

		self._apply_css()
		self._popouts = []
		self.views = {}
		self.joystick = JoystickBridge()
		self.arm_sender = ArmSender()
		self.drive_sender = BldcDriveSender()
		self.last_cmd = {}
		self.motor_states = ["STOP"] * 5
		self.arm_packets_sent = 0
		self.drive_packets_sent = 0
		self.last_arm_send = 0.0
		self.last_drive_send = 0.0
		self.last_drive_preview = None
		self._last_tick_monotonic = time.monotonic()
		self._flt_speed = 0.5
		self._flt_turn = 0.5
		self._flt_battery = 0.0
		self._flt_motor_temp = 0.2
		self._flt_left_cmd = 0.0
		self._flt_right_cmd = 0.0
		self._flt_joints = {
			"Joint 1": 0.5,
			"Joint 2": 0.5,
			"Joint 3": 0.5,
			"Joint 4": 0.5,
			"Joint 5": 0.5,
			"Joint 6": 0.5,
			"Grip Force": 0.0,
			"Arm Load": 0.0,
		}

		root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
		self.add(root)

		root.pack_start(self._build_header(), False, False, 0)
		root.pack_start(self._build_tabs(), True, True, 0)
		root.pack_end(self._build_status_bar(), False, False, 0)

		self.connect("destroy", self._on_destroy)
		self.connect("key-press-event", self._on_key)
		self.show_all()

		GLib.timeout_add(UI_TICK_MS, self._tick)

	def _apply_css(self):
		provider = Gtk.CssProvider()
		provider.load_from_data(CSS)
		Gtk.StyleContext.add_provider_for_screen(
			Gdk.Screen.get_default(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
		)

	def _build_header(self):
		hdr = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
		hdr.set_name("header")
		hdr.set_size_request(-1, 56)

		left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
		left.set_margin_start(14)
		left.set_margin_top(6)
		left.set_margin_bottom(6)

		title = Gtk.Label(label="ARC CONTROL SURFACE")
		title.set_name("title")
		title.set_halign(Gtk.Align.START)
		subtitle = Gtk.Label(label="ARM / WHEEL / SCIENCE / GPS")
		subtitle.set_name("subtitle")
		subtitle.set_halign(Gtk.Align.START)

		left.pack_start(title, False, False, 0)
		left.pack_start(subtitle, False, False, 0)

		hdr.pack_start(left, False, False, 0)
		hdr.pack_start(Gtk.Box(), True, True, 0)

		self.active_badge = Gtk.Label(label="TAB: Arm & Wheel")
		self.active_badge.set_name("active-badge")
		self.active_badge.set_valign(Gtk.Align.CENTER)
		hdr.pack_start(self.active_badge, False, False, 0)

		self.popout_btn = Gtk.Button(label="Pop Out Current Tab")
		style(self.popout_btn, "action-btn")
		self.popout_btn.set_valign(Gtk.Align.CENTER)
		self.popout_btn.set_margin_end(8)
		self.popout_btn.connect("clicked", self._pop_out_current)
		hdr.pack_start(self.popout_btn, False, False, 0)

		return hdr

	def _build_tabs(self):
		self.notebook = Gtk.Notebook()
		self.notebook.set_scrollable(True)
		self.notebook.connect("switch-page", self._on_tab_switch)

		specs = [
			("Arm & Wheel", ArmWheelView),
			("Science", ScienceView),
			("GPS", GPSView),
		]
		self._view_factories = {name: factory for name, factory in specs}

		for name, factory in specs:
			view = factory()
			self.views[name] = view
			lbl = Gtk.Label(label=name)
			self.notebook.append_page(view, lbl)

		self.notebook.set_current_page(0)
		return self.notebook

	def _build_status_bar(self):
		sb = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
		sb.set_name("statusbar")
		sb.set_size_request(-1, 24)

		self.status_lbl = Gtk.Label(label="READY")
		self.status_lbl.set_name("status-lbl")
		self.status_lbl.set_halign(Gtk.Align.START)

		self.clock_lbl = Gtk.Label(label="")
		self.clock_lbl.set_name("clock-lbl")
		self.clock_lbl.set_halign(Gtk.Align.END)

		sb.pack_start(self.status_lbl, True, True, 0)
		sb.pack_end(self.clock_lbl, False, False, 0)
		return sb

	def _current_tab_name(self):
		idx = self.notebook.get_current_page()
		child = self.notebook.get_nth_page(idx)
		tab_lbl = self.notebook.get_tab_label(child)
		if isinstance(tab_lbl, Gtk.Label):
			return tab_lbl.get_text()
		return "Unknown"

	def _on_tab_switch(self, _notebook, _page, page_num):
		child = self.notebook.get_nth_page(page_num)
		tab_lbl = self.notebook.get_tab_label(child)
		txt = tab_lbl.get_text() if isinstance(tab_lbl, Gtk.Label) else "Unknown"
		self.active_badge.set_text(f"TAB: {txt}")

	def _pop_out_current(self, _btn):
		name = self._current_tab_name()
		factory = self._view_factories.get(name)
		if not factory:
			return

		win = PopOutWindow(name, factory, self._remove_popout)
		self._popouts.append(win)
		self.status_lbl.set_text(f"Popped out tab: {name}")

	def _remove_popout(self, win):
		if win in self._popouts:
			self._popouts.remove(win)

	def _tick(self):
		now_m = time.monotonic()
		self._last_tick_monotonic = now_m

		self.joystick.poll()
		self.arm_sender.poll_feedback()

		arm_view = self.views.get("Arm & Wheel")
		if isinstance(arm_view, ModuleView):
			raw_y = clamp(-self.joystick.axis(1, 0.0), -1.0, 1.0)
			raw_z = clamp(self.joystick.axis(2, 0.0), -1.0, 1.0)
			raw_t = clamp(self.joystick.axis(3, -1.0), -1.0, 1.0)

			shaped_y = shape_drive_axis(raw_y, DEADZONE, DRIVE_SENSITIVITY_X, DRIVE_EXPO)
			shaped_z = shape_drive_axis(raw_z, DEADZONE, DRIVE_SENSITIVITY_Z, DRIVE_EXPO)

			speed_target = (shaped_y + 1.0) / 2.0
			turn_target = (shaped_z + 1.0) / 2.0
			battery_target = 1.0 - ((raw_t + 1.0) / 2.0)
			activity = max(abs(shaped_y), abs(shaped_z))
			motor_temp_target = 0.2 + 0.75 * activity

			self._flt_speed = low_pass(self._flt_speed, speed_target, VALUE_FILTER_ALPHA)
			self._flt_turn = low_pass(self._flt_turn, turn_target, VALUE_FILTER_ALPHA)
			self._flt_battery = low_pass(self._flt_battery, battery_target, VALUE_FILTER_ALPHA)
			self._flt_motor_temp = low_pass(self._flt_motor_temp, motor_temp_target, VALUE_FILTER_ALPHA)

			arm_view.set_wheel_value("Speed Cmd", self._flt_speed)
			arm_view.set_wheel_value("Turn Cmd", self._flt_turn)
			arm_view.set_wheel_value("Battery", self._flt_battery)
			arm_view.set_wheel_value("Motor Temp", self._flt_motor_temp)

			left_cmd_target = clamp(shaped_y - shaped_z, -1.0, 1.0)
			right_cmd_target = clamp(shaped_y + shaped_z, -1.0, 1.0)
			self._flt_left_cmd = low_pass(self._flt_left_cmd, left_cmd_target, AXIS_FILTER_ALPHA)
			self._flt_right_cmd = low_pass(self._flt_right_cmd, right_cmd_target, AXIS_FILTER_ALPHA)

			hat_x, hat_y = self.joystick.hat

			# Arm UDP command stream (same command mapping structure as BLDC GUI).
			if self.joystick.connected and (now_m - self.last_arm_send) >= SEND_RATE:
				self.last_arm_send = now_m

				def maybe_send(key, cmd, val=255):
					if self.last_cmd.get(key) != cmd:
						self.arm_sender.send(cmd, val)
						self.last_cmd[key] = cmd
						self.arm_packets_sent += 1

				if hat_y > 0:
					c1 = CMD_MOTOR1_FWD
					self.motor_states[0] = "FWD"
				elif hat_y < 0:
					c1 = CMD_MOTOR1_REV
					self.motor_states[0] = "REV"
				else:
					c1 = CMD_MOTOR1_STOP
					self.motor_states[0] = "STOP"
				maybe_send("m1", c1)

				if hat_x > 0:
					c2 = CMD_MOTOR2_FWD
					self.motor_states[1] = "FWD"
				elif hat_x < 0:
					c2 = CMD_MOTOR2_REV
					self.motor_states[1] = "REV"
				else:
					c2 = CMD_MOTOR2_STOP
					self.motor_states[1] = "STOP"
				maybe_send("m2", c2)

				if self.joystick.button(2):
					c3 = CMD_MOTOR3_FWD
					self.motor_states[2] = "FWD"
				elif self.joystick.button(3):
					c3 = CMD_MOTOR3_REV
					self.motor_states[2] = "REV"
				else:
					c3 = CMD_MOTOR3_STOP
					self.motor_states[2] = "STOP"
				maybe_send("m3", c3)

				if self.joystick.button(4):
					c4 = CMD_MOTOR4A_FWD
					self.motor_states[3] = "FWD"
				elif self.joystick.button(5):
					c4 = CMD_MOTOR4A_REV
					self.motor_states[3] = "REV"
				else:
					c4 = CMD_MOTOR4A_STOP
					self.motor_states[3] = "STOP"
				maybe_send("m4", c4)

				if self.joystick.button(6):
					c5 = CMD_MOTOR4B_FWD
					self.motor_states[4] = "FWD"
				elif self.joystick.button(7):
					c5 = CMD_MOTOR4B_REV
					self.motor_states[4] = "REV"
				else:
					c5 = CMD_MOTOR4B_STOP
					self.motor_states[4] = "STOP"
				maybe_send("m5", c5)

				self.arm_sender.send(CMD_SERVO_ANGLE, self.joystick.servo_angle)
				self.arm_packets_sent += 1

			# BLDC WS drive stream + telemetry poll.
			if (now_m - self.last_drive_send) >= DRIVE_SEND_RATE:
				self.last_drive_send = now_m
				sent = self.drive_sender.send_drive(shaped_y, shaped_z)
				if sent is not None:
					self.last_drive_preview = sent
				self.drive_packets_sent += 1

			telemetry = self.drive_sender.telemetry if isinstance(self.drive_sender.telemetry, dict) else {}
			odrv0 = telemetry.get("odrv0", {}) if isinstance(telemetry, dict) else {}
			odrv1 = telemetry.get("odrv1", {}) if isinstance(telemetry, dict) else {}

			arm_view.odrive_values["Vbus ODrive0"].set_text(f"{fmt_value(odrv0.get('vbus_voltage'), '.2f')} V")
			arm_view.odrive_values["Vbus ODrive1"].set_text(f"{fmt_value(odrv1.get('vbus_voltage'), '.2f')} V")

			def get_axis(board, axis_name):
				if not isinstance(board, dict):
					return {}
				axes = board.get("axes", {})
				if not isinstance(axes, dict):
					return {}
				axis = axes.get(axis_name, {})
				return axis if isinstance(axis, dict) else {}

			axis_map = {
				"FL": get_axis(odrv0, "axis0"),
				"RL": get_axis(odrv0, "axis1"),
				"FR": get_axis(odrv1, "axis0"),
				"RR": get_axis(odrv1, "axis1"),
			}
			for wheel, axis in axis_map.items():
				arm_view.odrive_values[f"{wheel} Iq"].set_text(fmt_value(axis.get("iq_measured"), "+.2f"))
				arm_view.odrive_values[f"{wheel} Id"].set_text(fmt_value(axis.get("id_measured"), "+.2f"))
				arm_view.odrive_values[f"{wheel} Ibus"].set_text(fmt_value(axis.get("ibus"), "+.2f"))
				arm_view.odrive_values[f"{wheel} Pos"].set_text(fmt_value(axis.get("pos_estimate"), "+.3f"))

			def pair_to_unit(pos_idx, neg_idx):
				return 1.0 if self.joystick.button(pos_idx) else (0.0 if self.joystick.button(neg_idx) else 0.5)

			joint_targets = {
				"Joint 1": (hat_y + 1) / 2.0,
				"Joint 2": (hat_x + 1) / 2.0,
				"Joint 3": pair_to_unit(2, 3),
				"Joint 4": pair_to_unit(4, 5),
				"Joint 5": pair_to_unit(6, 7),
				"Joint 6": self.joystick.servo_angle / 180.0,
				"Grip Force": 1.0 if self.joystick.button(0) else 0.0,
			}
			for name, target in joint_targets.items():
				self._flt_joints[name] = low_pass(self._flt_joints[name], target, VALUE_FILTER_ALPHA)
				arm_view.set_right_value(name, self._flt_joints[name])

			# Highlight Arm control buttons mapped to joystick buttons.
			for i in range(12):
				arm_view.set_right_button_pressed(f"ARM BTN {i + 1}", self.joystick.button(i))

			# Highlight wheel nav buttons from shaped axes.
			thr = 0.15
			arm_view.set_wheel_nav_pressed("FWD", shaped_y > thr)
			arm_view.set_wheel_nav_pressed("REV", shaped_y < -thr)
			arm_view.set_wheel_nav_pressed("LEFT", shaped_z < -thr)
			arm_view.set_wheel_nav_pressed("RIGHT", shaped_z > thr)
			arm_view.set_wheel_nav_pressed("STOP", abs(shaped_y) <= thr and abs(shaped_z) <= thr)

			active_count = sum(
				1
				for b in (
					self.joystick.button(0),
					self.joystick.button(2),
					self.joystick.button(3),
					self.joystick.button(4),
					self.joystick.button(5),
					self.joystick.button(6),
					self.joystick.button(7),
					hat_x != 0,
					hat_y != 0,
				)
				if b
			)
			arm_load_target = min(1.0, active_count / 6.0)
			self._flt_joints["Arm Load"] = low_pass(self._flt_joints["Arm Load"], arm_load_target, VALUE_FILTER_ALPHA)
			arm_view.set_right_value("Arm Load", self._flt_joints["Arm Load"])

		if self.joystick.connected:
			ax0 = self.joystick.axis(0, 0.0)
			ax1 = self.joystick.axis(1, 0.0)
			ax2 = self.joystick.axis(2, 0.0)
			ax3 = self.joystick.axis(3, -1.0)
			ws_state = "UP" if self.drive_sender.connected else "DOWN"
			udp_state = "OK" if not self.arm_sender.last_error else "ERR"
			self.status_lbl.set_text(
				f"JOY CONNECTED (BG READY): {self.joystick.name}  AX[{ax0:+.2f} {ax1:+.2f} {ax2:+.2f} {ax3:+.2f}]  HAT{self.joystick.hat}  UDP:{udp_state}@{self.arm_sender.local_port} WS:{ws_state}"
			)
		else:
			if self.joystick.last_error:
				self.status_lbl.set_text(f"JOYSTICK ERROR: {self.joystick.last_error}")
			else:
				self.status_lbl.set_text("WAITING FOR JOYSTICK (connect Logitech Extreme 3D Pro)")

		if self.drive_sender.last_error and not self.drive_sender.connected:
			self.status_lbl.set_text(self.drive_sender.last_error[:180])

		if isinstance(arm_view, ArmWheelView):
			arm_view.update_arm_feedback(self.arm_sender)

		self.clock_lbl.set_text(time.strftime("%Y-%m-%d  %H:%M:%S"))
		return True

	def _on_key(self, _widget, event):
		if event.keyval == Gdk.KEY_Escape:
			self.destroy()
			return True
		return False

	def _on_destroy(self, _widget):
		for w in list(self._popouts):
			try:
				w.destroy()
			except Exception:
				pass
		try:
			self.drive_sender.close()
		except Exception:
			pass
		self.joystick._disconnect()
		Gtk.main_quit()


def signal_handler(_sig, _frame):
	Gtk.main_quit()
	sys.exit(0)


def main():
	signal.signal(signal.SIGINT, signal_handler)
	App()
	Gtk.main()


if __name__ == "__main__":
	main()
