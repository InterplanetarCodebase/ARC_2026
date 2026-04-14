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
import math
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

VEL_MAX_TURNS_PER_S = 10.0

DEADZONE = 0.08
DRIVE_SENSITIVITY_X = 0.55
DRIVE_SENSITIVITY_Z = 0.50
DRIVE_EXPO = 1.8

UI_TICK_MS = 16
AXIS_FILTER_ALPHA = 0.22
VALUE_FILTER_ALPHA = 0.16
ARM_POSE_FILTER_ALPHA = 0.06
SERVO_STEP_INTERVAL_S = 0.05
WRIST_ROLL_PWM = 10

ARM_BUTTON_LABELS = [
	"North -> Elbow (Up)",
	"South -> Elbow (Down)",
	"East -> Base (Right)",
	"West -> Base (Left)",
	"Btn 6 -> Shoulder (Up)",
	"Btn 4 -> Shoulder (Down)",
	"Btn 5 -> Wrist_pitch (Up)",
	"Btn 3 -> Wrist_pitch (Down)",
	"Btn 8 -> Gripper (Open)",
	"Btn 7 -> Gripper (Close)",
	"Btn 10 -> Wrist_roll (right)",
	"Btn 9 -> wrist_roll (left)",
]

MOTOR_KEYS = ["FL", "RL", "FR", "RR"]
MOTOR_INDEX = {
	"FL": ("odrv0", "axis0"),
	"RL": ("odrv0", "axis1"),
	"FR": ("odrv1", "axis0"),
	"RR": ("odrv1", "axis1"),
}
VALUE_FIELDS = [
	("iq", "Iq (A)"),
	("id_m", "Id (A)"),
	("ibus", "Ibus (A)"),
	("fet_temp", "FET Temp (C)"),
	("axis_error", "Axis Error"),
	("motor_error", "Motor Error"),
	("encoder_error", "Encoder Error"),
	("controller_error", "Controller Error"),
	("pos", "Pos"),
]


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
.tele-label     { font-family: "Courier New"; font-size: 11px; color: #d68a34; }
.tele-value     { font-family: "Courier New"; font-size: 12px; color: #ffc07a; }

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


class RingBuffer:
	def __init__(self, maxlen):
		self.maxlen = int(maxlen)
		self.data = []

	def append(self, value):
		self.data.append(float(value))
		if len(self.data) > self.maxlen:
			self.data = self.data[-self.maxlen :]

	def values(self):
		return self.data


class WheelTelemetryView(Gtk.Box):
	def __init__(self):
		super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=8)
		self.set_hexpand(True)
		self.set_vexpand(True)

		self.wheel_circum_m = math.pi * 0.20
		self.track_width_m = 0.762
		self.right_sign = -1.0
		self.fwd_sign = -1.0
		self.prev_l = None
		self.prev_r = None
		self.prev_t = None
		self.x = 0.0
		self.y = 0.0
		self.yaw = 0.0
		self.path = [(0.0, 0.0)]

		self.t_buf = RingBuffer(1200)
		self.iq0_buf = RingBuffer(1200)
		self.iq1_buf = RingBuffer(1200)
		self.id0_buf = RingBuffer(1200)
		self.id1_buf = RingBuffer(1200)

		self.values = {field: {m: None for m in MOTOR_KEYS} for field, _name in VALUE_FIELDS}

		self.vbus0_lbl = None
		self.vbus1_lbl = None
		self.value_cells = {}
		self.map_area = None

		self._build_ui()

	def _build_ui(self):
		map_frame, map_box = panel("WHEEL ODOM 2D MAP")
		self.map_area = Gtk.DrawingArea()
		self.map_area.set_size_request(-1, 220)
		self.map_area.set_hexpand(True)
		self.map_area.connect("draw", self._draw_map)
		map_box.pack_start(self.map_area, True, True, 0)

		graph_frame, graph_box = panel("ODRIVE CURRENT GRAPHS")
		graphs_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
		self.iq_area = Gtk.DrawingArea()
		self.id_area = Gtk.DrawingArea()
		self.iq_area.set_size_request(-1, 170)
		self.id_area.set_size_request(-1, 170)
		self.iq_area.set_hexpand(True)
		self.id_area.set_hexpand(True)
		self.iq_area.connect("draw", self._draw_iq)
		self.id_area.connect("draw", self._draw_id)
		graphs_row.pack_start(self.iq_area, True, True, 0)
		graphs_row.pack_start(self.id_area, True, True, 0)
		graph_box.pack_start(graphs_row, True, True, 0)

		tele_frame, tele_box = panel("ODRIVE TELEMETRY VALUES")
		tele_frame.set_hexpand(True)
		tele_box.set_hexpand(True)
		bus_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
		bus_row.set_hexpand(True)
		_0row, self.vbus0_lbl = metric_row("Vbus ODrive0", "-- V")
		_1row, self.vbus1_lbl = metric_row("Vbus ODrive1", "-- V")
		_0row.set_hexpand(True)
		_1row.set_hexpand(True)
		style(self.vbus0_lbl, "tele-value")
		style(self.vbus1_lbl, "tele-value")
		bus_row.pack_start(_0row, True, True, 0)
		bus_row.pack_start(_1row, True, True, 0)

		grid = Gtk.Grid(column_spacing=10, row_spacing=4)
		grid.set_hexpand(True)
		grid.set_halign(Gtk.Align.FILL)
		grid.set_column_homogeneous(True)
		h0 = Gtk.Label(label="Metric")
		style(h0, "label")
		style(h0, "tele-label")
		h0.set_xalign(0.0)
		h0.set_hexpand(True)
		grid.attach(h0, 0, 0, 1, 1)
		for c, m in enumerate(MOTOR_KEYS, start=1):
			h = Gtk.Label(label=m)
			style(h, "label")
			style(h, "tele-label")
			h.set_xalign(1.0)
			h.set_hexpand(True)
			grid.attach(h, c, 0, 1, 1)

		for r, (field, name) in enumerate(VALUE_FIELDS, start=1):
			lbl = Gtk.Label(label=name)
			style(lbl, "label")
			style(lbl, "tele-label")
			lbl.set_xalign(0.0)
			lbl.set_hexpand(True)
			grid.attach(lbl, 0, r, 1, 1)
			self.value_cells[field] = {}
			for c, m in enumerate(MOTOR_KEYS, start=1):
				v = Gtk.Label(label="--")
				style(v, "value")
				style(v, "tele-value")
				v.set_xalign(1.0)
				v.set_hexpand(True)
				grid.attach(v, c, r, 1, 1)
				self.value_cells[field][m] = v

		tele_box.pack_start(bus_row, False, True, 0)
		tele_box.pack_start(grid, True, True, 0)

		self.pack_start(map_frame, False, False, 0)
		self.pack_start(graph_frame, False, False, 0)
		self.pack_start(tele_frame, True, True, 0)

	@staticmethod
	def _axis_obj(board, axis_name):
		axes = board.get("axes", {}) if isinstance(board, dict) else {}
		axis = axes.get(axis_name, {}) if isinstance(axes, dict) else {}
		return axis if isinstance(axis, dict) else {}

	def _set_color_hex(self, cr, hex_color):
		h = hex_color.lstrip("#")
		if len(h) != 6:
			cr.set_source_rgb(1.0, 1.0, 1.0)
			return
		r = int(h[0:2], 16) / 255.0
		g = int(h[2:4], 16) / 255.0
		b = int(h[4:6], 16) / 255.0
		cr.set_source_rgb(r, g, b)

	def _fmt_cell(self, field, value):
		if value is None:
			return "--"
		if "error" in field:
			return f"0x{int(value):08X}"
		return f"{float(value):.2f}"

	def _avg_pair(self, d, a, b):
		va = d.get(a)
		vb = d.get(b)
		vals = [v for v in (va, vb) if v is not None]
		if not vals:
			return None
		return sum(vals) / float(len(vals))

	def update_from_telemetry(self, telemetry, ts_monotonic):
		if not isinstance(telemetry, dict):
			return

		odrv0 = telemetry.get("odrv0", {}) if isinstance(telemetry, dict) else {}
		odrv1 = telemetry.get("odrv1", {}) if isinstance(telemetry, dict) else {}

		for m, (board_name, axis_name) in MOTOR_INDEX.items():
			board = odrv0 if board_name == "odrv0" else odrv1
			axis = self._axis_obj(board, axis_name)
			self.values["iq"][m] = axis.get("iq_measured")
			self.values["id_m"][m] = axis.get("id_measured")
			self.values["ibus"][m] = axis.get("ibus")
			self.values["fet_temp"][m] = axis.get("fet_thermistor_temp")
			self.values["axis_error"][m] = axis.get("axis_error")
			self.values["motor_error"][m] = axis.get("motor_error")
			self.values["encoder_error"][m] = axis.get("encoder_error")
			self.values["controller_error"][m] = axis.get("controller_error")
			self.values["pos"][m] = axis.get("pos_estimate")

		self.vbus0_lbl.set_text(f"{fmt_value(odrv0.get('vbus_voltage'), '.2f')} V")
		self.vbus1_lbl.set_text(f"{fmt_value(odrv1.get('vbus_voltage'), '.2f')} V")

		iq0 = self._avg_pair(self.values["iq"], "FL", "RL")
		iq1 = self._avg_pair(self.values["iq"], "FR", "RR")
		id0 = self._avg_pair(self.values["id_m"], "FL", "RL")
		id1 = self._avg_pair(self.values["id_m"], "FR", "RR")
		if iq0 is not None and iq1 is not None and id0 is not None and id1 is not None:
			self.t_buf.append(ts_monotonic)
			self.iq0_buf.append(iq0)
			self.iq1_buf.append(iq1)
			self.id0_buf.append(id0)
			self.id1_buf.append(id1)

		fl = self.values["pos"].get("FL")
		rl = self.values["pos"].get("RL")
		fr = self.values["pos"].get("FR")
		rr = self.values["pos"].get("RR")
		if all(v is not None for v in (fl, rl, fr, rr)):
			left = ((fl + rl) * 0.5) * self.fwd_sign
			right = ((fr + rr) * 0.5) * self.right_sign * self.fwd_sign
			if self.prev_l is None:
				self.prev_l, self.prev_r, self.prev_t = left, right, ts_monotonic
			else:
				dt = ts_monotonic - self.prev_t
				if dt > 0.0:
					dl = (left - self.prev_l) * self.wheel_circum_m
					dr = (right - self.prev_r) * self.wheel_circum_m
					dc = (dl + dr) * 0.5
					dyaw = (dr - dl) / self.track_width_m
					ym = self.yaw + 0.5 * dyaw
					self.x += dc * math.cos(ym)
					self.y += dc * math.sin(ym)
					self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
					self.path.append((self.x, self.y))
					if len(self.path) > 1200:
						self.path = self.path[-1200:]
				self.prev_l, self.prev_r, self.prev_t = left, right, ts_monotonic

		for field, _name in VALUE_FIELDS:
			for m in MOTOR_KEYS:
				self.value_cells[field][m].set_text(self._fmt_cell(field, self.values[field][m]))

		if self.map_area is not None:
			self.map_area.queue_draw()
		self.iq_area.queue_draw()
		self.id_area.queue_draw()

	def _draw_map(self, widget, cr):
		w = max(1, widget.get_allocated_width())
		h = max(1, widget.get_allocated_height())

		self._set_color_hex(cr, "#091019")
		cr.rectangle(0, 0, w, h)
		cr.fill()

		# Pure 2D top-down map: X right, Y up, origin fixed at map center.
		# Uniform scaling keeps geometry and heading angles visually correct.
		points = self.path if self.path else [(0.0, 0.0)]
		r_max = 1.0
		for px, py in points:
			r_max = max(r_max, math.hypot(px, py))
		r_max = max(r_max, math.hypot(self.x, self.y))

		fit_radius_px = max(20.0, (min(w, h) * 0.45))
		scale = fit_radius_px / r_max
		scale = clamp(scale, 12.0, 80.0)
		grid_step_px = max(18, int(scale))

		cx, cy = (w * 0.5), (h * 0.5)

		self._set_color_hex(cr, "#1b2c3d")
		for i in range(int(cx % grid_step_px), w, grid_step_px):
			cr.move_to(i, 0)
			cr.line_to(i, h)
		for j in range(int(cy % grid_step_px), h, grid_step_px):
			cr.move_to(0, j)
			cr.line_to(w, j)
		cr.set_line_width(0.6)
		cr.stroke()

		self._set_color_hex(cr, "#2a3f52")
		cr.set_line_width(1.0)
		cr.move_to(cx, 0)
		cr.line_to(cx, h)
		cr.move_to(0, cy)
		cr.line_to(w, cy)
		cr.stroke()

		def world_to_screen(px, py):
			sx = cx + (px * scale)
			sy = cy - (py * scale)
			return sx, sy

		self._set_color_hex(cr, "#ff8800")
		cr.set_line_width(2.0)
		sx0, sy0 = world_to_screen(points[0][0], points[0][1])
		cr.move_to(sx0, sy0)
		for px, py in points[1:]:
			sx, sy = world_to_screen(px, py)
			cr.line_to(sx, sy)
		cr.stroke()

		hx, hy = world_to_screen(self.x, self.y)
		arrow_len = 18.0
		ax = hx + arrow_len * math.cos(self.yaw)
		ay = hy - arrow_len * math.sin(self.yaw)
		self._set_color_hex(cr, "#00c8ff")
		cr.set_line_width(3.0)
		cr.move_to(hx, hy)
		cr.line_to(ax, ay)
		cr.stroke()

		# Arrowhead
		head_angle = math.atan2(ay - hy, ax - hx)
		head_size = 8.0
		for sign in (1, -1):
			wing_angle = head_angle + sign * (math.pi * 0.75)
			cr.move_to(ax, ay)
			cr.line_to(ax + head_size * math.cos(wing_angle), ay + head_size * math.sin(wing_angle))
		cr.stroke()

		# Position dot
		self._set_color_hex(cr, "#00c8ff")
		cr.arc(hx, hy, 4, 0, 2 * math.pi)
		cr.fill()

		# Origin marker
		self._set_color_hex(cr, "#7f8ea0")
		cr.arc(cx, cy, 3, 0, 2 * math.pi)
		cr.fill()

		# Coordinate label
		self._set_color_hex(cr, "#64748b")
		cr.move_to(8, h - 6)
		cr.show_text(f"2D map  scale={scale:.1f}px/m  x={self.x:.2f}m  y={self.y:.2f}m  yaw={math.degrees(self.yaw):.1f}°")

	def _draw_two_line_graph(self, widget, cr, title, y0_buf, y1_buf):
		w = max(1, widget.get_allocated_width())
		h = max(1, widget.get_allocated_height())
		self._set_color_hex(cr, "#091019")
		cr.rectangle(0, 0, w, h)
		cr.fill()

		ts = self.t_buf.values()
		y0 = y0_buf.values()
		y1 = y1_buf.values()
		n = min(len(ts), len(y0), len(y1))
		if n < 2:
			self._set_color_hex(cr, "#64748b")
			cr.move_to(10, 18)
			cr.show_text(title + "  (no data)")
			return

		ts = ts[-n:]
		y0 = y0[-n:]
		y1 = y1[-n:]
		t0 = ts[0]
		tn = max(1e-6, ts[-1] - t0)
		y_min = min(min(y0), min(y1))
		y_max = max(max(y0), max(y1))
		if abs(y_max - y_min) < 1e-6:
			y_max += 1.0
			y_min -= 1.0

		left = 32.0
		top = 16.0
		right = 10.0
		bottom = 16.0
		gw = max(1.0, w - left - right)
		gh = max(1.0, h - top - bottom)

		self._set_color_hex(cr, "#1b2c3d")
		cr.rectangle(left, top, gw, gh)
		cr.set_line_width(1.0)
		cr.stroke()

		def pt(t, y):
			x = left + ((t - t0) / tn) * gw
			yn = (y - y_min) / (y_max - y_min)
			ys = top + gh - yn * gh
			return x, ys

		self._set_color_hex(cr, "#ff9446")
		cr.set_line_width(1.8)
		x, y = pt(ts[0], y0[0])
		cr.move_to(x, y)
		for i in range(1, n):
			x, y = pt(ts[i], y0[i])
			cr.line_to(x, y)
		cr.stroke()

		self._set_color_hex(cr, "#00d2ff")
		cr.set_line_width(1.8)
		x, y = pt(ts[0], y1[0])
		cr.move_to(x, y)
		for i in range(1, n):
			x, y = pt(ts[i], y1[i])
			cr.line_to(x, y)
		cr.stroke()

		self._set_color_hex(cr, "#64748b")
		cr.move_to(8, 14)
		cr.show_text(title)

		# Y-axis range labels
		self._set_color_hex(cr, "#3a5068")
		cr.move_to(2, top + gh)
		cr.show_text(f"{y_min:.1f}")
		cr.move_to(2, top + 10)
		cr.show_text(f"{y_max:.1f}")

	def _draw_iq(self, widget, cr):
		self._draw_two_line_graph(widget, cr, "Iq (A)  [orange=L  blue=R]", self.iq0_buf, self.iq1_buf)

	def _draw_id(self, widget, cr):
		self._draw_two_line_graph(widget, cr, "Id (A)  [orange=L  blue=R]", self.id0_buf, self.id1_buf)


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
				# Remapped wrist-pitch control: Btn 5 up, Btn 3 down.
				if self.button(4):
					self.servo_angle = min(180, self.servo_angle + 2)
					self._last_servo_step_at = now
				elif self.button(2):
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
		self.wheel_segment = None
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

		left.pack_start(self._build_wheel_segment(), True, True, 0)
		right.pack_start(self._build_right_controls(right_title, right_button_labels, right_level_labels), True, True, 0)

	def _build_wheel_segment(self):
		frame, root = panel("WHEEL SEGMENT")
		self.wheel_segment = WheelTelemetryView()
		root.pack_start(self.wheel_segment, True, True, 0)
		return frame

	def update_wheel_telemetry(self, telemetry, ts_monotonic):
		if self.wheel_segment is not None:
			self.wheel_segment.update_from_telemetry(telemetry, ts_monotonic)

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
			right_button_labels=ARM_BUTTON_LABELS,
			right_level_labels=[
				"Base", "Shoulder", "Elbow", "Gripper",
				"Wrist Roll", "Wrist Pitch", "Grip Force", "Arm Load",
			],
		)
		self.arm_feedback_labels = {}
		self.encoder_value_labels = {}
		self.arm_joint_norm = {
			"Base": 0.5,
			"Shoulder": 0.5,
			"Elbow": 0.5,
			"Wrist Pitch": 0.5,
			"Wrist Roll": 0.5,
			"Gripper": 0.5,
		}
		self.arm_joint_display = dict(self.arm_joint_norm)
		self.arm_joint_active = {k: 0.0 for k in self.arm_joint_norm.keys()}
		self.arm_area = None
		self._build_arm_right_segment()

	def _clear_right_column(self):
		for child in self._right_col.get_children():
			self._right_col.remove(child)

	def _build_arm_right_segment(self):
		self._clear_right_column()

		top_frame, top_box = panel("ARM 2D SKELETON")
		top_frame.set_hexpand(True)
		self.arm_area = Gtk.DrawingArea()
		self.arm_area.set_size_request(-1, 250)
		self.arm_area.set_hexpand(True)
		self.arm_area.connect("draw", self._draw_arm_skeleton)
		top_box.pack_start(self.arm_area, True, True, 0)

		mid_frame, mid_box = panel("ARM MID SECTION")
		mid_frame.set_hexpand(True)
		mid_frame.set_vexpand(True)
		placeholder = Gtk.Label(label="Reserved")
		style(placeholder, "label")
		placeholder.set_halign(Gtk.Align.CENTER)
		placeholder.set_valign(Gtk.Align.CENTER)
		placeholder.set_hexpand(True)
		placeholder.set_vexpand(True)
		mid_box.pack_start(placeholder, True, True, 0)

		bot_frame, bot_box = panel("ARM ENCODER VALUES")
		bot_frame.set_hexpand(True)
		ack_row, self.arm_feedback_labels["ack"] = metric_row("ACK", "--", width_chars=16)
		age_row, self.arm_feedback_labels["ack_age"] = metric_row("ACK Age", "--", width_chars=10)
		bot_box.pack_start(ack_row, False, True, 0)
		bot_box.pack_start(age_row, False, True, 0)

		grid = Gtk.Grid(column_spacing=10, row_spacing=4)
		grid.set_hexpand(True)
		h_enc = Gtk.Label(label="Encoder")
		h_raw = Gtk.Label(label="Raw")
		h_v = Gtk.Label(label="Voltage")
		for h in (h_enc, h_raw, h_v):
			style(h, "label")
		grid.attach(h_enc, 0, 0, 1, 1)
		grid.attach(h_raw, 1, 0, 1, 1)
		grid.attach(h_v, 2, 0, 1, 1)

		for idx in range(1, 6):
			name = Gtk.Label(label=f"ENC{idx}")
			name.set_xalign(0.0)
			style(name, "label")
			raw_lbl = Gtk.Label(label="--")
			raw_lbl.set_xalign(1.0)
			style(raw_lbl, "value")
			v_lbl = Gtk.Label(label="--")
			v_lbl.set_xalign(1.0)
			style(v_lbl, "value")
			grid.attach(name, 0, idx, 1, 1)
			grid.attach(raw_lbl, 1, idx, 1, 1)
			grid.attach(v_lbl, 2, idx, 1, 1)
			self.encoder_value_labels[f"enc{idx}_raw"] = raw_lbl
			self.encoder_value_labels[f"enc{idx}_v"] = v_lbl

		bot_box.pack_start(grid, False, True, 0)

		self._right_col.pack_start(top_frame, False, True, 0)
		self._right_col.pack_start(mid_frame, True, True, 0)
		self._right_col.pack_start(bot_frame, False, True, 0)
		self._right_col.show_all()

	def update_arm_pose(self, joint_targets, active_flags):
		for name in self.arm_joint_norm.keys():
			if name in joint_targets:
				self.arm_joint_norm[name] = clamp(float(joint_targets[name]), 0.0, 1.0)
			self.arm_joint_display[name] = low_pass(
				self.arm_joint_display[name],
				self.arm_joint_norm[name],
				ARM_POSE_FILTER_ALPHA,
			)
			self.arm_joint_active[name] = 1.0 if active_flags.get(name, False) else 0.0
		if self.arm_area is not None:
			self.arm_area.queue_draw()

	def _draw_arm_skeleton(self, widget, cr):
		w = max(1, widget.get_allocated_width())
		h = max(1, widget.get_allocated_height())

		def set_hex(hex_color):
			hv = hex_color.lstrip("#")
			r = int(hv[0:2], 16) / 255.0
			g = int(hv[2:4], 16) / 255.0
			b = int(hv[4:6], 16) / 255.0
			cr.set_source_rgb(r, g, b)

		def mix(a, b, t):
			return a + ((b - a) * t)

		def joint_color(active):
			t = clamp(active, 0.0, 1.0)
			r = mix(0.70, 1.00, t)
			g = mix(0.45, 0.85, t)
			b = mix(0.12, 0.45, t)
			cr.set_source_rgb(r, g, b)

		set_hex("#091019")
		cr.rectangle(0, 0, w, h)
		cr.fill()

		# Ground line
		set_hex("#243648")
		cr.set_line_width(1.0)
		cr.move_to(12, h - 28)
		cr.line_to(w - 12, h - 28)
		cr.stroke()

		cx = w * 0.5
		cy = h - 42.0
		base_yaw = math.radians((self.arm_joint_display["Base"] - 0.5) * 90.0)
		shoulder = math.radians((self.arm_joint_display["Shoulder"] - 0.5) * 140.0)
		# Keep a slight neutral bend so the arm doesn't appear unnaturally straight.
		elbow = math.radians(18.0 + ((self.arm_joint_display["Elbow"] - 0.5) * 150.0))
		wrist_pitch = math.radians(-10.0 + ((self.arm_joint_display["Wrist Pitch"] - 0.5) * 110.0))
		wrist_roll = math.radians((self.arm_joint_display["Wrist Roll"] - 0.5) * 180.0)
		gripper = self.arm_joint_display["Gripper"]

		l1, l2, l3 = 58.0, 50.0, 34.0
		a1 = (-math.pi / 2.0) + shoulder
		a2 = a1 + elbow
		a3 = a2 + wrist_pitch

		p0 = (cx, cy)
		p1 = (p0[0] + (l1 * math.cos(a1)), p0[1] + (l1 * math.sin(a1)))
		p2 = (p1[0] + (l2 * math.cos(a2)), p1[1] + (l2 * math.sin(a2)))
		p3 = (p2[0] + (l3 * math.cos(a3)), p2[1] + (l3 * math.sin(a3)))

		# Base pedestal and yaw indicator
		set_hex("#2e465e")
		cr.rectangle(cx - 16, cy - 8, 32, 16)
		cr.fill()
		joint_color(self.arm_joint_active["Base"])
		cr.set_line_width(3.0)
		cr.arc(cx, cy, 12.0, 0, 2 * math.pi)
		cr.stroke()
		cr.set_line_width(2.0)
		cr.move_to(cx, cy)
		cr.line_to(cx + 16.0 * math.cos(base_yaw), cy - 16.0 * math.sin(base_yaw))
		cr.stroke()

		# Links
		set_hex("#6f7f90")
		cr.set_line_width(5.0)
		cr.move_to(p0[0], p0[1])
		cr.line_to(p1[0], p1[1])
		cr.line_to(p2[0], p2[1])
		cr.line_to(p3[0], p3[1])
		cr.stroke()

		# Joints
		for name, (jx, jy), rad in (
			("Shoulder", p0, 6.5),
			("Elbow", p1, 6.0),
			("Wrist Pitch", p2, 5.8),
		):
			joint_color(self.arm_joint_active[name])
			cr.arc(jx, jy, rad, 0, 2 * math.pi)
			cr.fill()

		# Wrist roll ring at tool point
		joint_color(self.arm_joint_active["Wrist Roll"])
		cr.set_line_width(2.6)
		cr.arc(p3[0], p3[1], 8.5, 0, 2 * math.pi)
		cr.stroke()
		cr.set_line_width(2.0)
		rx = p3[0] + 7.0 * math.cos(wrist_roll)
		ry = p3[1] + 7.0 * math.sin(wrist_roll)
		cr.move_to(p3[0], p3[1])
		cr.line_to(rx, ry)
		cr.stroke()

		# Gripper jaws
		open_px = 4.0 + (gripper * 8.0)
		gx, gy = p3
		joint_color(self.arm_joint_active["Gripper"])
		cr.set_line_width(2.4)
		cr.move_to(gx + 10.0, gy)
		cr.line_to(gx + 20.0, gy - open_px)
		cr.move_to(gx + 10.0, gy)
		cr.line_to(gx + 20.0, gy + open_px)
		cr.stroke()

		# Nomenclature labels
		set_hex("#c98a3a")
		cr.move_to(cx - 20, cy + 18)
		cr.show_text("Base")
		cr.move_to(p0[0] - 54, p0[1] - 10)
		cr.show_text("Shoulder")
		cr.move_to(p1[0] - 34, p1[1] - 10)
		cr.show_text("Elbow")
		cr.move_to(p2[0] - 34, p2[1] - 10)
		cr.show_text("Wrist Pitch")
		cr.move_to(p3[0] - 28, p3[1] - 14)
		cr.show_text("Wrist Roll")
		cr.move_to(p3[0] + 16, p3[1] + 4)
		cr.show_text("Gripper")

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
		for idx in range(1, 6):
			raw_key = f"enc{idx}_raw"
			v_key = f"enc{idx}_v"
			raw_val = enc.get(raw_key)
			v_val = enc.get(v_key)
			self.encoder_value_labels[raw_key].set_text("--" if raw_val is None else str(raw_val))
			self.encoder_value_labels[v_key].set_text("--" if v_val is None else f"{float(v_val):.3f} V")


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
			"Base": 0.5,
			"Shoulder": 0.5,
			"Elbow": 0.5,
			"Gripper": 0.5,
			"Wrist Roll": 0.5,
			"Wrist Pitch": 0.5,
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
			raw_y = clamp(self.joystick.axis(1, 0.0), -1.0, 1.0)
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

			# Arm UDP command stream
			if self.joystick.connected and (now_m - self.last_arm_send) >= SEND_RATE:
				self.last_arm_send = now_m

				def maybe_send(key, cmd, val=255):
					if self.last_cmd.get(key) != cmd:
						self.arm_sender.send(cmd, val)
						self.last_cmd[key] = cmd
						self.arm_packets_sent += 1

				if hat_x > 0:
					c1 = CMD_MOTOR1_REV
					self.motor_states[0] = "FWD"
				elif hat_x < 0:
					c1 = CMD_MOTOR1_FWD
					self.motor_states[0] = "REV"
				else:
					c1 = CMD_MOTOR1_STOP
					self.motor_states[0] = "STOP"
				maybe_send("m1", c1)

				if self.joystick.button(5):
					c2 = CMD_MOTOR2_REV
					self.motor_states[1] = "FWD"
				elif self.joystick.button(3):
					c2 = CMD_MOTOR2_FWD
					self.motor_states[1] = "REV"
				else:
					c2 = CMD_MOTOR2_STOP
					self.motor_states[1] = "STOP"
				maybe_send("m2", c2)

				if hat_y > 0:
					c3 = CMD_MOTOR3_FWD
					self.motor_states[2] = "FWD"
				elif hat_y < 0:
					c3 = CMD_MOTOR3_REV
					self.motor_states[2] = "REV"
				else:
					c3 = CMD_MOTOR3_STOP
					self.motor_states[2] = "STOP"
				maybe_send("m3", c3)

				if self.joystick.button(7):
					c4 = CMD_MOTOR4A_FWD
					self.motor_states[3] = "FWD"
				elif self.joystick.button(6):
					c4 = CMD_MOTOR4A_REV
					self.motor_states[3] = "REV"
				else:
					c4 = CMD_MOTOR4A_STOP
					self.motor_states[3] = "STOP"
				maybe_send("m4", c4)

				if self.joystick.button(9):
					c5 = CMD_MOTOR4B_FWD
					self.motor_states[4] = "FWD"
				elif self.joystick.button(8):
					c5 = CMD_MOTOR4B_REV
					self.motor_states[4] = "REV"
				else:
					c5 = CMD_MOTOR4B_STOP
					self.motor_states[4] = "STOP"
				pwm5 = WRIST_ROLL_PWM if c5 in (CMD_MOTOR4B_FWD, CMD_MOTOR4B_REV) else 255
				maybe_send("m5", c5, pwm5)

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
			arm_view.update_wheel_telemetry(telemetry, now_m)

			def pair_to_unit(pos_idx, neg_idx):
				return 1.0 if self.joystick.button(pos_idx) else (0.0 if self.joystick.button(neg_idx) else 0.5)

			joint_targets = {
				"Base": (hat_x + 1) / 2.0,
				"Shoulder": pair_to_unit(5, 3),
				"Elbow": (hat_y + 1) / 2.0,
				"Gripper": pair_to_unit(7, 6),
				"Wrist Roll": pair_to_unit(9, 8),
				"Wrist Pitch": self.joystick.servo_angle / 180.0,
				"Grip Force": 1.0 if self.joystick.button(0) else 0.0,
			}
			joint_active = {
				"Base": hat_x != 0,
				"Shoulder": self.joystick.button(5) or self.joystick.button(3),
				"Elbow": hat_y != 0,
				"Gripper": self.joystick.button(7) or self.joystick.button(6),
				"Wrist Roll": self.joystick.button(9) or self.joystick.button(8),
				"Wrist Pitch": self.joystick.button(4) or self.joystick.button(2),
			}
			for name, target in joint_targets.items():
				self._flt_joints[name] = low_pass(self._flt_joints[name], target, VALUE_FILTER_ALPHA)
				arm_view.set_right_value(name, self._flt_joints[name])

			if isinstance(arm_view, ArmWheelView):
				arm_pose = {
					"Base": self._flt_joints["Base"],
					"Shoulder": self._flt_joints["Shoulder"],
					"Elbow": self._flt_joints["Elbow"],
					"Wrist Pitch": self._flt_joints["Wrist Pitch"],
					"Wrist Roll": self._flt_joints["Wrist Roll"],
					"Gripper": self._flt_joints["Gripper"],
				}
				arm_view.update_arm_pose(arm_pose, joint_active)

			arm_view.set_right_button_pressed("North -> Elbow (Up)", hat_y > 0)
			arm_view.set_right_button_pressed("South -> Elbow (Down)", hat_y < 0)
			arm_view.set_right_button_pressed("East -> Base (Right)", hat_x > 0)
			arm_view.set_right_button_pressed("West -> Base (Left)", hat_x < 0)
			arm_view.set_right_button_pressed("Btn 6 -> Shoulder (Up)", self.joystick.button(5))
			arm_view.set_right_button_pressed("Btn 4 -> Shoulder (Down)", self.joystick.button(3))
			arm_view.set_right_button_pressed("Btn 5 -> Wrist_pitch (Up)", self.joystick.button(4))
			arm_view.set_right_button_pressed("Btn 3 -> Wrist_pitch (Down)", self.joystick.button(2))
			arm_view.set_right_button_pressed("Btn 8 -> Gripper (Open)", self.joystick.button(7))
			arm_view.set_right_button_pressed("Btn 7 -> Gripper (Close)", self.joystick.button(6))
			arm_view.set_right_button_pressed("Btn 10 -> Wrist_roll (right)", self.joystick.button(9))
			arm_view.set_right_button_pressed("Btn 9 -> wrist_roll (left)", self.joystick.button(8))

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
				f"JOY CONNECTED: {self.joystick.name}  AX[{ax0:+.2f} {ax1:+.2f} {ax2:+.2f} {ax3:+.2f}]  HAT{self.joystick.hat}  UDP:{udp_state}@{self.arm_sender.local_port} WS:{ws_state}"
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