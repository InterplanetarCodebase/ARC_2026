#!/usr/bin/env python3
"""
science_gui.py — ARC Science Board Control Surface
====================================================
Single-window dark navy + orange GUI for the science board ESP32, driven
by a Logitech Extreme 3D Pro joystick (with full mouse-button parity).

Hardware mapping
----------------
DC motors (no position feedback — visualization is simulated):
  • PLATFORM   → L298N A     up / down
  • BEAM       → L298N B     up / down
  • DRILL      → BTS 7960    collect (FWD) / release (REV), PWM 0-255

Servos (each 0°..180°):
  • STORAGE 1  Slide servo (Servo 1) — UNDER DRILL (0°) ↔ AT BASE (180°)
  • STORAGE 1  Lid servo   (Servo 2) — CLOSED (0°)      ↔ OPEN (180°)
  • STORAGE 2  Slide servo (Servo 3) — UNDER DRILL (0°) ↔ AT BASE (180°)
  • STORAGE 2  Lid servo   (Servo 4) — CLOSED (0°)      ↔ OPEN (180°)

Load cell (HX711) — read / tare with running history graph.

Joystick (Logitech Extreme 3D Pro)
----------------------------------
Stick Y          → Platform UP / DOWN     (push fwd = up)
Stick twist (Z)  → Beam     UP / DOWN     (twist right = up)
Throttle slider  → Drill PWM speed         (slider up = fast)
Trigger  (B0)    → Drill  COLLECT (hold)
Thumb    (B1)    → Drill  RELEASE (hold)
Top NW   (B2)    → Storage 1 — slide toggle
Top NE   (B3)    → Storage 1 — lid   toggle
Top SW   (B4)    → Storage 2 — slide toggle
Top SE   (B5)    → Storage 2 — lid   toggle
Base 1   (B6)    → Load cell READ
Base 2   (B7)    → Load cell TARE
Base 6   (B11)   → E-STOP

Every joystick action has an equivalent on-screen button (mouse usable).
Joystick polling reads /dev/input/js* directly so commands keep flowing
even when the window is not focused.
"""

import array
import errno
import fcntl
import glob
import json
import math
import os
import signal
import socket
import struct
import sys
import time

import gi
gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, GLib, Gtk

# ═══════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════
WINDOW_W = 1440
WINDOW_H = 880
MIN_W    = 1100
MIN_H    = 640

ROVER_IP     = "127.0.0.1"      # Change to Jetson IP on rover
SCI_UDP_PORT = 5761             # Must match science_receiver.py --udp-port

UI_TICK_MS       = 16
SEND_RATE_S      = 0.05         # max command repeat rate to receiver
LOAD_HISTORY     = 80           # samples kept in load-cell graph
BTS_DEFAULT_PWM  = 200

AXIS_THRESHOLD   = 0.30         # stick deflection required for FWD/REV
AXIS_DEADZONE    = 0.12

# ─── Joystick button indices (0-based) ──────────────────────────────
JS_BTN_DRILL_FWD     = 0   # trigger
JS_BTN_DRILL_REV     = 1   # thumb
JS_BTN_S1_SLIDE      = 2
JS_BTN_S1_LID        = 3
JS_BTN_S2_SLIDE      = 4
JS_BTN_S2_LID        = 5
JS_BTN_LOAD_READ     = 6
JS_BTN_LOAD_TARE     = 7
JS_BTN_ESTOP         = 11

JS_AX_PLATFORM       = 1   # stick Y
JS_AX_BEAM           = 2   # twist
JS_AX_THROTTLE       = 3   # throttle slider

# ═══════════════════════════════════════════════════════════════════
#  PROTOCOL  (mirrors Science/test_science_board.ino + science_receiver.py)
# ═══════════════════════════════════════════════════════════════════
SOF1       = 0xAA
SOF2       = 0xBB
ACK_BYTE   = 0xAC
ACK_LEN    = 4
PACKET_LEN = 7

CMD_PLATFORM_FWD  = 0x11   # L298N A FWD  → Platform UP
CMD_PLATFORM_REV  = 0x12   # L298N A REV  → Platform DOWN
CMD_PLATFORM_STOP = 0x13
CMD_BEAM_FWD      = 0x21   # L298N B FWD  → Beam UP
CMD_BEAM_REV      = 0x22   # L298N B REV  → Beam DOWN
CMD_BEAM_STOP     = 0x23
CMD_DRILL_FWD     = 0x31   # BTS FWD      → Drill COLLECT,  VAL = PWM
CMD_DRILL_REV     = 0x32   # BTS REV      → Drill RELEASE,  VAL = PWM
CMD_DRILL_STOP    = 0x33

CMD_S1_SLIDE      = 0x41   # Servo 1 angle
CMD_S1_LID        = 0x42   # Servo 2 angle
CMD_S2_SLIDE      = 0x43   # Servo 3 angle
CMD_S2_LID        = 0x44   # Servo 4 angle
CMD_SERVO_CENTER  = 0x45

CMD_LOADCELL_READ = 0x51
CMD_LOADCELL_TARE = 0x52
CMD_ESTOP         = 0xFF
CMD_HEARTBEAT     = 0x00

STATUS_NAMES = {0x00: "OK", 0x01: "CRC_ERR", 0x02: "UNK_CMD", 0x03: "ESTOP"}

# ═══════════════════════════════════════════════════════════════════
#  CSS — same dark navy + orange theme as working_gui_v2.py
# ═══════════════════════════════════════════════════════════════════
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

.panel          { background: #0a111a; border: 1px solid #203040; border-radius: 6px; }
.panel-title    { font-family: "Courier New"; font-size: 12px; font-weight: bold;
                  color: #ff8800; letter-spacing: 1px; }
.label          { font-family: "Courier New"; font-size: 10px; color: #cc7700; }
.value          { font-family: "Courier New"; font-size: 11px; color: #ffb060; }
.tele-label     { font-family: "Courier New"; font-size: 11px; color: #d68a34; }
.tele-value     { font-family: "Courier New"; font-size: 12px; color: #ffc07a; }
.big-value      { font-family: "Courier New"; font-size: 22px; font-weight: bold;
                  color: #ffd39a; }

.ctrl-btn       { font-family: "Courier New"; font-size: 11px; color: #ff8800;
                  background: #0b141e; border: 1px solid #2a1800;
                  border-radius: 4px; padding: 8px 14px; }
.ctrl-btn:hover { border-color: #ff8800; background: #162130; }
.ctrl-btn:active { background: #221200; }
.ctrl-btn.active-press { background: #2a1500; border-color: #ffb060; color: #ffd39a; }
.ctrl-btn.toggle-on    { background: #1f2c08; border-color: #b8e07a; color: #d8ff90; }

.estop-btn      { font-family: "Courier New"; font-size: 13px; font-weight: bold;
                  color: #ff4040; background: #1a0505; border: 2px solid #5a1010;
                  border-radius: 6px; padding: 10px 20px; }
.estop-btn:hover { background: #2a0808; border-color: #ff4040; }
.estop-btn.active-press { background: #4a0000; border-color: #ff8080; color: #ffffff; }

.strip          { background: #070d14; border-bottom: 1px solid #1a1000; }

scale trough    { background: #091019; border: 1px solid #1b2c3d; border-radius: 3px; }
scale slider    { background: #ff8800; min-width: 14px; min-height: 14px;
                  border-radius: 3px; }
scale highlight { background: #cc6600; border-radius: 3px; }
"""


# ═══════════════════════════════════════════════════════════════════
#  GENERIC HELPERS
# ═══════════════════════════════════════════════════════════════════
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
    lbl = Gtk.Label(label=title)
    style(lbl, "panel-title")
    lbl.set_halign(Gtk.Align.START)
    outer.pack_start(lbl, False, False, 0)
    frame.add(outer)
    return frame, outer


def kv_pair(key, value="--", val_chars=12):
    row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
    k = Gtk.Label(label=key + ":")
    style(k, "label")
    k.set_halign(Gtk.Align.START)
    v = Gtk.Label(label=value)
    style(v, "value")
    v.set_width_chars(val_chars)
    v.set_xalign(1.0)
    row.pack_start(k, True, True, 0)
    row.pack_end(v, False, False, 0)
    return row, v


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ═══════════════════════════════════════════════════════════════════
#  RING BUFFER (load cell history)
# ═══════════════════════════════════════════════════════════════════
class RingBuffer:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.data = []

    def append(self, v):
        self.data.append(float(v))
        if len(self.data) > self.maxlen:
            self.data = self.data[-self.maxlen:]

    def values(self):
        return list(self.data)


# ═══════════════════════════════════════════════════════════════════
#  UDP SENDER
# ═══════════════════════════════════════════════════════════════════
class ScienceSender:
    def __init__(self):
        self.addr = (ROVER_IP, SCI_UDP_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq = 0
        self.last_error = ""
        self.last_ack_seq = None
        self.last_ack_status = None
        self.last_ack_at = 0.0
        self.last_load = None
        self.last_load_at = 0.0

    def send(self, cmd, val=0):
        body = bytes([
            SOF1, SOF2,
            (self.seq >> 8) & 0xFF, self.seq & 0xFF,
            cmd & 0xFF, val & 0xFF,
        ])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as e:
            self.last_error = str(e)
        self.seq = (self.seq + 1) & 0xFFFF

    def poll_feedback(self, max_packets=16):
        for _ in range(max_packets):
            try:
                data, _ = self.sock.recvfrom(512)
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                self.last_error = f"recv: {e}"
                break

            if len(data) == ACK_LEN and data[0] == ACK_BYTE:
                self.last_ack_seq    = (data[1] << 8) | data[2]
                self.last_ack_status = data[3]
                self.last_ack_at     = time.monotonic()
                continue

            if data[:1] != b"{":
                continue
            try:
                payload = json.loads(data.decode("utf-8", errors="ignore"))
            except Exception:
                continue
            if isinstance(payload, dict) and payload.get("type") == "load_cell":
                raw = payload.get("raw")
                if raw is not None:
                    self.last_load = float(raw)
                    self.last_load_at = time.monotonic()


# ═══════════════════════════════════════════════════════════════════
#  JOYSTICK BRIDGE — Logitech Extreme 3D Pro
#  Reads /dev/input/jsX directly so it works regardless of window focus.
# ═══════════════════════════════════════════════════════════════════
class JoystickBridge:
    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS   = 0x02
    JS_EVENT_INIT   = 0x80
    JSIOCGAXES      = 0x80016A11
    JSIOCGBUTTONS   = 0x80016A12

    @staticmethod
    def _jsiocgname(length):
        return 0x80006A13 + (length << 16)

    def __init__(self):
        self.connected = False
        self.name = ""
        self.last_error = ""
        self.axes    = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0]
        self.buttons = [0] * 12
        self.hat = (0, 0)
        self._dev = None
        self._next_scan = 0.0

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
        if msg:
            self.last_error = msg

    def _connect_first(self):
        for path in sorted(glob.glob("/dev/input/js*")):
            try:
                dev = open(path, "rb", buffering=0)
                os.set_blocking(dev.fileno(), False)
                axes_buf = array.array("B", [0])
                btn_buf  = array.array("B", [0])
                fcntl.ioctl(dev, self.JSIOCGAXES,    axes_buf, True)
                fcntl.ioctl(dev, self.JSIOCGBUTTONS, btn_buf,  True)
                name_buf = array.array("B", [0] * 128)
                try:
                    fcntl.ioctl(dev, self._jsiocgname(128), name_buf, True)
                    name = name_buf.tobytes().split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
                except Exception:
                    name = os.path.basename(path)
                self._dev = dev
                self.connected = True
                self.name = name or os.path.basename(path)
                self.axes = [0.0] * max(6, int(axes_buf[0]))
                if len(self.axes) > 3:
                    self.axes[3] = -1.0
                self.buttons = [0] * max(12, int(btn_buf[0]))
                self.hat = (0, 0)
                self.last_error = ""
                return
            except Exception as e:
                self.last_error = f"joystick: {e}"

    def poll(self):
        now = time.monotonic()
        if not self.connected:
            if now >= self._next_scan:
                self._next_scan = now + 1.0
                self._connect_first()
            return
        try:
            while True:
                raw = self._dev.read(8)
                if not raw or len(raw) < 8:
                    break
                _, value, ev_type, number = struct.unpack("IhBB", raw)
                ev_type &= ~self.JS_EVENT_INIT
                if ev_type == self.JS_EVENT_AXIS:
                    while number >= len(self.axes):
                        self.axes.append(0.0)
                    self.axes[number] = max(-1.0, min(1.0, value / 32767.0))
                elif ev_type == self.JS_EVENT_BUTTON:
                    while number >= len(self.buttons):
                        self.buttons.append(0)
                    self.buttons[number] = 1 if value else 0
            hx = int(round(self.axis(4, 0.0))) if len(self.axes) > 4 else 0
            hy = int(round(-self.axis(5, 0.0))) if len(self.axes) > 5 else 0
            self.hat = (max(-1, min(1, hx)), max(-1, min(1, hy)))
        except BlockingIOError:
            pass
        except OSError as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self._disconnect(f"joystick read: {e}")


# ═══════════════════════════════════════════════════════════════════
#  HOLD-BUTTON — fires press_cb / release_cb on mouse-down/up
#  Used for DC-motor controls so click-and-hold drives the motor.
# ═══════════════════════════════════════════════════════════════════
class HoldButton(Gtk.Button):
    def __init__(self, label, on_change=None):
        super().__init__(label=label)
        style(self, "ctrl-btn")
        self._on_change = on_change
        self._mouse_pressed = False
        self.set_can_focus(False)
        self.connect("button-press-event",   self._press)
        self.connect("button-release-event", self._release)
        # Cancel hold if pointer leaves while held — otherwise we never see release.
        self.connect("leave-notify-event",   self._leave)

    def _press(self, _w, ev):
        if ev.button != 1:
            return False
        if not self._mouse_pressed:
            self._mouse_pressed = True
            if self._on_change:
                self._on_change(True)
        return False

    def _release(self, _w, ev):
        if ev.button != 1:
            return False
        if self._mouse_pressed:
            self._mouse_pressed = False
            if self._on_change:
                self._on_change(False)
        return False

    def _leave(self, _w, _ev):
        if self._mouse_pressed:
            self._mouse_pressed = False
            if self._on_change:
                self._on_change(False)

    def set_visual(self, active):
        ctx = self.get_style_context()
        if active:
            ctx.add_class("active-press")
        else:
            ctx.remove_class("active-press")


# ═══════════════════════════════════════════════════════════════════
#  BOARD STATE — shared simulation + telemetry, read by visualization
# ═══════════════════════════════════════════════════════════════════
class BoardState:
    def __init__(self):
        # DC motor commanded direction: -1 REV, 0 STOP, +1 FWD
        self.platform_dir = 0
        self.beam_dir     = 0
        self.drill_dir    = 0
        self.drill_pwm    = BTS_DEFAULT_PWM

        # Simulated positions (no real feedback)
        self.platform_y   = 0.50    # 0 = bottom, 1 = top
        self.beam_y       = 0.50
        self.drill_angle  = 0.0     # accumulating rotation

        # Servo target angles (the truth we send to ESP32)
        self.s1_slide = 0           # 0 = under drill, 180 = at base
        self.s1_lid   = 0           # 0 = closed, 180 = open
        self.s2_slide = 0
        self.s2_lid   = 0

        # Smoothed display angles for visualization
        self.s1_slide_disp = 0.0
        self.s1_lid_disp   = 0.0
        self.s2_slide_disp = 0.0
        self.s2_lid_disp   = 0.0

    def tick(self, dt):
        self.platform_y = clamp(self.platform_y + self.platform_dir * dt * 0.30, 0.0, 1.0)
        self.beam_y     = clamp(self.beam_y     + self.beam_dir     * dt * 0.30, 0.0, 1.0)
        if self.drill_dir != 0:
            self.drill_angle += self.drill_dir * dt * (self.drill_pwm / 255.0) * 9.0

        max_step = 240.0 * dt   # deg/s
        for attr in ("s1_slide", "s1_lid", "s2_slide", "s2_lid"):
            cur = getattr(self, attr + "_disp")
            tgt = getattr(self, attr)
            d   = clamp(tgt - cur, -max_step, max_step)
            setattr(self, attr + "_disp", cur + d)


# ═══════════════════════════════════════════════════════════════════
#  VISUALIZATION — side-view of the science assembly
# ═══════════════════════════════════════════════════════════════════
class BoardView(Gtk.DrawingArea):
    def __init__(self, state: BoardState):
        super().__init__()
        self._state = state
        self.set_size_request(420, 520)
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.connect("draw", self._draw)

    def _draw(self, _w, cr):
        s = self._state
        w = max(1, self.get_allocated_width())
        h = max(1, self.get_allocated_height())

        # Background
        cr.set_source_rgb(0.027, 0.045, 0.063)
        cr.rectangle(0, 0, w, h)
        cr.fill()

        # Faint grid
        cr.set_line_width(0.5)
        cr.set_source_rgba(0.15, 0.25, 0.35, 0.35)
        for x in range(0, w, 30):
            cr.move_to(x, 0); cr.line_to(x, h); cr.stroke()
        for y in range(0, h, 30):
            cr.move_to(0, y); cr.line_to(w, y); cr.stroke()

        # Geometry layout (relative)
        margin   = 30
        floor_y  = h - 60
        ceil_y   = 40
        rail_x_l = margin + 40
        rail_x_r = w - margin - 40

        # ── Side rails ──
        cr.set_source_rgb(0.30, 0.20, 0.10)
        cr.set_line_width(4)
        cr.move_to(rail_x_l, ceil_y); cr.line_to(rail_x_l, floor_y); cr.stroke()
        cr.move_to(rail_x_r, ceil_y); cr.line_to(rail_x_r, floor_y); cr.stroke()

        # ── Floor ──
        cr.set_source_rgb(0.40, 0.25, 0.10)
        cr.set_line_width(2)
        cr.move_to(margin, floor_y); cr.line_to(w - margin, floor_y); cr.stroke()

        # ── PLATFORM (slides up/down between rails) ──
        plat_w = rail_x_r - rail_x_l - 24
        plat_h = 12
        plat_y = floor_y - 60 - s.platform_y * (floor_y - ceil_y - 200)
        plat_x = rail_x_l + 12
        cr.set_source_rgb(1.00, 0.55, 0.10)
        cr.rectangle(plat_x, plat_y - plat_h / 2, plat_w, plat_h)
        cr.fill()
        cr.set_source_rgba(0.0, 0.0, 0.0, 0.25)
        cr.rectangle(plat_x, plat_y - plat_h / 2 + plat_h - 3, plat_w, 3)
        cr.fill()
        self._label(cr, plat_x + 4, plat_y - plat_h / 2 - 4, "PLATFORM", 9,
                    (1.0, 0.85, 0.55))

        # Platform direction arrow
        if s.platform_dir != 0:
            self._dir_arrow(cr, plat_x + plat_w + 10, plat_y, s.platform_dir,
                            (0.4, 1.0, 0.4))

        # ── BEAM (vertical, in middle, with drill carriage) ──
        beam_x = (rail_x_l + rail_x_r) / 2
        cr.set_source_rgb(0.55, 0.40, 0.20)
        cr.set_line_width(6)
        cr.move_to(beam_x, ceil_y + 10); cr.line_to(beam_x, plat_y - 10); cr.stroke()

        # Beam carriage moves along the beam — beam_y is height of carriage
        beam_top    = ceil_y + 30
        beam_bottom = plat_y - 30
        carriage_y  = beam_bottom - s.beam_y * (beam_bottom - beam_top)
        cr.set_source_rgb(1.00, 0.65, 0.20)
        cr.rectangle(beam_x - 14, carriage_y - 12, 28, 24)
        cr.fill()
        self._label(cr, beam_x + 18, carriage_y - 4, "BEAM", 9, (1.0, 0.85, 0.55))

        if s.beam_dir != 0:
            self._dir_arrow(cr, beam_x - 30, carriage_y, s.beam_dir,
                            (0.4, 1.0, 0.4))

        # ── DRILL — small box hanging from the carriage with rotating bit ──
        drill_y = carriage_y + 18
        cr.set_source_rgb(0.85, 0.85, 0.90)
        cr.rectangle(beam_x - 8, drill_y, 16, 22)
        cr.fill()

        # Drill bit (rotating)
        bit_top    = drill_y + 22
        bit_bottom = bit_top + 28
        cr.set_source_rgb(0.90, 0.90, 0.95)
        cr.set_line_width(3)
        cr.move_to(beam_x, bit_top); cr.line_to(beam_x, bit_bottom); cr.stroke()
        # Helix indicator: dots rotating around the bit
        for k in range(5):
            phase = s.drill_angle + k * 1.2
            ox = math.cos(phase) * 4
            oy = bit_top + 4 + k * 5
            cr.set_source_rgb(1.00, 0.55, 0.10)
            cr.arc(beam_x + ox, oy, 1.6, 0, 2 * math.pi)
            cr.fill()

        if s.drill_dir != 0:
            txt = "COLLECT" if s.drill_dir > 0 else "RELEASE"
            color = (0.4, 1.0, 0.4) if s.drill_dir > 0 else (1.0, 0.65, 0.40)
            self._label(cr, beam_x - 28, bit_bottom + 14, txt, 9, color)

        # ── STORAGES — two boxes on the floor either side of the beam ──
        st_w = 70
        st_h = 38
        st_y = floor_y - st_h
        st1_x = beam_x - 130 - st_w
        st2_x = beam_x + 130
        self._draw_storage(cr, st1_x, st_y, st_w, st_h, "STORAGE 1",
                           s.s1_slide_disp, s.s1_lid_disp, beam_x)
        self._draw_storage(cr, st2_x, st_y, st_w, st_h, "STORAGE 2",
                           s.s2_slide_disp, s.s2_lid_disp, beam_x)

        # ── Title bar ──
        cr.set_source_rgba(0.05, 0.05, 0.10, 0.85)
        cr.rectangle(0, 0, w, 22)
        cr.fill()
        cr.set_source_rgb(1.00, 0.55, 0.10)
        cr.select_font_face("Courier New", 0, 1)
        cr.set_font_size(11)
        cr.move_to(10, 15)
        cr.show_text("SCIENCE BOARD — LIVE")
        cr.set_source_rgba(1.00, 0.55, 0.10, 0.55)
        cr.set_font_size(9)
        cr.move_to(w - 110, 15)
        cr.show_text(f"DRILL PWM {s.drill_pwm:3d}")

    def _draw_storage(self, cr, x, y, w, h, name, slide, lid, drill_x):
        # Slide travel: 0° (under drill) → 180° (at base = original x).
        # Slide range: shift the box toward drill_x.
        base_cx = x + w / 2
        offset  = (1.0 - slide / 180.0) * (drill_x - base_cx)
        cx = base_cx + offset
        bx = cx - w / 2

        # Slide rail (under the boxes)
        cr.set_source_rgba(0.35, 0.25, 0.10, 0.6)
        cr.set_line_width(2)
        cr.move_to(min(x, x + offset) - 6, y + h + 4)
        cr.line_to(max(x + w, x + w + offset) + 6, y + h + 4)
        cr.stroke()

        # Box body
        cr.set_source_rgb(0.18, 0.22, 0.30)
        cr.rectangle(bx, y, w, h)
        cr.fill()
        cr.set_source_rgb(0.40, 0.55, 0.75)
        cr.set_line_width(1.5)
        cr.rectangle(bx, y, w, h)
        cr.stroke()

        # Lid hinged on left side, lid angle: 0 (closed/horizontal across top)
        # → 180 (folded back-left flat). We rotate from 0 to ~110°.
        lid_angle = (lid / 180.0) * (math.pi * 0.62)   # cap at ~110°
        cr.save()
        cr.translate(bx, y)
        cr.rotate(-lid_angle)
        cr.set_source_rgb(1.00, 0.55, 0.10)
        cr.rectangle(0, -4, w, 5)
        cr.fill()
        cr.restore()

        # Hinge dot
        cr.set_source_rgb(1.00, 0.85, 0.30)
        cr.arc(bx, y, 2.5, 0, 2 * math.pi)
        cr.fill()

        # Name + state
        self._label(cr, bx + 3, y + h + 16, name, 9, (1.0, 0.80, 0.45))
        slide_state = "UNDER DRILL" if slide < 90 else "AT BASE"
        lid_state   = "OPEN"        if lid   > 90 else "CLOSED"
        self._label(cr, bx + 3, y + h + 28,
                    f"{slide_state} | {lid_state}", 8, (0.85, 0.65, 0.35))

    def _dir_arrow(self, cr, x, y, direction, color):
        cr.set_source_rgb(*color)
        cr.set_line_width(2)
        size = 9
        if direction > 0:   # up arrow
            cr.move_to(x, y + size); cr.line_to(x, y - size); cr.stroke()
            cr.move_to(x - 5, y - size + 5); cr.line_to(x, y - size); cr.line_to(x + 5, y - size + 5); cr.stroke()
        else:               # down arrow
            cr.move_to(x, y - size); cr.line_to(x, y + size); cr.stroke()
            cr.move_to(x - 5, y + size - 5); cr.line_to(x, y + size); cr.line_to(x + 5, y + size - 5); cr.stroke()

    def _label(self, cr, x, y, text, size, color):
        cr.set_source_rgb(*color)
        cr.select_font_face("Courier New", 0, 0)
        cr.set_font_size(size)
        cr.move_to(x, y)
        cr.show_text(text)


# ═══════════════════════════════════════════════════════════════════
#  MOTORS PANEL — Platform / Beam / Drill (hold-to-move)
# ═══════════════════════════════════════════════════════════════════
class MotorsPanel(Gtk.Box):
    """3 DC motors. Each row has FWD / STOP / REV hold-buttons + state label.

    Keeps two flags per motor:  gui_pressed_fwd, gui_pressed_rev
    Joystick-driven flags are evaluated in tick(). Final commanded direction
    = joystick_dir if non-zero else gui_dir.
    """
    LABELS = {
        "platform": ("PLATFORM",        "UP",      "DOWN"),
        "beam":     ("BEAM",            "UP",      "DOWN"),
        "drill":    ("DRILL  (BTS7960)", "COLLECT", "RELEASE"),
    }
    CMDS = {
        "platform": (CMD_PLATFORM_FWD, CMD_PLATFORM_REV, CMD_PLATFORM_STOP),
        "beam":     (CMD_BEAM_FWD,     CMD_BEAM_REV,     CMD_BEAM_STOP),
        "drill":    (CMD_DRILL_FWD,    CMD_DRILL_REV,    CMD_DRILL_STOP),
    }

    def __init__(self, state: BoardState, sender: ScienceSender):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self._state  = state
        self._sender = sender

        # Per-motor: gui_dir (+1/0/-1), last sent dir, last send time
        self._gui_dir   = {k: 0 for k in self.LABELS}
        self._last_sent = {k: None for k in self.LABELS}
        self._last_send_t = {k: 0.0 for k in self.LABELS}

        self._fwd_btns  = {}
        self._rev_btns  = {}
        self._stop_btns = {}
        self._state_lbl = {}

        frame, box = panel("MOTORS  (hold to drive)")
        grid = Gtk.Grid(column_spacing=10, row_spacing=8)
        grid.set_hexpand(True)
        grid.set_column_homogeneous(False)

        for col, txt in enumerate(("Motor", "FWD", "STOP", "REV", "State")):
            h = Gtk.Label(label=txt)
            style(h, "label")
            h.set_xalign(0.5 if col else 0.0)
            h.set_hexpand(True)
            grid.attach(h, col, 0, 1, 1)

        for row, (key, (name, fwd_txt, rev_txt)) in enumerate(self.LABELS.items(), start=1):
            name_lbl = Gtk.Label(label=name)
            style(name_lbl, "tele-label")
            name_lbl.set_xalign(0.0)
            name_lbl.set_width_chars(16)
            grid.attach(name_lbl, 0, row, 1, 1)

            b_fwd = HoldButton(fwd_txt, lambda p, k=key: self._on_gui(k, +1, p))
            b_stp = Gtk.Button(label="STOP")
            style(b_stp, "ctrl-btn")
            b_stp.connect("clicked", lambda _w, k=key: self._on_stop(k))
            b_rev = HoldButton(rev_txt, lambda p, k=key: self._on_gui(k, -1, p))

            for col, b in enumerate((b_fwd, b_stp, b_rev), start=1):
                b.set_hexpand(True)
                grid.attach(b, col, row, 1, 1)

            st = Gtk.Label(label="STOP")
            style(st, "value")
            st.set_xalign(0.5)
            st.set_width_chars(8)
            grid.attach(st, 4, row, 1, 1)

            self._fwd_btns[key]  = b_fwd
            self._rev_btns[key]  = b_rev
            self._stop_btns[key] = b_stp
            self._state_lbl[key] = st

        box.pack_start(grid, False, False, 0)

        # Drill PWM slider
        sub_frame, sub_box = panel("DRILL  PWM  (BTS 7960)")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        lbl = Gtk.Label(label="PWM")
        style(lbl, "label")
        lbl.set_width_chars(5)
        self._pwm_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 255, 1)
        self._pwm_scale.set_value(BTS_DEFAULT_PWM)
        self._pwm_scale.set_draw_value(False)
        self._pwm_scale.set_hexpand(True)
        self._pwm_scale.connect("value-changed", self._on_pwm)
        self._pwm_lbl = Gtk.Label(label=str(BTS_DEFAULT_PWM))
        style(self._pwm_lbl, "value")
        self._pwm_lbl.set_width_chars(4)
        self._pwm_lbl.set_xalign(1.0)
        row.pack_start(lbl, False, False, 0)
        row.pack_start(self._pwm_scale, True, True, 0)
        row.pack_start(self._pwm_lbl, False, False, 0)
        sub_box.pack_start(row, False, False, 0)

        self.pack_start(frame,     False, False, 0)
        self.pack_start(sub_frame, False, False, 0)

    # ── GUI button callbacks ────────────────────────────────────────
    def _on_gui(self, key, direction, pressed):
        self._gui_dir[key] = direction if pressed else 0

    def _on_stop(self, key):
        self._gui_dir[key] = 0

    def _on_pwm(self, scale):
        self._state.drill_pwm = int(scale.get_value())
        self._pwm_lbl.set_text(str(self._state.drill_pwm))

    def set_pwm_silent(self, pwm):
        if pwm == int(self._pwm_scale.get_value()):
            return
        self._pwm_scale.handler_block_by_func(self._on_pwm)
        self._pwm_scale.set_value(pwm)
        self._pwm_scale.handler_unblock_by_func(self._on_pwm)
        self._state.drill_pwm = pwm
        self._pwm_lbl.set_text(str(pwm))

    # ── Per-tick: compute desired dir per motor and send if changed ─
    def tick(self, joystick: JoystickBridge):
        now = time.monotonic()

        # ── joystick → desired direction ────────────────────────────
        js = {"platform": 0, "beam": 0, "drill": 0}
        if joystick.connected:
            ay = -joystick.axis(JS_AX_PLATFORM, 0.0)   # push fwd (axis<0) = up
            if ay >  AXIS_THRESHOLD: js["platform"] = +1
            elif ay < -AXIS_THRESHOLD: js["platform"] = -1

            az = joystick.axis(JS_AX_BEAM, 0.0)         # twist right (>0) = up
            if az >  AXIS_THRESHOLD: js["beam"] = +1
            elif az < -AXIS_THRESHOLD: js["beam"] = -1

            if joystick.button(JS_BTN_DRILL_FWD): js["drill"] = +1
            elif joystick.button(JS_BTN_DRILL_REV): js["drill"] = -1

            # Throttle slider → drill PWM (top of slider = high speed)
            t = joystick.axis(JS_AX_THROTTLE, -1.0)
            new_pwm = int((1.0 - (t + 1.0) / 2.0) * 255)
            new_pwm = clamp(new_pwm, 0, 255)
            if abs(new_pwm - self._state.drill_pwm) >= 4:
                self.set_pwm_silent(new_pwm)

        # ── final direction = joystick wins, else GUI ───────────────
        for key in self.LABELS:
            d = js[key] if js[key] != 0 else self._gui_dir[key]
            self._update_motor(key, d, now, joystick_active=(js[key] != 0))

    def _update_motor(self, key, direction, now, joystick_active):
        fwd_cmd, rev_cmd, stop_cmd = self.CMDS[key]
        last = self._last_sent[key]

        # State changed → send immediately. Same state → throttle to SEND_RATE_S.
        if direction != last:
            self._send(key, direction)
        elif direction != 0 and (now - self._last_send_t[key]) >= SEND_RATE_S:
            self._send(key, direction)

        # Update visualization state
        if key == "platform":
            self._state.platform_dir = direction
        elif key == "beam":
            self._state.beam_dir = direction
        elif key == "drill":
            self._state.drill_dir = direction

        # Update visuals
        st = self._state_lbl[key]
        if direction > 0:
            st.set_text("FWD" if key != "drill" else "COLLECT")
        elif direction < 0:
            st.set_text("REV" if key != "drill" else "RELEASE")
        else:
            st.set_text("STOP")
        self._fwd_btns[key].set_visual(direction > 0)
        self._rev_btns[key].set_visual(direction < 0)
        # Mark joystick override visually too (slight active hint)
        if joystick_active and self._gui_dir[key] == 0:
            self._fwd_btns[key].set_visual(direction > 0)
            self._rev_btns[key].set_visual(direction < 0)

    def _send(self, key, direction):
        fwd_cmd, rev_cmd, stop_cmd = self.CMDS[key]
        if direction > 0:
            val = self._state.drill_pwm if key == "drill" else 0
            self._sender.send(fwd_cmd, val)
        elif direction < 0:
            val = self._state.drill_pwm if key == "drill" else 0
            self._sender.send(rev_cmd, val)
        else:
            self._sender.send(stop_cmd, 0)
        self._last_sent[key]   = direction
        self._last_send_t[key] = time.monotonic()

    def emergency_stop(self):
        for key in self.LABELS:
            self._gui_dir[key] = 0


# ═══════════════════════════════════════════════════════════════════
#  STORAGES PANEL — 2 storages × (slide servo, lid servo)
# ═══════════════════════════════════════════════════════════════════
class StoragesPanel(Gtk.Box):
    SERVOS = [
        # (storage_name, attr_slide, attr_lid, cmd_slide, cmd_lid)
        ("STORAGE 1", "s1_slide", "s1_lid", CMD_S1_SLIDE, CMD_S1_LID),
        ("STORAGE 2", "s2_slide", "s2_lid", CMD_S2_SLIDE, CMD_S2_LID),
    ]
    SLIDE_LABELS = ("UNDER DRILL", "AT BASE")
    LID_LABELS   = ("CLOSED", "OPEN")

    def __init__(self, state: BoardState, sender: ScienceSender):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self._state  = state
        self._sender = sender
        self._toggle_btns = {}    # (attr, kind) -> Gtk.Button

        frame, box = panel("STORAGES  (servos — toggle to switch position)")

        for name, a_slide, a_lid, c_slide, c_lid in self.SERVOS:
            hdr_frame, hdr_box = panel(name)

            # Slide row
            slide_row = self._build_servo_row(
                "Slide",  self.SLIDE_LABELS, a_slide, c_slide)
            lid_row   = self._build_servo_row(
                "Lid",    self.LID_LABELS,   a_lid,   c_lid)

            hdr_box.pack_start(slide_row, False, False, 0)
            hdr_box.pack_start(lid_row,   False, False, 0)
            box.pack_start(hdr_frame, False, False, 0)

        # Center-all
        center = Gtk.Button(label="Center all servos (90°)")
        style(center, "ctrl-btn")
        center.connect("clicked", self._on_center)
        box.pack_start(center, False, False, 4)

        self.pack_start(frame, True, True, 0)

    def _build_servo_row(self, kind, labels, attr, cmd):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)

        kind_lbl = Gtk.Label(label=kind)
        style(kind_lbl, "tele-label")
        kind_lbl.set_xalign(0.0)
        kind_lbl.set_width_chars(6)

        b0 = Gtk.Button(label=labels[0])
        b1 = Gtk.Button(label=labels[1])
        for b in (b0, b1):
            style(b, "ctrl-btn")
            b.set_hexpand(True)
        b0.connect("clicked", lambda _w: self._set_servo(attr, cmd, 0))
        b1.connect("clicked", lambda _w: self._set_servo(attr, cmd, 180))

        state_lbl = Gtk.Label(label=labels[0])
        style(state_lbl, "value")
        state_lbl.set_width_chars(13)
        state_lbl.set_xalign(0.5)

        row.pack_start(kind_lbl, False, False, 0)
        row.pack_start(b0, True, True, 0)
        row.pack_start(b1, True, True, 0)
        row.pack_start(state_lbl, False, False, 0)

        self._toggle_btns[(attr, "0")]   = b0
        self._toggle_btns[(attr, "180")] = b1
        self._toggle_btns[(attr, "lbl")] = state_lbl
        self._toggle_btns[(attr, "labels")] = labels
        # Mark initial selection visually
        b0.get_style_context().add_class("toggle-on")
        return row

    def _set_servo(self, attr, cmd, angle):
        if getattr(self._state, attr) == angle:
            return
        setattr(self._state, attr, angle)
        self._sender.send(cmd, angle)
        # Update toggle visuals + label
        labels = self._toggle_btns[(attr, "labels")]
        b0 = self._toggle_btns[(attr, "0")]
        b1 = self._toggle_btns[(attr, "180")]
        lbl = self._toggle_btns[(attr, "lbl")]
        on, off = (b1, b0) if angle == 180 else (b0, b1)
        on.get_style_context().add_class("toggle-on")
        off.get_style_context().remove_class("toggle-on")
        lbl.set_text(labels[1] if angle == 180 else labels[0])

    def toggle(self, attr, cmd):
        cur = getattr(self._state, attr)
        self._set_servo(attr, cmd, 180 if cur < 90 else 0)

    def _on_center(self, _w):
        self._sender.send(CMD_SERVO_CENTER, 0)
        for _name, a_slide, a_lid, _cs, _cl in self.SERVOS:
            for attr in (a_slide, a_lid):
                setattr(self._state, attr, 90)
                # Reflect in label & remove highlights
                self._toggle_btns[(attr, "lbl")].set_text("90°")
                self._toggle_btns[(attr, "0")].get_style_context().remove_class("toggle-on")
                self._toggle_btns[(attr, "180")].get_style_context().remove_class("toggle-on")


# ═══════════════════════════════════════════════════════════════════
#  LOAD CELL PANEL — read / tare + value + history graph
# ═══════════════════════════════════════════════════════════════════
class LoadCellPanel(Gtk.Box):
    def __init__(self, sender: ScienceSender):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self._sender   = sender
        self._history  = RingBuffer(LOAD_HISTORY)

        frame, box = panel("LOAD CELL")

        # Big reading + age
        big_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        self._big_lbl = Gtk.Label(label="--")
        style(self._big_lbl, "big-value")
        self._big_lbl.set_xalign(0.0)
        big_row.pack_start(self._big_lbl, True, True, 0)
        self._age_lbl = Gtk.Label(label="age --")
        style(self._age_lbl, "value")
        self._age_lbl.set_xalign(1.0)
        big_row.pack_end(self._age_lbl, False, False, 0)
        box.pack_start(big_row, False, False, 0)

        # Buttons
        btn_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self._read_btn = Gtk.Button(label="READ")
        self._tare_btn = Gtk.Button(label="TARE / ZERO")
        for b, cmd in ((self._read_btn, CMD_LOADCELL_READ),
                       (self._tare_btn, CMD_LOADCELL_TARE)):
            style(b, "ctrl-btn")
            b.set_hexpand(True)
            b.connect("clicked", lambda _w, c=cmd: self._sender.send(c, 0))
            btn_row.pack_start(b, True, True, 0)
        box.pack_start(btn_row, False, False, 0)

        # History graph
        self._graph = Gtk.DrawingArea()
        self._graph.set_size_request(-1, 160)
        self._graph.set_hexpand(True)
        self._graph.connect("draw", self._draw_graph)
        box.pack_start(self._graph, True, True, 0)

        # Recent strip
        self._recent_lbl = Gtk.Label(label="—")
        style(self._recent_lbl, "value")
        self._recent_lbl.set_halign(Gtk.Align.START)
        self._recent_lbl.set_line_wrap(True)
        box.pack_start(self._recent_lbl, False, False, 0)

        self.pack_start(frame, True, True, 0)
        self._last_seen_at = 0.0

    def request_read(self):
        self._sender.send(CMD_LOADCELL_READ, 0)

    def request_tare(self):
        self._sender.send(CMD_LOADCELL_TARE, 0)

    def tick(self):
        s = self._sender
        if s.last_load is not None and s.last_load_at != self._last_seen_at:
            self._last_seen_at = s.last_load_at
            self._history.append(s.last_load)
            self._big_lbl.set_text(f"{s.last_load:.3f}")
            self._graph.queue_draw()
            recent = self._history.values()[-8:]
            self._recent_lbl.set_text("  ".join(f"{v:.2f}" for v in reversed(recent)))
        if s.last_load_at:
            age = time.monotonic() - s.last_load_at
            self._age_lbl.set_text(f"age {age:4.1f}s")

    def _draw_graph(self, w, cr):
        ww = max(1, w.get_allocated_width())
        hh = max(1, w.get_allocated_height())
        cr.set_source_rgb(0.04, 0.07, 0.10)
        cr.rectangle(0, 0, ww, hh)
        cr.fill()

        vals = self._history.values()
        if len(vals) < 2:
            cr.set_source_rgba(0.5, 0.4, 0.2, 0.5)
            cr.select_font_face("Courier New", 0, 0)
            cr.set_font_size(11)
            cr.move_to(ww / 2 - 70, hh / 2)
            cr.show_text("No data yet — press READ")
            return

        lo, hi = min(vals), max(vals)
        span = hi - lo if hi != lo else 1.0
        pad = 16

        def to_xy(i, v):
            x = pad + (i / (len(vals) - 1)) * (ww - 2 * pad)
            y = hh - pad - ((v - lo) / span) * (hh - 2 * pad)
            return x, y

        # Grid
        cr.set_line_width(0.5)
        for frac in (0.25, 0.5, 0.75):
            y = pad + frac * (hh - 2 * pad)
            cr.set_source_rgba(0.1, 0.2, 0.3, 0.5)
            cr.move_to(pad, y); cr.line_to(ww - pad, y); cr.stroke()
            val = hi - frac * span
            cr.set_source_rgba(0.5, 0.4, 0.2, 0.7)
            cr.select_font_face("Courier New", 0, 0)
            cr.set_font_size(9)
            cr.move_to(2, y + 3); cr.show_text(f"{val:.2f}")

        # Fill
        cr.set_source_rgba(1.0, 0.53, 0.0, 0.18)
        x0, y0 = to_xy(0, vals[0])
        cr.move_to(x0, hh - pad); cr.line_to(x0, y0)
        for i, v in enumerate(vals[1:], start=1):
            cr.line_to(*to_xy(i, v))
        last_x, _ = to_xy(len(vals) - 1, vals[-1])
        cr.line_to(last_x, hh - pad); cr.close_path(); cr.fill()

        # Line
        cr.set_source_rgb(1.0, 0.53, 0.0)
        cr.set_line_width(1.8)
        cr.move_to(x0, y0)
        for i, v in enumerate(vals[1:], start=1):
            cr.line_to(*to_xy(i, v))
        cr.stroke()

        # Latest dot
        lx, ly = to_xy(len(vals) - 1, vals[-1])
        cr.set_source_rgb(1.0, 0.75, 0.25)
        cr.arc(lx, ly, 3.5, 0, 2 * math.pi); cr.fill()


# ═══════════════════════════════════════════════════════════════════
#  STATUS STRIP
# ═══════════════════════════════════════════════════════════════════
class StatusStrip(Gtk.Box):
    def __init__(self):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=22)
        self.set_name("strip")
        self.get_style_context().add_class("strip")
        self.set_margin_start(14); self.set_margin_end(14)
        self.set_margin_top(4);    self.set_margin_bottom(4)

        self._fields = {}
        for name in ("UDP", "Last ACK", "Load Cell", "Joystick"):
            self._fields[name] = self._add_field(name)

    def _add_field(self, name):
        box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        k = Gtk.Label(label=name + ":")
        style(k, "label")
        v = Gtk.Label(label="--")
        style(v, "tele-value")
        box.pack_start(k, False, False, 0)
        box.pack_start(v, False, False, 0)
        self.pack_start(box, False, False, 0)
        return v

    def update(self, sender: ScienceSender, joystick: JoystickBridge):
        now = time.monotonic()

        if sender.last_error:
            self._fields["UDP"].set_text(f"ERR {sender.last_error[:24]}")
        else:
            self._fields["UDP"].set_text(f"{ROVER_IP}:{SCI_UDP_PORT}  seq={sender.seq}")

        if sender.last_ack_at > 0:
            age = now - sender.last_ack_at
            sname = STATUS_NAMES.get(sender.last_ack_status, hex(sender.last_ack_status or 0))
            self._fields["Last ACK"].set_text(f"seq {sender.last_ack_seq}  {sname}  {age:.1f}s")
        else:
            self._fields["Last ACK"].set_text("waiting…")

        if sender.last_load is not None:
            age = now - sender.last_load_at
            self._fields["Load Cell"].set_text(f"{sender.last_load:.3f}  ({age:.1f}s)")
        else:
            self._fields["Load Cell"].set_text("--")

        if joystick.connected:
            self._fields["Joystick"].set_text(joystick.name[:36])
        else:
            self._fields["Joystick"].set_text(f"none — {joystick.last_error[:40]}")


# ═══════════════════════════════════════════════════════════════════
#  MAIN APP WINDOW
# ═══════════════════════════════════════════════════════════════════
class App(Gtk.Window):
    def __init__(self):
        super().__init__(title="ARC — Science Board")
        self.set_default_size(WINDOW_W, WINDOW_H)
        self.set_size_request(MIN_W, MIN_H)
        self._apply_css()

        self.state    = BoardState()
        self.sender   = ScienceSender()
        self.joystick = JoystickBridge()

        # ── Build root layout ────────────────────────────────────────
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(root)

        root.pack_start(self._build_header(), False, False, 0)

        self.strip = StatusStrip()
        root.pack_start(self.strip, False, False, 0)

        body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        body.set_margin_start(10); body.set_margin_end(10)
        body.set_margin_top(6);    body.set_margin_bottom(4)
        root.pack_start(body, True, True, 0)

        # Left column: visualization
        left_frame, left_box = panel("VISUALIZATION  (simulated — no encoders)")
        self.view = BoardView(self.state)
        left_box.pack_start(self.view, True, True, 0)
        body.pack_start(left_frame, True, True, 0)

        # Right column: motors / storages / load cell stacked
        right_col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        right_col.set_size_request(560, -1)
        self.motors_panel = MotorsPanel(self.state, self.sender)
        self.storages_panel = StoragesPanel(self.state, self.sender)
        self.loadcell_panel = LoadCellPanel(self.sender)
        right_col.pack_start(self.motors_panel,   False, False, 0)
        right_col.pack_start(self.storages_panel, False, False, 0)
        right_col.pack_start(self.loadcell_panel, True,  True,  0)
        body.pack_start(right_col, False, False, 0)

        # E-STOP bar
        estop_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        estop_row.set_margin_start(10); estop_row.set_margin_end(10)
        estop_row.set_margin_bottom(6)
        self.estop_btn = Gtk.Button(label="⚠   E-STOP   ALL   MOTORS   ⚠")
        style(self.estop_btn, "estop-btn")
        self.estop_btn.set_hexpand(True)
        self.estop_btn.connect("clicked", lambda _w: self._do_estop())
        estop_row.pack_start(self.estop_btn, True, True, 0)
        root.pack_start(estop_row, False, False, 0)

        root.pack_end(self._build_status_bar(), False, False, 0)

        # ── Joystick edge tracking ──────────────────────────────────
        self._prev_btns = [0] * 12

        # ── Window setup ────────────────────────────────────────────
        self.connect("destroy",         self._on_destroy)
        self.connect("key-press-event", self._on_key)
        self.show_all()

        self._last_tick = time.monotonic()
        GLib.timeout_add(UI_TICK_MS, self._tick)

    # ── CSS / chrome ────────────────────────────────────────────────
    def _apply_css(self):
        provider = Gtk.CssProvider()
        provider.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

    def _build_header(self):
        hdr = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        hdr.set_name("header")
        hdr.set_size_request(-1, 52)
        hdr.set_margin_end(14)
        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        left.set_margin_start(14); left.set_margin_top(6); left.set_margin_bottom(6)
        title    = Gtk.Label(label="ARC — SCIENCE BOARD")
        subtitle = Gtk.Label(label=f"ESP32 @ {ROVER_IP}:{SCI_UDP_PORT}  |  Logitech Extreme 3D Pro")
        title.set_name("title"); subtitle.set_name("subtitle")
        for w in (title, subtitle):
            w.set_halign(Gtk.Align.START)
            left.pack_start(w, False, False, 0)
        badge = Gtk.Label(label="MODE: Science")
        badge.set_name("active-badge")
        badge.set_valign(Gtk.Align.CENTER)
        hdr.pack_start(left, False, False, 0)
        hdr.pack_start(Gtk.Box(), True, True, 0)
        hdr.pack_start(badge, False, False, 0)
        return hdr

    def _build_status_bar(self):
        sb = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        sb.set_name("statusbar")
        sb.set_size_request(-1, 24)
        self.status_lbl = Gtk.Label(
            label="READY  —  joystick polls regardless of window focus"
        )
        self.status_lbl.set_name("status-lbl")
        self.status_lbl.set_halign(Gtk.Align.START)
        self.clock_lbl = Gtk.Label(label="")
        self.clock_lbl.set_name("clock-lbl")
        self.clock_lbl.set_halign(Gtk.Align.END)
        sb.pack_start(self.status_lbl, True, True, 0)
        sb.pack_end(self.clock_lbl, False, False, 0)
        return sb

    # ── E-STOP ──────────────────────────────────────────────────────
    def _do_estop(self):
        self.sender.send(CMD_ESTOP, 0)
        self.motors_panel.emergency_stop()
        # Visual flash
        self.estop_btn.get_style_context().add_class("active-press")
        GLib.timeout_add(200, lambda:
            (self.estop_btn.get_style_context().remove_class("active-press"), False)[1])

    # ── Joystick edge handlers (toggle-style buttons) ──────────────
    def _handle_joystick_edges(self):
        b = [self.joystick.button(i) for i in range(12)]
        rising = [int(b[i] and not self._prev_btns[i]) for i in range(12)]
        self._prev_btns = b

        if rising[JS_BTN_S1_SLIDE]:
            self.storages_panel.toggle("s1_slide", CMD_S1_SLIDE)
        if rising[JS_BTN_S1_LID]:
            self.storages_panel.toggle("s1_lid",   CMD_S1_LID)
        if rising[JS_BTN_S2_SLIDE]:
            self.storages_panel.toggle("s2_slide", CMD_S2_SLIDE)
        if rising[JS_BTN_S2_LID]:
            self.storages_panel.toggle("s2_lid",   CMD_S2_LID)
        if rising[JS_BTN_LOAD_READ]:
            self.loadcell_panel.request_read()
        if rising[JS_BTN_LOAD_TARE]:
            self.loadcell_panel.request_tare()
        if rising[JS_BTN_ESTOP]:
            self._do_estop()

    # ── Per-tick (~16ms) ────────────────────────────────────────────
    def _tick(self):
        now = time.monotonic()
        dt  = now - self._last_tick
        self._last_tick = now

        self.joystick.poll()
        self.sender.poll_feedback()
        self._handle_joystick_edges()

        self.motors_panel.tick(self.joystick)
        self.state.tick(dt)
        self.loadcell_panel.tick()
        self.view.queue_draw()

        self.strip.update(self.sender, self.joystick)
        self.clock_lbl.set_text(time.strftime("%Y-%m-%d  %H:%M:%S"))
        return True

    # ── Keyboard shortcuts (only when window is focused) ───────────
    def _on_key(self, _w, event):
        kv = event.keyval
        if kv == Gdk.KEY_Escape:
            self.destroy()
            return True
        if kv == Gdk.KEY_space:
            self._do_estop()
            return True
        return False

    def _on_destroy(self, _w):
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
