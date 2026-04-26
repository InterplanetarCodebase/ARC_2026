#!/usr/bin/env python3
"""
Unified Rover Control GUI — tabbed interface for Arm, Misc, and Science.

Tabs:
  1) ARM      — arm motors, wrist servo, wheel drive, laser 1, laser 2
  2) MISC     — night LED, indicator LED matrix, 4 servos
  3) SCIENCE  — DC motors (platform/beam/drill), storage servos, load cell

Tab switching:
  - Joystick B12 (button index 11) cycles forward through tabs
  - Keyboard 1 / 2 / 3 jumps directly to a tab
  - Click the tab name in the top bar

Per-tab joystick mapping:
  ARM     — same as arm_with_laser.py, plus B11 (idx 10) toggles Laser 1
  MISC    — same as misc_gui.py except lasers (now on Arm tab)
  SCIENCE — same as science_gui.py except B12 (now reserved for tab cycling).
            Use the on-screen TARE button instead of B12.

The misc receiver heartbeat is kept alive on every tab so its watchdog
never trips while the operator is on another tab. Wheel drive only sends
while the Arm tab is active; switching away sends a final stop packet.

Run:  python3 GUI/unified_gui.py
"""

import json
import math
import os
import socket
import struct as _struct
import sys
import time
from collections import deque

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame

# ── Network ──────────────────────────────────────────────────────────────────
ROVER_IP = "192.168.10.177"
ARM_PORT = 5760
WHEEL_PORT = 5761
SCI_PORT = 5762
MISC_PORT = 5763

# Common framing
SOF1 = 0xAA
SOF2 = 0xBB
ACK_BYTE = 0xAC

# ── Arm board commands ──────────────────────────────────────────────────────
ARM_M1_FWD, ARM_M1_REV, ARM_M1_STOP = 0x11, 0x12, 0x13
ARM_M2_FWD, ARM_M2_REV, ARM_M2_STOP = 0x21, 0x22, 0x23
ARM_M3_FWD, ARM_M3_REV, ARM_M3_STOP = 0x31, 0x32, 0x33
ARM_M4A_FWD, ARM_M4A_REV, ARM_M4A_STOP = 0x41, 0x42, 0x43
ARM_M4B_FWD, ARM_M4B_REV, ARM_M4B_STOP = 0x51, 0x52, 0x53
ARM_SERVO_ANGLE = 0x60

MOTOR_NAMES = ["Base", "Shoulder", "Elbow", "Roller", "Gripper"]
MOTOR_DIR_LABELS = {
    "Base":     ("CW",       "CCW"),
    "Shoulder": ("Up",       "Down"),
    "Elbow":    ("Up",       "Down"),
    "Roller":   ("CW",       "CCW"),
    "Gripper":  ("Opening",  "Closing"),
}
SERVO_NAME = "Wrist_Servo"
SERVO_MIN, SERVO_MAX, SERVO_STEP = 0, 180, 2

# ── Misc board commands ─────────────────────────────────────────────────────
MISC_HEARTBEAT = 0x00
MISC_NIGHT_ON = 0x10
MISC_NIGHT_OFF = 0x11
MISC_IND_ON = 0x20
MISC_IND_OFF = 0x21
MISC_IND_PWM = 0x22
MISC_SERVO1 = 0x31
MISC_SERVO2 = 0x32
MISC_SERVO3 = 0x33
MISC_SERVO4 = 0x34
# Lasers were on the misc board command set originally; moved into the Arm
# tab UI but still routed to the misc receiver (which owns the laser pins).
MISC_LASER1_ON = 0x41
MISC_LASER1_OFF = 0x42
MISC_LASER2_ON = 0x43
MISC_LASER2_OFF = 0x44
MISC_ESTOP = 0xFF

LED_MODE_NAMES = {0: "OFF", 1: "RED", 2: "GREEN", 3: "YELLOW"}

# ── Science board commands ──────────────────────────────────────────────────
SCI_M1_FWD, SCI_M1_REV, SCI_M1_STOP = 0x11, 0x12, 0x13
SCI_M2_FWD, SCI_M2_REV, SCI_M2_STOP = 0x21, 0x22, 0x23
SCI_M3_FWD, SCI_M3_REV, SCI_M3_STOP = 0x31, 0x32, 0x33
SCI_S1, SCI_S2, SCI_S3, SCI_S4 = 0x41, 0x42, 0x43, 0x44
SCI_SERVO_CENTER = 0x45
SCI_LOAD_READ = 0x51
SCI_LOAD_TARE = 0x52
SCI_ESTOP = 0xFF

STATUS_NAMES = {0x00: "OK", 0x01: "CRC_ERR", 0x02: "UNK_CMD", 0x03: "ESTOP"}

# ── Wheel drive ──────────────────────────────────────────────────────────────
WHEEL_SOF1 = 0xAA
WHEEL_SOF2 = 0xBB
DEADZONE = 0.08
WHEEL_SEND_RATE = 0.05

# ── Timing ───────────────────────────────────────────────────────────────────
SEND_RATE = 0.05
SEND_GAP_S = 0.04
HEARTBEAT_INTERVAL = 0.4

# ── Theme ────────────────────────────────────────────────────────────────────
BG = (0, 0, 0)
ORANGE_HI = (255, 140, 0)
ORANGE_MID = (200, 90, 0)
ORANGE_DIM = (150, 75, 0)
ORANGE_GLOW = (255, 180, 60)
BORDER_LINE = (120, 55, 0)
RED_ERR = (255, 50, 20)
TAB_BG_INACTIVE = (8, 8, 8)
TAB_BG_ACTIVE = (40, 18, 0)

BASE_W, BASE_H = 1180, 820
TAB_BAR_H = 38
TAB_AREA_Y = 50  # below title


# ── CRC ──────────────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ── Senders ──────────────────────────────────────────────────────────────────
class ArmSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ROVER_IP, ARM_PORT)
        self.seq = 0
        self.last_error = ""

    def send(self, cmd, val=0):
        body = bytes([SOF1, SOF2, (self.seq >> 8) & 0xFF, self.seq & 0xFF,
                      cmd, val & 0xFF])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as e:
            self.last_error = str(e)
        self.seq = (self.seq + 1) & 0xFFFF
        return pkt


class WheelSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ROVER_IP, WHEEL_PORT)
        self.seq = 0
        self.last_error = ""

    def send_drive(self, x, z, throttle_pct):
        x_scaled = max(-1.0, min(1.0, x * throttle_pct))
        z_scaled = max(-1.0, min(1.0, z * throttle_pct))
        x_i8 = int(max(-127.0, min(127.0, x_scaled * 127.0)))
        z_i8 = int(max(-127.0, min(127.0, z_scaled * 127.0)))
        x_byte = _struct.pack('b', x_i8)[0]
        z_byte = _struct.pack('b', z_i8)[0]
        body = bytes([WHEEL_SOF1, WHEEL_SOF2,
                      (self.seq >> 8) & 0xFF, self.seq & 0xFF,
                      x_byte, z_byte, 0xFF])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as e:
            self.last_error = str(e)
        self.seq = (self.seq + 1) & 0xFFFF
        return pkt


class MiscSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)
        self.addr = (ROVER_IP, MISC_PORT)
        self.seq = 0
        self.last_error = ""
        self.last_ack_status = "-"
        self.last_ack_seq = -1
        self.last_ack_time = 0.0

    def send(self, cmd, val=0):
        body = bytes([SOF1, SOF2, (self.seq >> 8) & 0xFF, self.seq & 0xFF,
                      cmd & 0xFF, val & 0xFF])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as exc:
            self.last_error = str(exc)
        sent = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return pkt, sent

    def poll_ack(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(64)
            except BlockingIOError:
                break
            except OSError as exc:
                self.last_error = str(exc)
                break
            if len(data) != 4 or data[0] != ACK_BYTE:
                continue
            self.last_ack_seq = (data[1] << 8) | data[2]
            self.last_ack_status = STATUS_NAMES.get(data[3], f"0x{data[3]:02X}")
            self.last_ack_time = time.monotonic()


class ScienceSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.setblocking(False)
        self.addr = (ROVER_IP, SCI_PORT)
        self.seq = 0
        self.last_error = ""
        self.last_ack = "-"
        self.last_ack_seq = -1
        self.last_load = None
        self.last_load_err = ""

    def send(self, cmd, val=0):
        body = bytes([SOF1, SOF2, (self.seq >> 8) & 0xFF, self.seq & 0xFF,
                      cmd & 0xFF, val & 0xFF])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as exc:
            self.last_error = str(exc)
        sent = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return pkt, sent

    def poll_feedback(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(256)
            except BlockingIOError:
                break
            except OSError as exc:
                self.last_error = str(exc)
                break
            if len(data) == 4 and data[0] == ACK_BYTE:
                self.last_ack_seq = (data[1] << 8) | data[2]
                self.last_ack = STATUS_NAMES.get(data[3], hex(data[3]))
                continue
            try:
                msg = json.loads(data.decode("utf-8", errors="ignore"))
            except Exception:
                continue
            if isinstance(msg, dict) and msg.get("type") == "load_cell":
                try:
                    self.last_load = float(msg.get("raw"))
                    self.last_load_err = ""
                except Exception:
                    pass
            elif isinstance(msg, dict) and msg.get("type") == "load_cell_err":
                self.last_load_err = str(msg.get("msg", "load cell error"))


# ── Drawing helpers ──────────────────────────────────────────────────────────
def draw_panel(surf, rect, title=None):
    pygame.draw.rect(surf, (4, 4, 4), rect, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, rect, width=1, border_radius=3)
    if title:
        pygame.draw.rect(surf, ORANGE_HI, (rect.x + 10, rect.y + 10, 2, 14),
                         border_radius=1)
        lbl = FONT_SM.render(title, True, ORANGE_HI)
        surf.blit(lbl, (rect.x + 18, rect.y + 10))


def draw_hline(surf, x, y, w):
    pygame.draw.line(surf, BORDER_LINE, (x, y), (x + w, y))


def draw_button_rect(surf, rect, label, active=False, error=False, subtle=False):
    if error:
        fg, bg = RED_ERR, (30, 8, 8)
    elif active:
        fg, bg = ORANGE_HI, ORANGE_DIM
    elif subtle:
        fg, bg = ORANGE_DIM, (10, 10, 10)
    else:
        fg, bg = ORANGE_DIM, (8, 8, 8)
    pygame.draw.rect(surf, bg, rect, border_radius=3)
    pygame.draw.rect(surf, fg, rect, width=1, border_radius=3)
    txt = FONT_XS.render(label, True, ORANGE_HI if active else fg)
    surf.blit(txt, (rect.centerx - txt.get_width() // 2,
                    rect.centery - txt.get_height() // 2))


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
        v = FONT_XS.render(f"{value:+.3f}", True, ORANGE_HI)
        surf.blit(v, (x + w - v.get_width(), y - 15))


def draw_servo_slider(surf, x, y, w, h, value, label):
    pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)
    fill = int((value / 180.0) * (w - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (x + 2, y + 2, fill, h - 4),
                         border_radius=2)
    surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x, y - 16))
    v = FONT_XS.render(f"{value:3d} deg", True, ORANGE_HI)
    surf.blit(v, (x + w + 8, y - 1))


def draw_xy_pad(surf, x, y, size, ax, ay):
    pygame.draw.rect(surf, (8, 8, 8), (x, y, size, size), border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, size, size), width=1, border_radius=3)
    cx, cy = x + size // 2, y + size // 2
    for t in (0.25, 0.5, 0.75):
        gx = x + int(t * size); gy = y + int(t * size)
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


def draw_throttle_v(surf, x, y, w, h, value):
    pct = 1.0 - (value + 1) / 2.0
    pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)
    fill_h = int(pct * (h - 4))
    if fill_h > 0:
        pygame.draw.rect(surf, ORANGE_MID,
                         (x + 2, y + h - 2 - fill_h, w - 4, fill_h),
                         border_radius=2)
    surf.blit(FONT_XS.render("THR", True, ORANGE_DIM),
              (x + w // 2 - FONT_XS.size("THR")[0] // 2, y - 15))
    val = FONT_XS.render(f"{int(pct*100)}%", True, ORANGE_HI)
    surf.blit(val, (x + w // 2 - val.get_width() // 2, y + h + 4))


def draw_lift_visual(surf, rect, name, pos, active_dir):
    draw_panel(surf, rect, name)
    rail = pygame.Rect(rect.x + rect.w // 2 - 10, rect.y + 36, 20, rect.h - 64)
    pygame.draw.rect(surf, (10, 10, 10), rail, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, rail, width=1, border_radius=3)
    y_top = rail.y + 6
    y_bot = rail.bottom - 6
    y = int(y_bot - ((pos + 1.0) * 0.5) * (y_bot - y_top))
    car = pygame.Rect(rail.x - 14, y - 10, rail.w + 28, 20)
    color = ORANGE_HI if active_dir != 0 else ORANGE_DIM
    pygame.draw.rect(surf, (18, 8, 0), car, border_radius=3)
    pygame.draw.rect(surf, color, car, width=1, border_radius=3)
    st = "UP" if active_dir > 0 else "DOWN" if active_dir < 0 else "STOP"
    s = FONT_XS.render(st, True, ORANGE_HI if active_dir != 0 else ORANGE_DIM)
    surf.blit(s, (rect.centerx - s.get_width() // 2, rect.bottom - 24))


def draw_drill_visual(surf, rect, state, phase):
    draw_panel(surf, rect, "Drill")
    cx, cy = rect.centerx, rect.y + rect.h // 2 + 10
    radius = min(rect.w, rect.h) // 4
    pygame.draw.circle(surf, (10, 10, 10), (cx, cy), radius + 10)
    pygame.draw.circle(surf, ORANGE_DIM, (cx, cy), radius + 10, width=1)
    pygame.draw.circle(surf, (15, 7, 0), (cx, cy), radius)
    pygame.draw.circle(surf, ORANGE_HI if state != 0 else ORANGE_DIM,
                       (cx, cy), radius, width=2)
    for i in range(6):
        a = phase + i * (math.pi / 3.0)
        x1 = cx + int(math.cos(a) * 8)
        y1 = cy + int(math.sin(a) * 8)
        x2 = cx + int(math.cos(a) * (radius - 6))
        y2 = cy + int(math.sin(a) * (radius - 6))
        pygame.draw.line(surf, ORANGE_MID, (x1, y1), (x2, y2), 2)
    st = "COLLECT" if state > 0 else "RELEASE" if state < 0 else "STOP"
    t = FONT_XS.render(st, True, ORANGE_HI if state != 0 else ORANGE_DIM)
    surf.blit(t, (rect.centerx - t.get_width() // 2, rect.bottom - 24))


def draw_servo_row(surf, x, y, label, angle):
    surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x, y))
    bx, by, bw, bh = x + 168, y + 2, 170, 12
    pygame.draw.rect(surf, (8, 8, 8), (bx, by, bw, bh), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (bx, by, bw, bh), width=1, border_radius=2)
    fill = int((angle / 180.0) * (bw - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (bx + 2, by + 2, fill, bh - 4),
                         border_radius=2)
    a = FONT_XS.render(f"{angle:3d} deg", True, ORANGE_HI)
    surf.blit(a, (bx + bw + 8, y - 1))


# ── Tab bar ─────────────────────────────────────────────────────────────────
TAB_NAMES = ["ARM", "MISC", "SCIENCE"]


def layout_tab_bar(canvas_w):
    """Return list of (name, rect)."""
    tab_w = 150
    gap = 6
    total = tab_w * len(TAB_NAMES) + gap * (len(TAB_NAMES) - 1)
    x0 = (canvas_w - total) // 2
    rects = []
    for i, name in enumerate(TAB_NAMES):
        r = pygame.Rect(x0 + i * (tab_w + gap), TAB_AREA_Y, tab_w, TAB_BAR_H)
        rects.append((name, r))
    return rects


def draw_tab_bar(surf, active_idx):
    rects = layout_tab_bar(BASE_W)
    for i, (name, r) in enumerate(rects):
        active = (i == active_idx)
        bg = TAB_BG_ACTIVE if active else TAB_BG_INACTIVE
        fg = ORANGE_HI if active else ORANGE_DIM
        pygame.draw.rect(surf, bg, r, border_radius=4)
        pygame.draw.rect(surf, fg, r, width=2 if active else 1, border_radius=4)
        label = FONT_MD.render(f"[{i+1}] {name}", True, fg)
        surf.blit(label, (r.centerx - label.get_width() // 2,
                          r.centery - label.get_height() // 2))
    return rects


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global FONT_XS, FONT_SM, FONT_MD, FONT_LG

    pygame.init()
    pygame.joystick.init()
    pygame.event.set_grab(False)

    screen = pygame.display.set_mode((BASE_W, BASE_H),
                                     pygame.RESIZABLE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Rover Unified Control — ARM / MISC / SCIENCE")
    canvas = pygame.Surface((BASE_W, BASE_H))
    clock = pygame.time.Clock()

    for fname in ("couriernew", "courier new", "monospace"):
        try:
            FONT_XS = pygame.font.SysFont(fname, 13, bold=True)
            FONT_SM = pygame.font.SysFont(fname, 15, bold=True)
            FONT_MD = pygame.font.SysFont(fname, 19, bold=True)
            FONT_LG = pygame.font.SysFont(fname, 26, bold=True)
            break
        except Exception:
            continue

    # Senders
    arm = ArmSender()
    wheels = WheelSender()
    misc = MiscSender()
    sci = ScienceSender()
    print(f"[GUI] arm  -> {ROVER_IP}:{ARM_PORT}")
    print(f"[GUI] whl  -> {ROVER_IP}:{WHEEL_PORT}")
    print(f"[GUI] misc -> {ROVER_IP}:{MISC_PORT}")
    print(f"[GUI] sci  -> {ROVER_IP}:{SCI_PORT}")

    # Joystick
    joy = None

    def try_connect_joy():
        nonlocal joy
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            print(f"[GUI] Joystick: {joy.get_name()}")

    try_connect_joy()

    # Active tab: 0=Arm, 1=Misc, 2=Science
    active_tab = 0

    # ── Arm state ───────────────────────────────────────────────────────────
    arm_motor_states = ["STOP"] * 5
    arm_servo = 90
    arm_last_cmd = {}
    arm_pkts = 0
    arm_last_send = 0.0
    wheel_x = wheel_z = wheel_thr = 0.0
    wheel_pkts = 0
    wheel_last_send = 0.0
    laser1_on = False
    laser2_on = False

    # ── Misc state ──────────────────────────────────────────────────────────
    misc_state = {
        "night": False,
        "led_mode": 0,
        "servo1": 90, "servo2": 90, "servo3": 90, "servo4": 90,
    }
    misc_pkts = 0
    misc_last_send_btn = 0.0   # debounce on-screen clicks
    misc_last_send_loop = 0.0  # joystick-driven send rate
    misc_last_heartbeat = 0.0

    # ── Science state ───────────────────────────────────────────────────────
    sci_dc_state = {"platform": 0, "beam": 0, "drill": 0}
    sci_servo_cfg = {
        SCI_S1: {"name": "Storage A Carriage", "drill": 25, "base": 155},
        SCI_S2: {"name": "Storage A Lid", "open": 150, "close": 35},
        SCI_S3: {"name": "Storage B Carriage", "drill": 25, "base": 155},
        SCI_S4: {"name": "Storage B Lid", "open": 150, "close": 35},
    }
    sci_servo_angle = {SCI_S1: 90, SCI_S2: 90, SCI_S3: 90, SCI_S4: 90}
    sci_servo_dirty = {SCI_S1: True, SCI_S2: True, SCI_S3: True, SCI_S4: True}
    sci_load_history = deque(maxlen=140)
    sci_pkts = 0
    sci_last_send = 0.0
    sci_last_cmd = {}
    sci_platform_pos = 0.0
    sci_beam_pos = 0.0
    sci_drill_phase = 0.0

    # Joystick-edge tracking
    prev_buttons = [0] * 16
    prev_hat = (0, 0)
    # Sticky rising-edge flags consumed by rate-limited tab blocks.
    pending_edge = [False] * 16

    def consume_edge(i):
        if pending_edge[i]:
            pending_edge[i] = False
            return True
        return False

    W, H = BASE_W, BASE_H

    def window_to_canvas(pt):
        wx, wy = pt
        if (W, H) == (BASE_W, BASE_H):
            return wx, wy
        scale = min(W / BASE_W, H / BASE_H)
        out_w = int(BASE_W * scale)
        out_h = int(BASE_H * scale)
        ox = (W - out_w) // 2
        oy = (H - out_h) // 2
        if wx < ox or wy < oy or wx >= ox + out_w or wy >= oy + out_h:
            return -1, -1
        return int((wx - ox) / scale), int((wy - oy) / scale)

    def stop_wheels():
        nonlocal wheel_pkts
        wheels.send_drive(0.0, 0.0, 0.0)
        wheel_pkts += 1

    def switch_tab(new_idx):
        nonlocal active_tab
        if new_idx == active_tab:
            return
        # Leaving Arm tab — make sure wheels stop.
        if active_tab == 0 and new_idx != 0:
            stop_wheels()
        # Discard any pending button edges so a press on the previous
        # tab can't fire an unrelated action on the new tab.
        for i in range(len(pending_edge)):
            pending_edge[i] = False
        active_tab = new_idx
        print(f"[GUI] Switched to tab: {TAB_NAMES[active_tab]}")

    while True:
        now = time.monotonic()
        dt = clock.get_time() / 1000.0

        # Pump joystick state regardless of focus.
        for _ in pygame.event.get([
            pygame.JOYAXISMOTION, pygame.JOYBALLMOTION,
            pygame.JOYHATMOTION,  pygame.JOYBUTTONDOWN,
            pygame.JOYBUTTONUP]):
            pass

        click_pos = None
        mouse_pos = window_to_canvas(pygame.mouse.get_pos())
        mouse_down = pygame.mouse.get_pressed(3)[0]

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_wheels()
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    stop_wheels()
                    pygame.quit(); sys.exit()
                if event.key == pygame.K_1:
                    switch_tab(0)
                elif event.key == pygame.K_2:
                    switch_tab(1)
                elif event.key == pygame.K_3:
                    switch_tab(2)
                elif event.key == pygame.K_l and active_tab == 0:
                    laser2_on = not laser2_on
                    misc.send(MISC_LASER2_ON if laser2_on else MISC_LASER2_OFF, 0)
                    misc_pkts += 1
            if event.type == pygame.JOYDEVICEADDED:
                try_connect_joy()
            if event.type == pygame.JOYDEVICEREMOVED:
                joy = None
            if event.type == pygame.VIDEORESIZE:
                W, H = event.w, event.h
                screen = pygame.display.set_mode((W, H),
                    pygame.RESIZABLE | pygame.DOUBLEBUF)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                click_pos = window_to_canvas(event.pos)

        W, H = screen.get_size()

        # Read joystick.
        connected = joy is not None and pygame.joystick.get_count() > 0
        ax_x = ax_y = ax_z = 0.0
        ax_t = -1.0
        buttons_state = [0] * 16
        hat_x = hat_y = 0
        joy_name = ""
        if connected:
            try:
                n_axes = joy.get_numaxes()
                n_btn = joy.get_numbuttons()
                n_hat = joy.get_numhats()
                axes = [joy.get_axis(i) for i in range(n_axes)]
                buttons_state = [joy.get_button(i) for i in range(n_btn)]
                hats = [joy.get_hat(i) for i in range(n_hat)]
                ax_x = axes[0] if n_axes > 0 else 0.0
                ax_y = axes[1] if n_axes > 1 else 0.0
                ax_z = axes[2] if n_axes > 2 else 0.0
                ax_t = axes[3] if n_axes > 3 else -1.0
                hat_x, hat_y = hats[0] if n_hat > 0 else (0, 0)
                joy_name = joy.get_name()
            except Exception:
                connected = False
                buttons_state = [0] * 16
                hat_x = hat_y = 0
                ax_x = ax_y = ax_z = 0.0
                ax_t = -1.0

        # Pad buttons to length 16 for safe indexing.
        bs = list(buttons_state) + [0] * max(0, 16 - len(buttons_state))

        # Compute every-frame rising edges, then OR them into a sticky
        # set so rate-limited tab blocks (which run at ~20Hz) don't miss
        # short presses that happen between ticks.
        edge_now = [bs[i] and not prev_buttons[i] for i in range(16)]
        for i in range(16):
            if edge_now[i]:
                pending_edge[i] = True

        # ── Tab cycle on B12 (idx 11) rising edge ───────────────────────────
        if edge_now[11]:
            switch_tab((active_tab + 1) % len(TAB_NAMES))
        pending_edge[11] = False  # consumed; never seen by tab handlers

        # ── Click-to-switch on tab bar ─────────────────────────────────────
        if click_pos is not None:
            for i, (_n, r) in enumerate(layout_tab_bar(BASE_W)):
                if r.collidepoint(click_pos):
                    switch_tab(i)
                    click_pos = None  # consume the click
                    break

        # ── Misc heartbeat — runs on all tabs ───────────────────────────────
        if now - misc_last_heartbeat >= HEARTBEAT_INTERVAL:
            misc.send(MISC_HEARTBEAT, 0)
            misc_pkts += 1
            misc_last_heartbeat = now

        # ── Poll feedback from all boards always ────────────────────────────
        misc.poll_ack()
        sci.poll_feedback()

        # ─────────────────────────────────────────────────────────────────────
        # ARM TAB updates
        # ─────────────────────────────────────────────────────────────────────
        if active_tab == 0 and connected and (now - arm_last_send) >= SEND_RATE:
            arm_last_send = now

            def arm_send(key, cmd, val=255):
                nonlocal arm_pkts
                if arm_last_cmd.get(key) != cmd:
                    arm.send(cmd, val)
                    arm_last_cmd[key] = cmd
                    arm_pkts += 1

            # M1 Base via HAT E/W
            if hat_x > 0:
                arm_send("m1", ARM_M1_FWD); arm_motor_states[0] = "FWD"
            elif hat_x < 0:
                arm_send("m1", ARM_M1_REV); arm_motor_states[0] = "REV"
            else:
                arm_send("m1", ARM_M1_STOP); arm_motor_states[0] = "STOP"

            # M2 Shoulder via HAT N/S
            if hat_y > 0:
                arm_send("m2", ARM_M2_FWD); arm_motor_states[1] = "FWD"
            elif hat_y < 0:
                arm_send("m2", ARM_M2_REV); arm_motor_states[1] = "REV"
            else:
                arm_send("m2", ARM_M2_STOP); arm_motor_states[1] = "STOP"

            # M3 Elbow B6/B4
            if bs[5]:
                arm_send("m3", ARM_M3_FWD); arm_motor_states[2] = "FWD"
            elif bs[3]:
                arm_send("m3", ARM_M3_REV); arm_motor_states[2] = "REV"
            else:
                arm_send("m3", ARM_M3_STOP); arm_motor_states[2] = "STOP"

            # Wrist servo B5/B3
            if bs[4]:
                arm_servo = min(SERVO_MAX, arm_servo + SERVO_STEP)
            if bs[2]:
                arm_servo = max(SERVO_MIN, arm_servo - SERVO_STEP)
            arm.send(ARM_SERVO_ANGLE, arm_servo)
            arm_pkts += 1

            # Roller B8/B7
            if bs[7]:
                arm_send("m4", ARM_M4A_FWD); arm_motor_states[3] = "FWD"
            elif bs[6]:
                arm_send("m4", ARM_M4A_REV); arm_motor_states[3] = "REV"
            else:
                arm_send("m4", ARM_M4A_STOP); arm_motor_states[3] = "STOP"

            # Gripper B10/B9
            if bs[9]:
                arm_send("m5", ARM_M4B_FWD); arm_motor_states[4] = "FWD"
            elif bs[8]:
                arm_send("m5", ARM_M4B_REV); arm_motor_states[4] = "REV"
            else:
                arm_send("m5", ARM_M4B_STOP); arm_motor_states[4] = "STOP"

            # Laser 1 toggle on B11 (idx 10) rising edge — Arm tab only.
            if consume_edge(10):
                laser1_on = not laser1_on
                misc.send(MISC_LASER1_ON if laser1_on else MISC_LASER1_OFF, 0)
                misc_pkts += 1

        # Wheel drive — only while on Arm tab.
        if active_tab == 0 and connected and (now - wheel_last_send) >= WHEEL_SEND_RATE:
            wheel_last_send = now
            raw_x = -ax_y
            raw_z = ax_z
            wheel_x = raw_x if abs(raw_x) > DEADZONE else 0.0
            wheel_z = raw_z if abs(raw_z) > DEADZONE else 0.0
            wheel_thr = (1.0 - ax_t) / 2.0
            wheels.send_drive(wheel_x, wheel_z, wheel_thr)
            wheel_pkts += 1
        elif active_tab != 0:
            wheel_x = wheel_z = wheel_thr = 0.0

        # ─────────────────────────────────────────────────────────────────────
        # MISC TAB updates
        # ─────────────────────────────────────────────────────────────────────
        if active_tab == 1 and connected and (now - misc_last_send_loop) >= SEND_RATE:
            misc_last_send_loop = now

            def misc_send_cmd(cmd, val=0):
                nonlocal misc_pkts
                misc.send(cmd, val)
                misc_pkts += 1

            # Night toggle on B11 (idx 10)
            if consume_edge(10):
                misc_state["night"] = not misc_state["night"]
                misc_send_cmd(MISC_NIGHT_ON if misc_state["night"] else MISC_NIGHT_OFF)

            # E-stop on trigger (idx 0)
            if consume_edge(0):
                misc_state["night"] = False
                misc_state["led_mode"] = 0
                misc_send_cmd(MISC_ESTOP, 0)

            # LED mode cycle on B14 (idx 13). B13 (idx 12) also cycles.
            if consume_edge(12):
                misc_state["led_mode"] = (misc_state["led_mode"] + 1) % 4
                misc_send_cmd(MISC_IND_ON, 0)
            if consume_edge(13):
                misc_state["led_mode"] = (misc_state["led_mode"] + 1) % 4
                misc_send_cmd(MISC_IND_ON, 0)

            # D-pad direct LED mode: Up=RED, Right=GREEN, Left=YELLOW, Down=OFF
            if (hat_x, hat_y) != prev_hat:
                if hat_y == 1:
                    misc_state["led_mode"] = 1; misc_send_cmd(MISC_IND_PWM, 1)
                elif hat_x == 1:
                    misc_state["led_mode"] = 2; misc_send_cmd(MISC_IND_PWM, 2)
                elif hat_x == -1:
                    misc_state["led_mode"] = 3; misc_send_cmd(MISC_IND_PWM, 3)
                elif hat_y == -1:
                    misc_state["led_mode"] = 0; misc_send_cmd(MISC_IND_PWM, 0)

            # Servo trims (held)
            if bs[4]:
                misc_state["servo1"] = min(180, misc_state["servo1"] + SERVO_STEP)
                misc_send_cmd(MISC_SERVO1, misc_state["servo1"])
            if bs[2]:
                misc_state["servo1"] = max(0, misc_state["servo1"] - SERVO_STEP)
                misc_send_cmd(MISC_SERVO1, misc_state["servo1"])
            if bs[5]:
                misc_state["servo2"] = min(180, misc_state["servo2"] + SERVO_STEP)
                misc_send_cmd(MISC_SERVO2, misc_state["servo2"])
            if bs[3]:
                misc_state["servo2"] = max(0, misc_state["servo2"] - SERVO_STEP)
                misc_send_cmd(MISC_SERVO2, misc_state["servo2"])
            if bs[9]:
                misc_state["servo3"] = min(180, misc_state["servo3"] + SERVO_STEP)
                misc_send_cmd(MISC_SERVO3, misc_state["servo3"])
            if bs[8]:
                misc_state["servo3"] = max(0, misc_state["servo3"] - SERVO_STEP)
                misc_send_cmd(MISC_SERVO3, misc_state["servo3"])
            if bs[1]:
                misc_state["servo4"] = min(180, misc_state["servo4"] + SERVO_STEP)
                misc_send_cmd(MISC_SERVO4, misc_state["servo4"])
            if bs[6]:
                misc_state["servo4"] = max(0, misc_state["servo4"] - SERVO_STEP)
                misc_send_cmd(MISC_SERVO4, misc_state["servo4"])

        # ─────────────────────────────────────────────────────────────────────
        # SCIENCE TAB updates
        # ─────────────────────────────────────────────────────────────────────
        if active_tab == 2 and connected:
            sci_dc_state["platform"] = 1 if hat_y > 0 else -1 if hat_y < 0 else 0
            sci_dc_state["beam"] = 1 if hat_x > 0 else -1 if hat_x < 0 else 0
            sci_dc_state["drill"] = 1 if bs[7] else -1 if bs[6] else 0

            # Servo nudges (held)
            if bs[4]:
                sci_servo_angle[SCI_S1] = min(SERVO_MAX, sci_servo_angle[SCI_S1] + SERVO_STEP)
                sci_servo_dirty[SCI_S1] = True
            if bs[2]:
                sci_servo_angle[SCI_S1] = max(SERVO_MIN, sci_servo_angle[SCI_S1] - SERVO_STEP)
                sci_servo_dirty[SCI_S1] = True
            if bs[5]:
                sci_servo_angle[SCI_S2] = min(SERVO_MAX, sci_servo_angle[SCI_S2] + SERVO_STEP)
                sci_servo_dirty[SCI_S2] = True
            if bs[3]:
                sci_servo_angle[SCI_S2] = max(SERVO_MIN, sci_servo_angle[SCI_S2] - SERVO_STEP)
                sci_servo_dirty[SCI_S2] = True
            if bs[9]:
                sci_servo_angle[SCI_S3] = min(SERVO_MAX, sci_servo_angle[SCI_S3] + SERVO_STEP)
                sci_servo_dirty[SCI_S3] = True
            if bs[8]:
                sci_servo_angle[SCI_S3] = max(SERVO_MIN, sci_servo_angle[SCI_S3] - SERVO_STEP)
                sci_servo_dirty[SCI_S3] = True
            # Trigger / thumb for S4 (note: B12 idx 11 is reserved for tab cycle)
            if bs[0]:
                sci_servo_angle[SCI_S4] = min(SERVO_MAX, sci_servo_angle[SCI_S4] + SERVO_STEP)
                sci_servo_dirty[SCI_S4] = True
            if bs[1]:
                sci_servo_angle[SCI_S4] = max(SERVO_MIN, sci_servo_angle[SCI_S4] - SERVO_STEP)
                sci_servo_dirty[SCI_S4] = True

            # Load read on B11 (idx 10) edge
            if consume_edge(10):
                sci.send(SCI_LOAD_READ, 0)
                sci_pkts += 1

        if active_tab == 2 and (now - sci_last_send) >= SEND_RATE:
            sci_last_send = now
            motor_cmds = {
                "platform": (SCI_M1_FWD, SCI_M1_REV, SCI_M1_STOP),
                "beam": (SCI_M2_FWD, SCI_M2_REV, SCI_M2_STOP),
                "drill": (SCI_M3_FWD, SCI_M3_REV, SCI_M3_STOP),
            }
            for name, st in sci_dc_state.items():
                fwd, rev, stp = motor_cmds[name]
                cmd = fwd if st > 0 else rev if st < 0 else stp
                if sci_last_cmd.get(name) != cmd:
                    sci.send(cmd, 220 if name == "drill" and st != 0 else 255)
                    sci_pkts += 1
                    sci_last_cmd[name] = cmd
            for scmd, dirty in sci_servo_dirty.items():
                if dirty:
                    sci.send(scmd, sci_servo_angle[scmd])
                    sci_pkts += 1
                    sci_servo_dirty[scmd] = False

        # If we left the Science tab, send a stop to the science motors once.
        if active_tab != 2 and any(v != 0 for v in sci_dc_state.values()):
            sci_dc_state["platform"] = 0
            sci_dc_state["beam"] = 0
            sci_dc_state["drill"] = 0
            sci.send(SCI_M1_STOP, 0); sci_pkts += 1
            sci.send(SCI_M2_STOP, 0); sci_pkts += 1
            sci.send(SCI_M3_STOP, 0); sci_pkts += 1
            sci_last_cmd = {"platform": SCI_M1_STOP, "beam": SCI_M2_STOP, "drill": SCI_M3_STOP}

        prev_buttons = bs[:]
        prev_hat = (hat_x, hat_y)

        # ─────────────────────────────────────────────────────────────────────
        # DRAW
        # ─────────────────────────────────────────────────────────────────────
        canvas.fill(BG)

        title = FONT_LG.render("ROVER UNIFIED CONTROL", True, ORANGE_HI)
        canvas.blit(title, (18, 10))
        ctrl_lbl = FONT_XS.render(
            f"CTRL: {'CONNECTED' if connected else 'NO CONTROLLER'}",
            True, ORANGE_HI if connected else RED_ERR)
        canvas.blit(ctrl_lbl, (BASE_W - 220, 14))
        if joy_name:
            jn = FONT_XS.render(joy_name[:34], True, ORANGE_DIM)
            canvas.blit(jn, (BASE_W - 380, 32))

        draw_tab_bar(canvas, active_tab)

        content_top = TAB_AREA_Y + TAB_BAR_H + 10
        draw_hline(canvas, 16, content_top - 4, BASE_W - 32)

        if active_tab == 0:
            draw_arm_tab(canvas, content_top, ax_x, ax_y, ax_z, ax_t,
                         arm_motor_states, arm_servo, arm_pkts, arm.last_error,
                         wheel_x, wheel_z, wheel_thr, wheel_pkts, wheels.last_error,
                         laser1_on, laser2_on)
        elif active_tab == 1:
            draw_misc_tab(canvas, content_top, ax_x, ax_y, ax_z, hat_x, hat_y,
                          misc_state, misc_pkts, misc.last_error,
                          misc.last_ack_status, misc.last_ack_seq, misc.last_ack_time,
                          now, click_pos)
        else:
            draw_science_tab(canvas, content_top, ax_x, ax_y, ax_z, hat_x, hat_y,
                             sci_dc_state, sci_servo_cfg, sci_servo_angle,
                             sci_load_history, sci_platform_pos, sci_beam_pos,
                             sci_drill_phase, sci_pkts, sci.last_error,
                             sci.last_ack, sci.last_ack_seq,
                             sci.last_load, sci.last_load_err,
                             click_pos, mouse_down, mouse_pos)

        # Footer
        if active_tab == 0:
            foot = ("ARM:  HAT E/W: Base  HAT N/S: Shoulder  B6/B4: Elbow  "
                    "B5/B3: Wrist  B8/B7: Roller  B10/B9: Gripper  "
                    "B11: Laser1  L key: Laser2  Stick+THR: Drive   |   B12: Next Tab")
        elif active_tab == 1:
            foot = ("MISC: B11: Night  D-pad U/R/L/D: RED/GREEN/YELLOW/OFF  "
                    "B5/B3: S2  B6/B4: S1  B10/B9: S3  Thumb/B7: S4  "
                    "Trigger: ESTOP   |   B12: Next Tab")
        else:
            foot = ("SCI:  HAT N/S: Platform  HAT E/W: Beam  B8/B7: Drill C/R  "
                    "B5/B3: S1  B6/B4: S2  B10/B9: S3  Trigger/Thumb: S4  "
                    "B11: Load Read   |   B12: Next Tab")
        f = FONT_XS.render(foot, True, ORANGE_MID)
        canvas.blit(f, (BASE_W // 2 - f.get_width() // 2, BASE_H - 18))

        # ── Tab-specific click handling that needs main-state mutation ──────
        # Done after draw so we use the same canvas-space rects produced.
        if click_pos is not None and active_tab == 0:
            # Arm laser buttons
            r1 = pygame.Rect(540, content_top + 10, 130, 36)
            r2 = pygame.Rect(680, content_top + 10, 130, 36)
            estp = pygame.Rect(820, content_top + 10, 130, 36)
            if r1.collidepoint(click_pos):
                laser1_on = not laser1_on
                misc.send(MISC_LASER1_ON if laser1_on else MISC_LASER1_OFF, 0)
                misc_pkts += 1
            elif r2.collidepoint(click_pos):
                laser2_on = not laser2_on
                misc.send(MISC_LASER2_ON if laser2_on else MISC_LASER2_OFF, 0)
                misc_pkts += 1
            elif estp.collidepoint(click_pos):
                # Arm tab E-stop: stop motors + lasers off
                arm.send(ARM_M1_STOP, 0); arm_pkts += 1
                arm.send(ARM_M2_STOP, 0); arm_pkts += 1
                arm.send(ARM_M3_STOP, 0); arm_pkts += 1
                arm.send(ARM_M4A_STOP, 0); arm_pkts += 1
                arm.send(ARM_M4B_STOP, 0); arm_pkts += 1
                if laser1_on:
                    laser1_on = False
                    misc.send(MISC_LASER1_OFF, 0); misc_pkts += 1
                if laser2_on:
                    laser2_on = False
                    misc.send(MISC_LASER2_OFF, 0); misc_pkts += 1
                stop_wheels()

        if click_pos is not None and active_tab == 1:
            # Top control buttons positions defined in draw_misc_tab
            # We re-evaluate the rectangles here and act.
            misc_buttons = misc_layout(content_top)
            for key, r in misc_buttons.items():
                if r.collidepoint(click_pos) and (now - misc_last_send_btn) >= SEND_GAP_S:
                    misc_last_send_btn = now
                    if key == "night":
                        misc_state["night"] = not misc_state["night"]
                        misc.send(MISC_NIGHT_ON if misc_state["night"]
                                  else MISC_NIGHT_OFF, 0)
                        misc_pkts += 1
                    elif key == "estop":
                        misc_state["night"] = False
                        misc_state["led_mode"] = 0
                        misc.send(MISC_ESTOP, 0); misc_pkts += 1
                    elif key == "led_off":
                        misc_state["led_mode"] = 0
                        misc.send(MISC_IND_PWM, 0); misc_pkts += 1
                    elif key == "led_red":
                        misc_state["led_mode"] = 1
                        misc.send(MISC_IND_PWM, 1); misc_pkts += 1
                    elif key == "led_green":
                        misc_state["led_mode"] = 2
                        misc.send(MISC_IND_PWM, 2); misc_pkts += 1
                    elif key == "led_yellow":
                        misc_state["led_mode"] = 3
                        misc.send(MISC_IND_PWM, 3); misc_pkts += 1
                    elif key == "led_toggle":
                        misc_state["led_mode"] = (misc_state["led_mode"] + 1) % 4
                        misc.send(MISC_IND_ON, 0); misc_pkts += 1
                    elif key.endswith("_dn"):
                        sk = key[:-3]
                        misc_state[sk] = max(0, misc_state[sk] - 5)
                        cmd_map = {"servo1": MISC_SERVO1, "servo2": MISC_SERVO2,
                                   "servo3": MISC_SERVO3, "servo4": MISC_SERVO4}
                        misc.send(cmd_map[sk], misc_state[sk]); misc_pkts += 1
                    elif key.endswith("_up"):
                        sk = key[:-3]
                        misc_state[sk] = min(180, misc_state[sk] + 5)
                        cmd_map = {"servo1": MISC_SERVO1, "servo2": MISC_SERVO2,
                                   "servo3": MISC_SERVO3, "servo4": MISC_SERVO4}
                        misc.send(cmd_map[sk], misc_state[sk]); misc_pkts += 1
                    break

        if click_pos is not None and active_tab == 2:
            sci_buttons = science_button_layout(content_top)
            for key, r in sci_buttons.items():
                if not r.collidepoint(click_pos):
                    continue
                if key == "s1_drill":
                    sci_servo_angle[SCI_S1] = sci_servo_cfg[SCI_S1]["drill"]
                    sci_servo_dirty[SCI_S1] = True
                elif key == "s1_base":
                    sci_servo_angle[SCI_S1] = sci_servo_cfg[SCI_S1]["base"]
                    sci_servo_dirty[SCI_S1] = True
                elif key == "s2_open":
                    sci_servo_angle[SCI_S2] = sci_servo_cfg[SCI_S2]["open"]
                    sci_servo_dirty[SCI_S2] = True
                elif key == "s2_close":
                    sci_servo_angle[SCI_S2] = sci_servo_cfg[SCI_S2]["close"]
                    sci_servo_dirty[SCI_S2] = True
                elif key == "s3_drill":
                    sci_servo_angle[SCI_S3] = sci_servo_cfg[SCI_S3]["drill"]
                    sci_servo_dirty[SCI_S3] = True
                elif key == "s3_base":
                    sci_servo_angle[SCI_S3] = sci_servo_cfg[SCI_S3]["base"]
                    sci_servo_dirty[SCI_S3] = True
                elif key == "s4_open":
                    sci_servo_angle[SCI_S4] = sci_servo_cfg[SCI_S4]["open"]
                    sci_servo_dirty[SCI_S4] = True
                elif key == "s4_close":
                    sci_servo_angle[SCI_S4] = sci_servo_cfg[SCI_S4]["close"]
                    sci_servo_dirty[SCI_S4] = True
                elif key == "center":
                    sci.send(SCI_SERVO_CENTER, 90); sci_pkts += 1
                    for k in sci_servo_angle:
                        sci_servo_angle[k] = 90
                        sci_servo_dirty[k] = False
                elif key == "load_read":
                    sci.send(SCI_LOAD_READ, 0); sci_pkts += 1
                elif key == "load_tare":
                    sci.send(SCI_LOAD_TARE, 0); sci_pkts += 1
                elif key == "estop":
                    sci.send(SCI_ESTOP, 0); sci_pkts += 1
                break

            # Mouse-hold motor override buttons
            sci_motor_buttons = science_motor_button_layout(content_top)
            if mouse_down:
                for (mname, direction), rect in sci_motor_buttons.items():
                    if rect.collidepoint(mouse_pos):
                        sci_dc_state[mname] = direction

        # Animate science visuals
        sci_platform_pos = max(-1.0, min(1.0,
            sci_platform_pos + sci_dc_state["platform"] * dt * 0.8))
        sci_beam_pos = max(-1.0, min(1.0,
            sci_beam_pos + sci_dc_state["beam"] * dt * 0.8))
        sci_drill_phase += sci_dc_state["drill"] * dt * 6.5

        # Scale to window
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


# ─────────────────────────────────────────────────────────────────────────────
# Per-tab draw functions
# ─────────────────────────────────────────────────────────────────────────────
def draw_arm_tab(surf, top, ax_x, ax_y, ax_z, ax_t,
                 motor_states, servo_angle, packets_sent, net_err,
                 wheel_x, wheel_z, wheel_thr, wheel_pkts, wheel_err,
                 laser1_on, laser2_on):
    # Top action row: laser buttons + arm e-stop
    row_y = top + 4
    surf.blit(FONT_SM.render("ARM TAB", True, ORANGE_HI), (20, row_y + 12))

    l1 = pygame.Rect(540, top + 10, 130, 36)
    l2 = pygame.Rect(680, top + 10, 130, 36)
    estp = pygame.Rect(820, top + 10, 130, 36)
    draw_button_rect(surf, l1, "LASER 1", active=laser1_on)
    draw_button_rect(surf, l2, "LASER 2", active=laser2_on)
    draw_button_rect(surf, estp, "ARM E-STOP", error=True)

    # Axes panel
    LP = pygame.Rect(14, top + 60, 440, 360)
    draw_panel(surf, LP, "AXES & STICK")
    ay = LP.y + 40
    draw_axis_bar(surf, LP.x + 14, ay,     LP.w - 28, 14, ax_x, "X AXIS   L/R")
    draw_axis_bar(surf, LP.x + 14, ay + 42,  LP.w - 28, 14, ax_y, "Y AXIS   FWD/BACK")
    draw_axis_bar(surf, LP.x + 14, ay + 84,  LP.w - 28, 14, ax_z, "Z TWIST  RUDDER")
    draw_xy_pad(surf, LP.x + 14, ay + 126, 140, ax_x, ax_y)
    draw_throttle_v(surf, LP.x + 172, ay + 126, 34, 140, ax_t)
    raw_y = ay + 286
    rr = pygame.Rect(LP.x + 14, raw_y, LP.w - 28, 42)
    pygame.draw.rect(surf, (6, 2, 0), rr, border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, rr, width=1, border_radius=2)
    surf.blit(FONT_XS.render("RAW AXIS VALUES", True, ORANGE_MID),
              (rr.x + 8, rr.y + 4))
    for i, (n, v) in enumerate(zip(["AX0", "AX1", "AX2", "AX3"],
                                    [ax_x, ax_y, ax_z, ax_t])):
        rx = rr.x + 10 + i * (rr.w // 4)
        surf.blit(FONT_XS.render(n, True, ORANGE_DIM), (rx, rr.y + 17))
        surf.blit(FONT_XS.render(f"{v:+.4f}", True, ORANGE_HI), (rx, rr.y + 28))

    # Motor / servo state panel
    MP = pygame.Rect(466, top + 60, 700, 188)
    draw_panel(surf, MP, "ARM / MOTOR COMMANDS")
    x0, y0 = MP.x + 14, MP.y + 36
    all_names = MOTOR_NAMES + [SERVO_NAME]
    col_w = (MP.w - 28) // len(all_names)
    for i, name in enumerate(all_names):
        cx = x0 + i * col_w + col_w // 2
        surf.blit(FONT_XS.render(name, True, ORANGE_DIM),
                  (cx - FONT_XS.size(name)[0] // 2, y0))
        if name == SERVO_NAME:
            bar_x = cx - col_w // 2 + 4
            bar_w = col_w - 8
            by = y0 + 16
            pygame.draw.rect(surf, (8, 8, 8), (bar_x, by, bar_w, 14), border_radius=2)
            pygame.draw.rect(surf, ORANGE_DIM, (bar_x, by, bar_w, 14), width=1, border_radius=2)
            fill = int((servo_angle / 180) * (bar_w - 4))
            if fill > 0:
                pygame.draw.rect(surf, ORANGE_MID,
                                 (bar_x + 2, by + 2, fill, 10), border_radius=2)
            ang = FONT_XS.render(f"{servo_angle} deg", True, ORANGE_HI)
            surf.blit(ang, (cx - ang.get_width() // 2, by + 17))
        else:
            state = motor_states[i]
            if state == "FWD":
                disp = MOTOR_DIR_LABELS[name][0]; sc = ORANGE_HI
            elif state == "REV":
                disp = MOTOR_DIR_LABELS[name][1]; sc = RED_ERR
            else:
                disp = "STOP"; sc = (60, 30, 0)
            bc = sc if state != "STOP" else ORANGE_DIM
            bx = cx - col_w // 2 + 4
            bw_box = col_w - 8
            by = y0 + 16
            pygame.draw.rect(surf, (6, 2, 0), (bx, by, bw_box, 18), border_radius=2)
            pygame.draw.rect(surf, bc, (bx, by, bw_box, 18), width=1, border_radius=2)
            st = FONT_XS.render(disp, True, sc)
            surf.blit(st, (cx - st.get_width() // 2, by + 3))
    nc = ORANGE_HI if not net_err else RED_ERR
    surf.blit(FONT_XS.render(
        f"UDP {ROVER_IP}:{ARM_PORT}   pkts:{packets_sent}", True, nc),
        (x0, MP.bottom - 22))

    # Wheel panel
    WP = pygame.Rect(466, top + 256, 700, 164)
    draw_panel(surf, WP, "WHEEL DRIVE")
    wx0, wy0 = WP.x + 14, WP.y + 36
    bar_w = WP.w - 124

    def wrow(label, value, y):
        surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (wx0, y))
        draw_axis_bar(surf, wx0 + 100, y, bar_w, 12, value)

    wrow("FWD / REV", wheel_x, wy0)
    wrow("TURN",      wheel_z, wy0 + 22)
    ty = wy0 + 44
    surf.blit(FONT_XS.render("THROTTLE", True, ORANGE_DIM), (wx0, ty))
    bx = wx0 + 100
    pygame.draw.rect(surf, (8, 8, 8), (bx, ty, bar_w, 12), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (bx, ty, bar_w, 12), width=1, border_radius=2)
    fill = int(wheel_thr * (bar_w - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (bx + 2, ty + 2, fill, 8), border_radius=2)
    surf.blit(FONT_XS.render(f"{int(wheel_thr*100)}%", True, ORANGE_HI),
              (bx + bar_w + 6, ty))
    lv = max(-1.0, min(1.0, wheel_x - wheel_z)) * wheel_thr
    rv = max(-1.0, min(1.0, wheel_x + wheel_z)) * wheel_thr
    wrow("LEFT  MTR", lv, ty + 22)
    wrow("RIGHT MTR", rv, ty + 44)
    wnc = ORANGE_HI if not wheel_err else RED_ERR
    surf.blit(FONT_XS.render(
        f"UDP {ROVER_IP}:{WHEEL_PORT}   pkts:{wheel_pkts}", True, wnc),
        (wx0, WP.bottom - 18))

    # Laser status panel
    LS = pygame.Rect(14, top + 430, 1152, 80)
    draw_panel(surf, LS, "LASERS  (routed to MISC board)")
    l1c = ORANGE_HI if laser1_on else ORANGE_DIM
    l2c = ORANGE_HI if laser2_on else ORANGE_DIM
    surf.blit(FONT_MD.render(f"LASER 1: {'ON' if laser1_on else 'OFF'}", True, l1c),
              (LS.x + 24, LS.y + 36))
    surf.blit(FONT_MD.render(f"LASER 2: {'ON' if laser2_on else 'OFF'}", True, l2c),
              (LS.x + 320, LS.y + 36))
    surf.blit(FONT_XS.render(
        "B11 (controller) toggles Laser 1   |   L key toggles Laser 2   |   "
        "On-screen buttons toggle either",
        True, ORANGE_MID),
        (LS.x + 620, LS.y + 44))


def misc_layout(top):
    """Return dict of misc on-screen button rects."""
    btns = {
        "night":      pygame.Rect(28,  top + 56, 170, 44),
        "estop":      pygame.Rect(218, top + 56, 170, 44),
        "led_toggle": pygame.Rect(28,  top + 130, 190, 36),
        "led_off":    pygame.Rect(238, top + 130, 130, 36),
        "led_red":    pygame.Rect(378, top + 130, 130, 36),
        "led_green":  pygame.Rect(518, top + 130, 130, 36),
        "led_yellow": pygame.Rect(658, top + 130, 130, 36),
    }
    rows = [("servo1", 250), ("servo2", 315), ("servo3", 380), ("servo4", 445)]
    for key, y_off in rows:
        btns[f"{key}_dn"] = pygame.Rect(720, top + y_off - 4, 54, 28)
        btns[f"{key}_up"] = pygame.Rect(780, top + y_off - 4, 54, 28)
    return btns


def draw_misc_tab(surf, top, ax_x, ax_y, ax_z, hat_x, hat_y,
                  state, packets_sent, net_err,
                  ack_status, ack_seq, ack_time, now,
                  click_pos):
    P = pygame.Rect(14, top + 4, 1152, 200)
    draw_panel(surf, P, "Power and Safety")
    btns = misc_layout(top)
    draw_button_rect(surf, btns["night"], "NIGHT LED", active=state["night"])
    draw_button_rect(surf, btns["estop"], "MISC E-STOP", error=True)
    surf.blit(FONT_XS.render("LED MATRIX MODES:", True, ORANGE_DIM),
              (28, top + 110))
    draw_button_rect(surf, btns["led_toggle"], "LED TOGGLE")
    draw_button_rect(surf, btns["led_off"], "OFF",
                     active=state["led_mode"] == 0)
    draw_button_rect(surf, btns["led_red"], "RED",
                     active=state["led_mode"] == 1)
    draw_button_rect(surf, btns["led_green"], "GREEN",
                     active=state["led_mode"] == 2)
    draw_button_rect(surf, btns["led_yellow"], "YELLOW",
                     active=state["led_mode"] == 3)
    surf.blit(FONT_XS.render(
        f"Current: {LED_MODE_NAMES[state['led_mode']]}", True, ORANGE_GLOW),
        (808, top + 144))

    # Joystick monitor
    JM = pygame.Rect(14, top + 220, 332, 318)
    draw_panel(surf, JM, "Joystick Monitor")
    draw_axis_bar(surf, JM.x + 14, JM.y + 64, 220, 14, ax_x, "AXIS X")
    draw_axis_bar(surf, JM.x + 14, JM.y + 128, 220, 14, ax_y, "AXIS Y")
    draw_axis_bar(surf, JM.x + 14, JM.y + 192, 220, 14, ax_z, "AXIS Z")
    surf.blit(FONT_SM.render(f"HAT: ({hat_x:+d}, {hat_y:+d})", True, ORANGE_HI),
              (JM.x + 14, JM.bottom - 42))

    # Servo panel
    SP = pygame.Rect(356, top + 220, 810, 318)
    draw_panel(surf, SP, "Servo Test")
    rows = [("servo1", 250), ("servo2", 315), ("servo3", 380), ("servo4", 445)]
    for key, y_off in rows:
        draw_servo_slider(surf, 378, top + y_off, 320, 14, state[key],
                          key.upper())
        draw_button_rect(surf, btns[f"{key}_dn"], "-")
        draw_button_rect(surf, btns[f"{key}_up"], "+")

    # Network panel
    NP = pygame.Rect(14, top + 552, 1152, 160)
    draw_panel(surf, NP, "Link and ACK")
    nc = ORANGE_HI if not net_err else RED_ERR
    surf.blit(FONT_SM.render(f"UDP target: {ROVER_IP}:{MISC_PORT}", True, nc),
              (32, top + 590))
    surf.blit(FONT_SM.render(f"Packets sent: {packets_sent}", True, ORANGE_HI),
              (32, top + 614))
    age = now - ack_time if ack_time > 0 else -1.0
    ack_color = ORANGE_HI if 0 <= age < 2.0 else ORANGE_DIM
    surf.blit(FONT_SM.render(
        f"Last ACK: {ack_status}   seq={ack_seq}", True, ack_color),
        (32, top + 638))
    age_text = "ACK age: --" if age < 0 else f"ACK age: {age:.2f}s"
    surf.blit(FONT_SM.render(age_text, True, ack_color),
              (32, top + 662))
    if net_err:
        surf.blit(FONT_SM.render(f"Socket error: {net_err}", True, RED_ERR),
                  (420, top + 614))


def science_motor_button_layout(top):
    p_dc_x = 382
    p_dc_y = top + 6
    col_w = 244
    rects = {}
    # Platform col
    r1 = pygame.Rect(p_dc_x + 12, p_dc_y + 36, col_w, 232)
    rects[("platform", 1)]  = pygame.Rect(r1.x + 20, r1.y + 56, 86, 30)
    rects[("platform", -1)] = pygame.Rect(r1.x + 20, r1.y + 94, 86, 30)
    # Beam col
    r2 = pygame.Rect(p_dc_x + 270, p_dc_y + 36, col_w, 232)
    rects[("beam", 1)]  = pygame.Rect(r2.x + 20, r2.y + 56, 86, 30)
    rects[("beam", -1)] = pygame.Rect(r2.x + 20, r2.y + 94, 86, 30)
    # Drill col
    r3 = pygame.Rect(p_dc_x + 528, p_dc_y + 36, col_w, 232)
    rects[("drill", 1)]  = pygame.Rect(r3.x + 20, r3.y + 56, 100, 30)
    rects[("drill", -1)] = pygame.Rect(r3.x + 20, r3.y + 94, 100, 30)
    return rects


def science_button_layout(top):
    p_sv_x, p_sv_y = 12, top + 214
    p_sv_w, p_sv_h = 760, 390
    rects = {}

    sa_x = p_sv_x + 10
    sa_y = p_sv_y + 32
    sa_right = p_sv_x + p_sv_w - 10
    rects["s1_drill"] = pygame.Rect(sa_right - 248, sa_y + 36, 74, 26)
    rects["s1_base"]  = pygame.Rect(sa_right - 166, sa_y + 36, 74, 26)
    rects["s2_open"]  = pygame.Rect(sa_right - 248, sa_y + 78, 74, 26)
    rects["s2_close"] = pygame.Rect(sa_right - 166, sa_y + 78, 74, 26)

    sb_y = p_sv_y + 212
    rects["s3_drill"] = pygame.Rect(sa_right - 248, sb_y + 36, 74, 26)
    rects["s3_base"]  = pygame.Rect(sa_right - 166, sb_y + 36, 74, 26)
    rects["s4_open"]  = pygame.Rect(sa_right - 248, sb_y + 78, 74, 26)
    rects["s4_close"] = pygame.Rect(sa_right - 166, sb_y + 78, 74, 26)

    rects["center"] = pygame.Rect(p_sv_x + p_sv_w - 126,
                                  p_sv_y + p_sv_h - 38, 112, 26)

    p_load_x = 782
    p_load_y = top + 296
    rects["load_read"] = pygame.Rect(p_load_x + 18, p_load_y + 38, 110, 32)
    rects["load_tare"] = pygame.Rect(p_load_x + 136, p_load_y + 38, 110, 32)
    rects["estop"]     = pygame.Rect(p_load_x + 254, p_load_y + 38, 112, 32)
    return rects


def draw_science_tab(surf, top, ax_x, ax_y, ax_z, hat_x, hat_y,
                     dc_state, servo_cfg, servo_angle, load_history,
                     platform_pos, beam_pos, drill_phase,
                     packets_sent, net_err, ack_status, ack_seq,
                     last_load, last_load_err,
                     click_pos, mouse_down, mouse_pos):
    # Joystick monitor
    JM = pygame.Rect(12, top + 6, 360, 198)
    draw_panel(surf, JM, "Joystick Monitor")
    draw_axis_bar(surf, JM.x + 12, JM.y + 46, JM.w - 24, 14, ax_x, "Axis X")
    draw_axis_bar(surf, JM.x + 12, JM.y + 82, JM.w - 24, 14, ax_y, "Axis Y")
    draw_axis_bar(surf, JM.x + 12, JM.y + 118, JM.w - 24, 14, ax_z, "Axis Z Twist")
    surf.blit(FONT_XS.render(
        f"HAT: ({hat_x:+d}, {hat_y:+d})", True, ORANGE_HI),
        (JM.x + 14, JM.bottom - 30))

    # DC motor section
    DC = pygame.Rect(382, top + 6, 786, 280)
    draw_panel(surf, DC, "DC Motors (Platform / Beam / Drill)")
    col_w = 244
    r1 = pygame.Rect(DC.x + 12, DC.y + 36, col_w, 232)
    draw_lift_visual(surf, r1, "Platform", platform_pos, dc_state["platform"])
    draw_button_rect(surf, pygame.Rect(r1.x + 20, r1.y + 56, 86, 30),
                     "UP", active=dc_state["platform"] > 0)
    draw_button_rect(surf, pygame.Rect(r1.x + 20, r1.y + 94, 86, 30),
                     "DOWN", active=dc_state["platform"] < 0)

    r2 = pygame.Rect(DC.x + 270, DC.y + 36, col_w, 232)
    draw_lift_visual(surf, r2, "Beam", beam_pos, dc_state["beam"])
    draw_button_rect(surf, pygame.Rect(r2.x + 20, r2.y + 56, 86, 30),
                     "UP", active=dc_state["beam"] > 0)
    draw_button_rect(surf, pygame.Rect(r2.x + 20, r2.y + 94, 86, 30),
                     "DOWN", active=dc_state["beam"] < 0)

    r3 = pygame.Rect(DC.x + 528, DC.y + 36, col_w, 232)
    draw_drill_visual(surf, r3, dc_state["drill"], drill_phase)
    draw_button_rect(surf, pygame.Rect(r3.x + 20, r3.y + 56, 100, 30),
                     "COLLECT", active=dc_state["drill"] > 0)
    draw_button_rect(surf, pygame.Rect(r3.x + 20, r3.y + 94, 100, 30),
                     "RELEASE", active=dc_state["drill"] < 0)

    # Servo / storage panel
    SP = pygame.Rect(12, top + 214, 760, 390)
    draw_panel(surf, SP, "Storage Servos")

    sa = pygame.Rect(SP.x + 10, SP.y + 32, SP.w - 20, 170)
    pygame.draw.rect(surf, (9, 9, 9), sa, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, sa, width=1, border_radius=3)
    surf.blit(FONT_SM.render("Storage A", True, ORANGE_HI), (sa.x + 10, sa.y + 8))
    draw_servo_row(surf, sa.x + 12, sa.y + 40, servo_cfg[SCI_S1]["name"],
                   servo_angle[SCI_S1])
    draw_servo_row(surf, sa.x + 12, sa.y + 82, servo_cfg[SCI_S2]["name"],
                   servo_angle[SCI_S2])
    draw_button_rect(surf, pygame.Rect(sa.right - 248, sa.y + 36, 74, 26), "DRILL")
    draw_button_rect(surf, pygame.Rect(sa.right - 166, sa.y + 36, 74, 26), "BASE")
    draw_button_rect(surf, pygame.Rect(sa.right - 248, sa.y + 78, 74, 26), "OPEN")
    draw_button_rect(surf, pygame.Rect(sa.right - 166, sa.y + 78, 74, 26), "CLOSE")

    sb = pygame.Rect(SP.x + 10, SP.y + 212, SP.w - 20, 170)
    pygame.draw.rect(surf, (9, 9, 9), sb, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, sb, width=1, border_radius=3)
    surf.blit(FONT_SM.render("Storage B", True, ORANGE_HI), (sb.x + 10, sb.y + 8))
    draw_servo_row(surf, sb.x + 12, sb.y + 40, servo_cfg[SCI_S3]["name"],
                   servo_angle[SCI_S3])
    draw_servo_row(surf, sb.x + 12, sb.y + 82, servo_cfg[SCI_S4]["name"],
                   servo_angle[SCI_S4])
    draw_button_rect(surf, pygame.Rect(sb.right - 248, sb.y + 36, 74, 26), "DRILL")
    draw_button_rect(surf, pygame.Rect(sb.right - 166, sb.y + 36, 74, 26), "BASE")
    draw_button_rect(surf, pygame.Rect(sb.right - 248, sb.y + 78, 74, 26), "OPEN")
    draw_button_rect(surf, pygame.Rect(sb.right - 166, sb.y + 78, 74, 26), "CLOSE")

    draw_button_rect(surf,
                     pygame.Rect(SP.right - 126, SP.bottom - 38, 112, 26),
                     "CENTER ALL", subtle=True)

    # Load cell + link
    LP = pygame.Rect(782, top + 296, 386, 308)
    draw_panel(surf, LP, "Load Cell and Link")
    draw_button_rect(surf, pygame.Rect(LP.x + 18, LP.y + 38, 110, 32), "READ")
    draw_button_rect(surf, pygame.Rect(LP.x + 136, LP.y + 38, 110, 32), "TARE")
    draw_button_rect(surf, pygame.Rect(LP.x + 254, LP.y + 38, 112, 32),
                     "E-STOP", subtle=True)

    load_txt = "---" if last_load is None else f"{last_load:.4f}"
    surf.blit(FONT_MD.render(f"Load raw: {load_txt}", True, ORANGE_HI),
              (LP.x + 18, LP.y + 88))
    if last_load_err:
        surf.blit(FONT_XS.render(last_load_err[:44], True, RED_ERR),
                  (LP.x + 18, LP.y + 114))
    if last_load is not None:
        load_history.append(last_load)
    graph = pygame.Rect(LP.x + 18, LP.y + 144, LP.w - 36, 98)
    pygame.draw.rect(surf, (8, 8, 8), graph, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, graph, width=1, border_radius=3)
    if len(load_history) > 3:
        mn = min(load_history)
        mx = max(load_history)
        span = (mx - mn) if (mx - mn) > 1e-9 else 1.0
        points = []
        for i, val in enumerate(load_history):
            x = graph.x + int(i * (graph.w - 4) / (len(load_history) - 1)) + 2
            y = graph.bottom - 2 - int(((val - mn) / span) * (graph.h - 6))
            points.append((x, y))
        if len(points) > 1:
            pygame.draw.lines(surf, ORANGE_HI, False, points, 2)

    nc = ORANGE_HI if not net_err else RED_ERR
    surf.blit(FONT_XS.render(f"UDP: {ROVER_IP}:{SCI_PORT}", True, nc),
              (LP.x + 18, LP.y + 252))
    surf.blit(FONT_XS.render(f"Packets sent: {packets_sent}", True, ORANGE_HI),
              (LP.x + 18, LP.y + 272))
    surf.blit(FONT_XS.render(
        f"Last ACK: {ack_status} seq={ack_seq}", True, ORANGE_MID),
        (LP.x + 18, LP.y + 290))


if __name__ == "__main__":
    main()
