#!/usr/bin/env python3
"""
Science GUI for Logitech Extreme 3D Pro.

Single-window interface for:
- 3 DC motors: Platform, Beam, Drill
- 4 servos for two storages
- Load cell read/tare + live telemetry display

Control inputs:
- Joystick (works even when GUI is not focused)
- Mouse buttons on GUI widgets
"""

import json
import math
import os
import socket
import sys
import time
from collections import deque

# Must be set before pygame import to keep joystick events while unfocused.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame

# Network
ROVER_IP = "192.168.10.177"  # Change to rover/science-receiver IP.
SCI_PORT = 5762

# Packet protocol
SOF1 = 0xAA
SOF2 = 0xBB
ACK_BYTE = 0xAC

SEND_RATE = 0.05
SERVO_MIN = 0
SERVO_MAX = 180
SERVO_STEP = 2

# Commands
CMD_M1_FWD = 0x11
CMD_M1_REV = 0x12
CMD_M1_STOP = 0x13
CMD_M2_FWD = 0x21
CMD_M2_REV = 0x22
CMD_M2_STOP = 0x23
CMD_M3_FWD = 0x31
CMD_M3_REV = 0x32
CMD_M3_STOP = 0x33

CMD_S1 = 0x41
CMD_S2 = 0x42
CMD_S3 = 0x43
CMD_S4 = 0x44
CMD_SERVO_CENTER = 0x45

CMD_LOAD_READ = 0x51
CMD_LOAD_TARE = 0x52
CMD_ESTOP = 0xFF

STATUS_NAMES = {
    0x00: "OK",
    0x01: "CRC_ERR",
    0x02: "UNK_CMD",
    0x03: "ESTOP",
}

# Visuals
BG = (0, 0, 0)
ORANGE_HI = (255, 140, 0)
ORANGE_MID = (205, 95, 0)
ORANGE_DIM = (140, 68, 0)
ORANGE_GLOW = (255, 185, 60)
RED_ERR = (255, 70, 30)

BASE_W, BASE_H = 1180, 780


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class ScienceSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.addr = (ROVER_IP, SCI_PORT)
        self.seq = 0
        self.last_error = ""
        self.last_ack = "-"
        self.last_ack_seq = -1
        self.last_load = None
        self.last_load_err = ""

    def send(self, cmd: int, val: int = 0):
        body = bytes([
            SOF1,
            SOF2,
            (self.seq >> 8) & 0xFF,
            self.seq & 0xFF,
            cmd & 0xFF,
            val & 0xFF,
        ])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as exc:
            self.last_error = str(exc)
        sent_seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return pkt, sent_seq

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
                seq = (data[1] << 8) | data[2]
                st = data[3]
                self.last_ack_seq = seq
                self.last_ack = STATUS_NAMES.get(st, hex(st))
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


def clamp(val, lo, hi):
    return lo if val < lo else hi if val > hi else val


def draw_panel(surf, rect, title):
    pygame.draw.rect(surf, (6, 6, 6), rect, border_radius=4)
    pygame.draw.rect(surf, ORANGE_DIM, rect, width=1, border_radius=4)
    if title:
        t = FONT_SM.render(title, True, ORANGE_HI)
        surf.blit(t, (rect.x + 10, rect.y + 8))


def draw_button(surf, rect, label, active=False, subtle=False):
    bg = (10, 10, 10) if subtle else (20, 8, 0)
    if active:
        bg = ORANGE_DIM
    border = ORANGE_HI if active else ORANGE_DIM
    pygame.draw.rect(surf, bg, rect, border_radius=4)
    pygame.draw.rect(surf, border, rect, width=1, border_radius=4)
    txt = FONT_XS.render(label, True, ORANGE_HI)
    surf.blit(txt, (rect.centerx - txt.get_width() // 2, rect.centery - txt.get_height() // 2))


def draw_axis_bar(surf, x, y, w, h, value, label):
    pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=3)
    cx = x + w // 2
    pygame.draw.line(surf, ORANGE_DIM, (cx, y + 2), (cx, y + h - 2))

    pct = (value + 1.0) / 2.0
    if value >= 0:
        fill_x = cx
        fill_w = int((pct - 0.5) * w)
    else:
        fill_x = x + int(pct * w)
        fill_w = cx - fill_x
    if fill_w > 0:
        pygame.draw.rect(surf, ORANGE_MID, (fill_x, y + 2, fill_w, h - 4), border_radius=3)

    l = FONT_XS.render(label, True, ORANGE_DIM)
    v = FONT_XS.render(f"{value:+.3f}", True, ORANGE_HI)
    surf.blit(l, (x, y - 16))
    surf.blit(v, (x + w - v.get_width(), y - 16))


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
    pygame.draw.circle(surf, ORANGE_HI if state != 0 else ORANGE_DIM, (cx, cy), radius, width=2)

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
    l = FONT_XS.render(label, True, ORANGE_DIM)
    surf.blit(l, (x, y))

    bx, by, bw, bh = x + 168, y + 2, 170, 12
    pygame.draw.rect(surf, (8, 8, 8), (bx, by, bw, bh), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (bx, by, bw, bh), width=1, border_radius=2)
    fill = int((angle / 180.0) * (bw - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (bx + 2, by + 2, fill, bh - 4), border_radius=2)

    a = FONT_XS.render(f"{angle:3d} deg", True, ORANGE_HI)
    surf.blit(a, (bx + bw + 8, y - 1))


def main():
    global FONT_XS, FONT_SM, FONT_MD, FONT_LG

    pygame.init()
    pygame.joystick.init()

    screen = pygame.display.set_mode((BASE_W, BASE_H), pygame.RESIZABLE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Science Control - Extreme 3D Pro")
    canvas = pygame.Surface((BASE_W, BASE_H))
    clock = pygame.time.Clock()

    FONT_XS = pygame.font.SysFont("couriernew", 13, bold=True)
    FONT_SM = pygame.font.SysFont("couriernew", 15, bold=True)
    FONT_MD = pygame.font.SysFont("couriernew", 19, bold=True)
    FONT_LG = pygame.font.SysFont("couriernew", 26, bold=True)

    pygame.event.set_grab(False)

    joy = None

    def try_connect_joy():
        nonlocal joy
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            print(f"[SCI] Joystick connected: {joy.get_name()}")

    try_connect_joy()

    sender = ScienceSender()

    # DC motor state: -1 reverse/down/release, 0 stop, +1 forward/up/collect
    dc_state = {
        "platform": 0,
        "beam": 0,
        "drill": 0,
    }

    # Named servos for two storage bins
    servo_cfg = {
        CMD_S1: {"name": "Storage A Carriage", "drill": 25, "base": 155},
        CMD_S2: {"name": "Storage A Lid", "open": 150, "close": 35},
        CMD_S3: {"name": "Storage B Carriage", "drill": 25, "base": 155},
        CMD_S4: {"name": "Storage B Lid", "open": 150, "close": 35},
    }
    servo_angle = {CMD_S1: 90, CMD_S2: 90, CMD_S3: 90, CMD_S4: 90}
    servo_dirty = {CMD_S1: True, CMD_S2: True, CMD_S3: True, CMD_S4: True}

    load_history = deque(maxlen=140)

    last_send = 0.0
    packets_sent = 0
    last_cmd = {}
    prev_buttons = [0] * 16

    # Visual animation state
    platform_pos = 0.0
    beam_pos = 0.0
    drill_phase = 0.0

    W, H = BASE_W, BASE_H

    def window_to_canvas(pt):
        """Convert window-space point to fixed-canvas space."""
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
        cx = int((wx - ox) / scale)
        cy = int((wy - oy) / scale)
        return cx, cy

    while True:
        now = time.monotonic()
        dt = clock.get_time() / 1000.0

        mouse_down = pygame.mouse.get_pressed(3)[0]
        mouse_pos = window_to_canvas(pygame.mouse.get_pos())

        # Keep event queue pumped, then process app-level events.
        for ev in pygame.event.get([
            pygame.JOYAXISMOTION,
            pygame.JOYBALLMOTION,
            pygame.JOYHATMOTION,
            pygame.JOYBUTTONDOWN,
            pygame.JOYBUTTONUP,
        ]):
            _ = ev

        click_pos = None
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()
            if event.type == pygame.JOYDEVICEADDED:
                try_connect_joy()
            if event.type == pygame.JOYDEVICEREMOVED:
                joy = None
            if event.type == pygame.VIDEORESIZE:
                W, H = event.w, event.h
                screen = pygame.display.set_mode((W, H), pygame.RESIZABLE | pygame.DOUBLEBUF)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                click_pos = window_to_canvas(event.pos)

        connected = joy is not None and pygame.joystick.get_count() > 0
        if connected:
            try:
                n_axes = joy.get_numaxes()
                n_btn = joy.get_numbuttons()
                n_hat = joy.get_numhats()
                axes = [joy.get_axis(i) for i in range(n_axes)]
                buttons = [joy.get_button(i) for i in range(n_btn)]
                hats = [joy.get_hat(i) for i in range(n_hat)]
                hat_x, hat_y = hats[0] if n_hat > 0 else (0, 0)
                ax_x = axes[0] if n_axes > 0 else 0.0
                ax_y = axes[1] if n_axes > 1 else 0.0
                ax_z = axes[2] if n_axes > 2 else 0.0
                joy_name = joy.get_name()
            except Exception:
                connected = False
        if not connected:
            buttons = [0] * 16
            hat_x = hat_y = 0
            ax_x = ax_y = ax_z = 0.0
            joy_name = ""

        # Base control from joystick (same feel as current GUI style)
        dc_state["platform"] = 1 if hat_y > 0 else -1 if hat_y < 0 else 0
        dc_state["beam"] = 1 if hat_x > 0 else -1 if hat_x < 0 else 0
        dc_state["drill"] = 1 if len(buttons) > 7 and buttons[7] else -1 if len(buttons) > 6 and buttons[6] else 0

        # Joystick servo nudges
        if len(buttons) > 4:
            if buttons[4]:
                servo_angle[CMD_S1] = clamp(servo_angle[CMD_S1] + SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S1] = True
            if buttons[2]:
                servo_angle[CMD_S1] = clamp(servo_angle[CMD_S1] - SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S1] = True
            if buttons[5]:
                servo_angle[CMD_S2] = clamp(servo_angle[CMD_S2] + SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S2] = True
            if buttons[3]:
                servo_angle[CMD_S2] = clamp(servo_angle[CMD_S2] - SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S2] = True

        if len(buttons) > 9:
            if buttons[9]:
                servo_angle[CMD_S3] = clamp(servo_angle[CMD_S3] + SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S3] = True
            if buttons[8]:
                servo_angle[CMD_S3] = clamp(servo_angle[CMD_S3] - SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S3] = True

        if len(buttons) > 1:
            if buttons[0]:
                servo_angle[CMD_S4] = clamp(servo_angle[CMD_S4] + SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S4] = True
            if buttons[1]:
                servo_angle[CMD_S4] = clamp(servo_angle[CMD_S4] - SERVO_STEP, SERVO_MIN, SERVO_MAX)
                servo_dirty[CMD_S4] = True

        # Joystick load cell action on edge
        if len(buttons) > 11:
            if buttons[10] and not prev_buttons[10]:
                sender.send(CMD_LOAD_READ, 0)
                packets_sent += 1
            if buttons[11] and not prev_buttons[11]:
                sender.send(CMD_LOAD_TARE, 0)
                packets_sent += 1
        prev_buttons = buttons + [0] * max(0, 16 - len(buttons))

        # Draw layout
        canvas.fill(BG)
        title = FONT_LG.render("EXTREME 3D PRO // SCIENCE MODULE CONTROL", True, ORANGE_HI)
        canvas.blit(title, (16, 10))

        status = "CONNECTED" if connected else "NO CONTROLLER"
        sc = ORANGE_HI if connected else RED_ERR
        st = FONT_XS.render(f"CTRL: {status}", True, sc)
        canvas.blit(st, (BASE_W - 210, 16))
        if joy_name:
            jn = FONT_XS.render(joy_name[:34], True, ORANGE_DIM)
            canvas.blit(jn, (BASE_W - 330, 34))

        # Input monitor
        p_in = pygame.Rect(12, 58, 360, 198)
        draw_panel(canvas, p_in, "Joystick Monitor")
        draw_axis_bar(canvas, p_in.x + 12, p_in.y + 46, p_in.w - 24, 14, ax_x, "Axis X")
        draw_axis_bar(canvas, p_in.x + 12, p_in.y + 82, p_in.w - 24, 14, ax_y, "Axis Y")
        draw_axis_bar(canvas, p_in.x + 12, p_in.y + 118, p_in.w - 24, 14, ax_z, "Axis Z Twist")
        hats_txt = FONT_XS.render(f"HAT: ({hat_x:+d}, {hat_y:+d})", True, ORANGE_HI)
        canvas.blit(hats_txt, (p_in.x + 14, p_in.bottom - 30))

        # DC section
        p_dc = pygame.Rect(382, 58, 786, 280)
        draw_panel(canvas, p_dc, "DC Motors (Platform / Beam / Drill)")

        motor_buttons = {}

        col_w = 244
        # Platform
        r1 = pygame.Rect(p_dc.x + 12, p_dc.y + 36, col_w, 232)
        draw_lift_visual(canvas, r1, "Platform", platform_pos, dc_state["platform"])
        b_up = pygame.Rect(r1.x + 20, r1.y + 56, 86, 30)
        b_dn = pygame.Rect(r1.x + 20, r1.y + 94, 86, 30)
        draw_button(canvas, b_up, "UP", dc_state["platform"] > 0)
        draw_button(canvas, b_dn, "DOWN", dc_state["platform"] < 0)
        motor_buttons[("platform", 1)] = b_up
        motor_buttons[("platform", -1)] = b_dn

        # Beam
        r2 = pygame.Rect(p_dc.x + 270, p_dc.y + 36, col_w, 232)
        draw_lift_visual(canvas, r2, "Beam", beam_pos, dc_state["beam"])
        b_up = pygame.Rect(r2.x + 20, r2.y + 56, 86, 30)
        b_dn = pygame.Rect(r2.x + 20, r2.y + 94, 86, 30)
        draw_button(canvas, b_up, "UP", dc_state["beam"] > 0)
        draw_button(canvas, b_dn, "DOWN", dc_state["beam"] < 0)
        motor_buttons[("beam", 1)] = b_up
        motor_buttons[("beam", -1)] = b_dn

        # Drill
        r3 = pygame.Rect(p_dc.x + 528, p_dc.y + 36, col_w, 232)
        draw_drill_visual(canvas, r3, dc_state["drill"], drill_phase)
        b_cl = pygame.Rect(r3.x + 20, r3.y + 56, 100, 30)
        b_rl = pygame.Rect(r3.x + 20, r3.y + 94, 100, 30)
        draw_button(canvas, b_cl, "COLLECT", dc_state["drill"] > 0)
        draw_button(canvas, b_rl, "RELEASE", dc_state["drill"] < 0)
        motor_buttons[("drill", 1)] = b_cl
        motor_buttons[("drill", -1)] = b_rl

        # Mouse hold motor override (works alongside joystick)
        if mouse_down:
            for (mname, direction), rect in motor_buttons.items():
                if rect.collidepoint(mouse_pos):
                    dc_state[mname] = direction

        # Servo + storage section (single window, no tabs)
        p_sv = pygame.Rect(12, 266, 760, 390)
        draw_panel(canvas, p_sv, "Storage Servos (No Tabs)")

        click_buttons = {}

        # Storage A block
        sa = pygame.Rect(p_sv.x + 10, p_sv.y + 32, p_sv.w - 20, 170)
        pygame.draw.rect(canvas, (9, 9, 9), sa, border_radius=3)
        pygame.draw.rect(canvas, ORANGE_DIM, sa, width=1, border_radius=3)
        sa_t = FONT_SM.render("Storage A", True, ORANGE_HI)
        canvas.blit(sa_t, (sa.x + 10, sa.y + 8))

        draw_servo_row(canvas, sa.x + 12, sa.y + 40, servo_cfg[CMD_S1]["name"], servo_angle[CMD_S1])
        draw_servo_row(canvas, sa.x + 12, sa.y + 82, servo_cfg[CMD_S2]["name"], servo_angle[CMD_S2])

        s1_drill = pygame.Rect(sa.right - 248, sa.y + 36, 74, 26)
        s1_base = pygame.Rect(sa.right - 166, sa.y + 36, 74, 26)
        s2_open = pygame.Rect(sa.right - 248, sa.y + 78, 74, 26)
        s2_close = pygame.Rect(sa.right - 166, sa.y + 78, 74, 26)
        draw_button(canvas, s1_drill, "DRILL")
        draw_button(canvas, s1_base, "BASE")
        draw_button(canvas, s2_open, "OPEN")
        draw_button(canvas, s2_close, "CLOSE")
        click_buttons["s1_drill"] = s1_drill
        click_buttons["s1_base"] = s1_base
        click_buttons["s2_open"] = s2_open
        click_buttons["s2_close"] = s2_close

        # Storage B block
        sb = pygame.Rect(p_sv.x + 10, p_sv.y + 212, p_sv.w - 20, 170)
        pygame.draw.rect(canvas, (9, 9, 9), sb, border_radius=3)
        pygame.draw.rect(canvas, ORANGE_DIM, sb, width=1, border_radius=3)
        sb_t = FONT_SM.render("Storage B", True, ORANGE_HI)
        canvas.blit(sb_t, (sb.x + 10, sb.y + 8))

        draw_servo_row(canvas, sb.x + 12, sb.y + 40, servo_cfg[CMD_S3]["name"], servo_angle[CMD_S3])
        draw_servo_row(canvas, sb.x + 12, sb.y + 82, servo_cfg[CMD_S4]["name"], servo_angle[CMD_S4])

        s3_drill = pygame.Rect(sb.right - 248, sb.y + 36, 74, 26)
        s3_base = pygame.Rect(sb.right - 166, sb.y + 36, 74, 26)
        s4_open = pygame.Rect(sb.right - 248, sb.y + 78, 74, 26)
        s4_close = pygame.Rect(sb.right - 166, sb.y + 78, 74, 26)
        draw_button(canvas, s3_drill, "DRILL")
        draw_button(canvas, s3_base, "BASE")
        draw_button(canvas, s4_open, "OPEN")
        draw_button(canvas, s4_close, "CLOSE")
        click_buttons["s3_drill"] = s3_drill
        click_buttons["s3_base"] = s3_base
        click_buttons["s4_open"] = s4_open
        click_buttons["s4_close"] = s4_close

        b_center = pygame.Rect(p_sv.right - 126, p_sv.bottom - 38, 112, 26)
        draw_button(canvas, b_center, "CENTER ALL", subtle=True)
        click_buttons["center"] = b_center

        # Load cell + link status section
        p_load = pygame.Rect(782, 348, 386, 308)
        draw_panel(canvas, p_load, "Load Cell and Link")

        b_read = pygame.Rect(p_load.x + 18, p_load.y + 38, 110, 32)
        b_tare = pygame.Rect(p_load.x + 136, p_load.y + 38, 110, 32)
        b_estop = pygame.Rect(p_load.x + 254, p_load.y + 38, 112, 32)
        draw_button(canvas, b_read, "READ")
        draw_button(canvas, b_tare, "TARE")
        draw_button(canvas, b_estop, "E-STOP", subtle=True)
        click_buttons["load_read"] = b_read
        click_buttons["load_tare"] = b_tare
        click_buttons["estop"] = b_estop

        load_txt = "---" if sender.last_load is None else f"{sender.last_load:.4f}"
        lv = FONT_MD.render(f"Load raw: {load_txt}", True, ORANGE_HI)
        canvas.blit(lv, (p_load.x + 18, p_load.y + 88))
        if sender.last_load_err:
            err = FONT_XS.render(sender.last_load_err[:44], True, RED_ERR)
            canvas.blit(err, (p_load.x + 18, p_load.y + 114))

        if sender.last_load is not None:
            load_history.append(sender.last_load)

        # Sparkline visualization
        graph = pygame.Rect(p_load.x + 18, p_load.y + 144, p_load.w - 36, 98)
        pygame.draw.rect(canvas, (8, 8, 8), graph, border_radius=3)
        pygame.draw.rect(canvas, ORANGE_DIM, graph, width=1, border_radius=3)
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
                pygame.draw.lines(canvas, ORANGE_HI, False, points, 2)

        net = ORANGE_HI if not sender.last_error else RED_ERR
        n1 = FONT_XS.render(f"UDP: {ROVER_IP}:{SCI_PORT}", True, net)
        n2 = FONT_XS.render(f"Packets sent: {packets_sent}", True, ORANGE_HI)
        n3 = FONT_XS.render(f"Last ACK: {sender.last_ack} seq={sender.last_ack_seq}", True, ORANGE_MID)
        canvas.blit(n1, (p_load.x + 18, p_load.y + 252))
        canvas.blit(n2, (p_load.x + 18, p_load.y + 272))
        canvas.blit(n3, (p_load.x + 18, p_load.y + 290))

        # Mouse click actions (one-shot)
        if click_pos is not None:
            for key, rect in click_buttons.items():
                if rect.collidepoint(click_pos):
                    if key == "s1_drill":
                        servo_angle[CMD_S1] = servo_cfg[CMD_S1]["drill"]
                        servo_dirty[CMD_S1] = True
                    elif key == "s1_base":
                        servo_angle[CMD_S1] = servo_cfg[CMD_S1]["base"]
                        servo_dirty[CMD_S1] = True
                    elif key == "s2_open":
                        servo_angle[CMD_S2] = servo_cfg[CMD_S2]["open"]
                        servo_dirty[CMD_S2] = True
                    elif key == "s2_close":
                        servo_angle[CMD_S2] = servo_cfg[CMD_S2]["close"]
                        servo_dirty[CMD_S2] = True
                    elif key == "s3_drill":
                        servo_angle[CMD_S3] = servo_cfg[CMD_S3]["drill"]
                        servo_dirty[CMD_S3] = True
                    elif key == "s3_base":
                        servo_angle[CMD_S3] = servo_cfg[CMD_S3]["base"]
                        servo_dirty[CMD_S3] = True
                    elif key == "s4_open":
                        servo_angle[CMD_S4] = servo_cfg[CMD_S4]["open"]
                        servo_dirty[CMD_S4] = True
                    elif key == "s4_close":
                        servo_angle[CMD_S4] = servo_cfg[CMD_S4]["close"]
                        servo_dirty[CMD_S4] = True
                    elif key == "center":
                        sender.send(CMD_SERVO_CENTER, 90)
                        packets_sent += 1
                        for scmd in servo_angle:
                            servo_angle[scmd] = 90
                            servo_dirty[scmd] = False
                    elif key == "load_read":
                        sender.send(CMD_LOAD_READ, 0)
                        packets_sent += 1
                    elif key == "load_tare":
                        sender.send(CMD_LOAD_TARE, 0)
                        packets_sent += 1
                    elif key == "estop":
                        sender.send(CMD_ESTOP, 0)
                        packets_sent += 1

        # Command send loop
        if now - last_send >= SEND_RATE:
            last_send = now

            # Map motor states to commands with dedupe
            motor_cmds = {
                "platform": (CMD_M1_FWD, CMD_M1_REV, CMD_M1_STOP),
                "beam": (CMD_M2_FWD, CMD_M2_REV, CMD_M2_STOP),
                "drill": (CMD_M3_FWD, CMD_M3_REV, CMD_M3_STOP),
            }
            for name, state in dc_state.items():
                fwd, rev, stp = motor_cmds[name]
                cmd = fwd if state > 0 else rev if state < 0 else stp
                if last_cmd.get(name) != cmd:
                    sender.send(cmd, 220 if name == "drill" and state != 0 else 255)
                    packets_sent += 1
                    last_cmd[name] = cmd

            # Send servo angles when changed.
            for scmd, is_dirty in servo_dirty.items():
                if is_dirty:
                    sender.send(scmd, servo_angle[scmd])
                    packets_sent += 1
                    servo_dirty[scmd] = False

            sender.poll_feedback()

        # Keep simple animation in sync with command states
        platform_pos = clamp(platform_pos + dc_state["platform"] * dt * 0.8, -1.0, 1.0)
        beam_pos = clamp(beam_pos + dc_state["beam"] * dt * 0.8, -1.0, 1.0)
        drill_phase += dc_state["drill"] * dt * 6.5

        footer = (
            "HAT N/S: Platform  HAT E/W: Beam  B8/B7: Drill Collect/Release  "
            "B5/B3: S1 +/-  B6/B4: S2 +/-  B10/B9: S3 +/-  Trigger/Thumb: S4 +/-  B11: Load Read  B12: Tare"
        )
        f = FONT_XS.render(footer, True, ORANGE_MID)
        canvas.blit(f, (BASE_W // 2 - f.get_width() // 2, BASE_H - 18))

        # Scale for resizable window
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


if __name__ == "__main__":
    main()
