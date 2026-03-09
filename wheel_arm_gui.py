"""
Logitech Extreme 3D Pro — Rover Arm Control + Monitor
pip install pygame
"""

import pygame
import sys
import socket
import time
import struct as _struct

# ── Network ─────────────────────────────────────────────────────────────────
ROVER_IP   = "192.168.10.177"
ROVER_PORT = 5760
WHEEL_PORT = 5761

WHEEL_SOF1    = 0xAA
WHEEL_SOF2    = 0xBB
DEADZONE      = 0.08
WHEEL_SEND_RATE = 0.05   # 20 Hz

SOF1 = 0xAA
SOF2 = 0xBB

CMD_MOTOR1_FWD   = 0x11
CMD_MOTOR1_REV   = 0x12
CMD_MOTOR1_STOP  = 0x13
CMD_MOTOR2_FWD   = 0x21
CMD_MOTOR2_REV   = 0x22
CMD_MOTOR2_STOP  = 0x23
CMD_MOTOR3_FWD   = 0x31
CMD_MOTOR3_REV   = 0x32
CMD_MOTOR3_STOP  = 0x33
CMD_MOTOR4A_FWD  = 0x41
CMD_MOTOR4A_REV  = 0x42
CMD_MOTOR4A_STOP = 0x43
CMD_MOTOR4B_FWD  = 0x51
CMD_MOTOR4B_REV  = 0x52
CMD_MOTOR4B_STOP = 0x53
CMD_SERVO_ANGLE  = 0x60

SERVO_MIN  = 0
SERVO_MAX  = 180
SERVO_STEP = 2
SEND_RATE  = 0.05

# ── Colour Palette — Black + Orange only ─────────────────────────────────────
BG          = (0,    0,    0)       # pure black
ORANGE_HI   = (255, 140,   0)       # bright orange — primary text / active
ORANGE_MID  = (200,  90,   0)       # medium orange — labels / bars
ORANGE_DIM  = (150,  75,   0)       # dim orange — borders / inactive
ORANGE_GLOW = (255, 180,  60)       # near-white orange — highlights
BORDER_LINE = (120,  55,   0)       # separator lines
RED_ERR     = (255,  50,  20)       # error / REV state

MOTOR_NAMES = ["M1 (HAT N/S)", "M2 (HAT W/E)", "M3 (B3/B4)",
               "M4A (B5/B6)", "M4B (B7/B8)"]

BUTTON_LABELS = [
    "TRIGGER","THUMB","BTN 3","BTN 4","BTN 5","BTN 6",
    "BTN 7","BTN 8","BTN 9","BTN 10","BTN 11","BTN 12"
]

# Base design canvas
BASE_W, BASE_H = 960, 700


# ── Helpers ───────────────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ── UDP senders ───────────────────────────────────────────────────────────────
class ArmSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ROVER_IP, ROVER_PORT)
        self.seq  = 0
        self.last_error = ""

    def send(self, cmd, val=0):
        body = bytes([SOF1, SOF2, (self.seq >> 8) & 0xFF, self.seq & 0xFF,
                      cmd, val & 0xFF])
        pkt  = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as e:
            self.last_error = str(e)
        self.seq = (self.seq + 1) & 0xFFFF


class WheelSender:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ROVER_IP, WHEEL_PORT)
        self.seq  = 0
        self.last_error = ""

    @staticmethod
    def _crc8(data):
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return crc

    def send_drive(self, x, z, throttle_pct):
        x_i8 = int(max(-127.0, min(127.0, x * 127.0)))
        z_i8 = int(max(-127.0, min(127.0, z * 127.0)))
        thr  = int(max(0, min(255, throttle_pct * 255.0)))
        x_byte = _struct.pack('b', x_i8)[0]
        z_byte = _struct.pack('b', z_i8)[0]
        body = bytes([WHEEL_SOF1, WHEEL_SOF2,
                      (self.seq >> 8) & 0xFF, self.seq & 0xFF,
                      x_byte, z_byte, thr])
        pkt = body + bytes([self._crc8(body)])
        try:
            self.sock.sendto(pkt, self.addr)
            self.last_error = ""
        except OSError as e:
            self.last_error = str(e)
        self.seq = (self.seq + 1) & 0xFFFF


# ── Draw helpers ──────────────────────────────────────────────────────────────
def draw_panel(surf, rect, title=None):
    """Flat black panel with orange border and optional title."""
    pygame.draw.rect(surf, (4, 4, 4), rect, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, rect, width=1, border_radius=3)
    if title:
        # title bar accent line
        pygame.draw.rect(surf, ORANGE_HI,
                         (rect.x + 10, rect.y + 10, 2, 14), border_radius=1)
        lbl = FONT_SM.render(title, True, ORANGE_HI)
        surf.blit(lbl, (rect.x + 18, rect.y + 10))


def draw_hline(surf, x, y, w):
    pygame.draw.line(surf, BORDER_LINE, (x, y), (x + w, y))


def draw_axis_bar(surf, x, y, w, h, value, label="", show_val=True):
    """Bipolar bar centred at 0. Orange fill, black bg."""
    pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)
    cx = x + w // 2
    pygame.draw.line(surf, ORANGE_DIM, (cx, y + 2), (cx, y + h - 2))

    pct  = (value + 1) / 2.0
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
    """Vertical throttle bar. value = raw axis (-1 full, +1 off)."""
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
    val = FONT_XS.render(f"{int(pct*100)}%", True, ORANGE_HI)
    surf.blit(val, (x + w // 2 - val.get_width() // 2, y + h + 4))


def draw_xy_pad(surf, x, y, size, ax, ay):
    """2-D joystick pad."""
    pygame.draw.rect(surf, (8, 8, 8), (x, y, size, size), border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, size, size), width=1, border_radius=3)
    cx, cy = x + size // 2, y + size // 2
    # grid lines
    for t in (0.25, 0.5, 0.75):
        gx = x + int(t * size); gy = y + int(t * size)
        pygame.draw.line(surf, (20, 8, 0), (gx, y + 1), (gx, y + size - 1))
        pygame.draw.line(surf, (20, 8, 0), (x + 1, gy), (x + size - 1, gy))
    # crosshair
    pygame.draw.line(surf, ORANGE_DIM, (cx, y + 2), (cx, y + size - 2))
    pygame.draw.line(surf, ORANGE_DIM, (x + 2, cy), (x + size - 2, cy))
    # dot
    dx = cx + int(ax * (size // 2 - 6))
    dy = cy + int(ay * (size // 2 - 6))
    pygame.draw.circle(surf, ORANGE_HI, (dx, dy), 6)
    pygame.draw.circle(surf, ORANGE_GLOW, (dx, dy), 2)
    surf.blit(FONT_XS.render("X/Y", True, ORANGE_DIM),
              (x + size // 2 - FONT_XS.size("X/Y")[0] // 2, y - 15))


def draw_button_widget(surf, x, y, w, h, pressed, label):
    bg  = ORANGE_DIM if pressed else (8, 8, 8)
    bc  = ORANGE_HI  if pressed else ORANGE_DIM
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
    dirs = {(0,0):"·",(0,-1):"N",(0,1):"S",(-1,0):"W",(1,0):"E",
            (-1,-1):"NW",(1,-1):"NE",(-1,1):"SW",(1,1):"SE"}
    cell = size // 3
    for row in range(3):
        for col in range(3):
            dx = col - 1; dy = row - 1
            is_center = (dx == 0 and dy == 0)
            # Note: hat_y > 0 = UP in pygame hat; we show N = up visually
            is_active = (dx, dy) == (active_dir[0], -active_dir[1]) and active_dir != (0,0)
            rx = cx - size // 2 + col * cell + 2
            ry = cy - size // 2 + row * cell + 2
            rw = cell - 4; rh = cell - 4
            bg = ORANGE_DIM if is_active else (8, 8, 8)
            bc = ORANGE_HI  if is_active else ORANGE_DIM
            pygame.draw.rect(surf, bg, (rx, ry, rw, rh), border_radius=2)
            pygame.draw.rect(surf, bc, (rx, ry, rw, rh), width=1, border_radius=2)
            if is_center:
                pygame.draw.circle(surf,
                    ORANGE_HI if active_dir != (0,0) else (30,12,0),
                    (rx + rw // 2, ry + rh // 2), 4)
    lbl = FONT_XS.render("POV HAT", True, ORANGE_DIM)
    surf.blit(lbl, (cx - lbl.get_width() // 2, cy - size // 2 - 16))
    dir_label = dirs.get((active_dir[0], active_dir[1]), "·")
    d = FONT_MD.render(dir_label, True, ORANGE_HI)
    surf.blit(d, (cx - d.get_width() // 2, cy + size // 2 + 4))


def draw_trigger(surf, cx, cy, pressed):
    c  = RED_ERR    if pressed else (8, 8, 8)
    bc = RED_ERR    if pressed else ORANGE_DIM
    pygame.draw.circle(surf, c,  (cx, cy), 24)
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
    pygame.draw.rect(surf, color,     (bx, 12, bw, 22), width=1, border_radius=2)
    pygame.draw.circle(surf, color, (bx + 10, 23), 3)
    surf.blit(badge, (bx + 18, 17))
    if name and connected:
        n = FONT_XS.render(name[:50], True, ORANGE_DIM)
        surf.blit(n, (bx, 38))


# ── Rover command panel ───────────────────────────────────────────────────────
def draw_rover_panel(surf, rect, motor_states, servo_angle, net_ok, packets_sent):
    draw_panel(surf, rect, "ARM / MOTOR COMMANDS")
    x0, y0 = rect.x + 14, rect.y + 36

    col_w = (rect.w - 28) // len(MOTOR_NAMES)
    for i, (name, state) in enumerate(zip(MOTOR_NAMES, motor_states)):
        cx = x0 + i * col_w + col_w // 2
        # label
        lbl = FONT_XS.render(name, True, ORANGE_DIM)
        surf.blit(lbl, (cx - lbl.get_width() // 2, y0))
        # state box
        sc = ORANGE_HI if state == "FWD" else (RED_ERR if state == "REV" else (25, 10, 0))
        bc = sc if state != "STOP" else ORANGE_DIM
        bx = cx - 24; by = y0 + 16
        pygame.draw.rect(surf, (6, 2, 0), (bx, by, 48, 16), border_radius=2)
        pygame.draw.rect(surf, bc,         (bx, by, 48, 16), width=1, border_radius=2)
        st = FONT_XS.render(state, True, sc)
        surf.blit(st, (cx - st.get_width() // 2, by + 2))

    # Servo bar
    sy = y0 + 48
    surf.blit(FONT_XS.render("SERVO ANGLE", True, ORANGE_DIM), (x0, sy))
    bar_x = x0 + 90; bar_w = rect.w - 110
    pygame.draw.rect(surf, (8, 8, 8), (bar_x, sy, bar_w, 12), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (bar_x, sy, bar_w, 12), width=1, border_radius=2)
    fill = int((servo_angle / 180) * (bar_w - 4))
    pygame.draw.rect(surf, ORANGE_MID, (bar_x + 2, sy + 2, fill, 8), border_radius=2)
    ang = FONT_XS.render(f"{servo_angle}°", True, ORANGE_HI)
    surf.blit(ang, (bar_x + bar_w + 6, sy))

    # Network line
    ny = sy + 22
    nc = ORANGE_HI if net_ok else RED_ERR
    surf.blit(FONT_XS.render(
        f"UDP  {ROVER_IP}:{ROVER_PORT}   pkts:{packets_sent}",
        True, nc), (x0, ny))


# ── Wheel drive panel ─────────────────────────────────────────────────────────
def draw_wheel_panel(surf, rect, x_val, z_val, throttle_pct, net_ok, pkts):
    draw_panel(surf, rect, "WHEEL DRIVE")
    x0, y0 = rect.x + 14, rect.y + 36
    bar_w  = rect.w - 110

    def row(label, value, y):
        surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x0, y))
        draw_axis_bar(surf, x0 + 95, y, bar_w, 12, value, show_val=True)

    row("FWD/REV",  x_val,    y0)
    row("TURN",     z_val,    y0 + 20)

    # Throttle
    ty = y0 + 40
    surf.blit(FONT_XS.render("THROTTLE", True, ORANGE_DIM), (x0, ty))
    bx = x0 + 95
    pygame.draw.rect(surf, (8, 8, 8),   (bx, ty, bar_w, 12), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM,  (bx, ty, bar_w, 12), width=1, border_radius=2)
    fill = int(throttle_pct * (bar_w - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (bx + 2, ty + 2, fill, 8), border_radius=2)
    surf.blit(FONT_XS.render(f"{int(throttle_pct*100)}%", True, ORANGE_HI),
              (bx + bar_w + 6, ty))

    # Differential
    lv = max(-1.0, min(1.0, x_val - z_val)) * throttle_pct
    rv = max(-1.0, min(1.0, x_val + z_val)) * throttle_pct
    row("LEFT  MTR", lv, ty + 20)
    row("RIGHT MTR", rv, ty + 40)

    # Net
    nc = ORANGE_HI if net_ok else RED_ERR
    surf.blit(FONT_XS.render(
        f"UDP  {ROVER_IP}:{WHEEL_PORT}   pkts:{pkts}", True, nc),
        (x0, ty + 62))


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global FONT_XS, FONT_SM, FONT_MD, FONT_LG

    pygame.init()

    # Start windowed, allow resize / maximise
    screen = pygame.display.set_mode((BASE_W, BASE_H),
                                     pygame.RESIZABLE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Extreme 3D Pro — Rover Arm Control")
    clock = pygame.time.Clock()

    # Fixed-resolution canvas we always draw to, then scale once
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
    # Allow joystick input even when window is not focused
    pygame.event.set_grab(False)
    pygame.joystick.init()
    joy = None

    def try_connect():
        nonlocal joy
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()

    try_connect()

    arm  = ArmSender()
    last_cmd: dict = {}
    servo_angle  = 90
    motor_states = ["STOP"] * 5
    packets_sent = 0
    last_send    = 0.0

    wheels = WheelSender()
    wheel_pkts_sent = 0
    last_wheel_send = 0.0
    wheel_x = wheel_z = wheel_thr = 0.0

    W, H = BASE_W, BASE_H   # current window size

    while True:
        now = time.monotonic()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit(); sys.exit()
            if event.type == pygame.JOYDEVICEADDED:
                try_connect()
            if event.type == pygame.JOYDEVICEREMOVED:
                joy = None
            if event.type == pygame.VIDEORESIZE:
                W, H = event.w, event.h
                screen = pygame.display.set_mode((W, H),
                    pygame.RESIZABLE | pygame.DOUBLEBUF)
            if event.type == pygame.WINDOWMAXIMIZED:
                W, H = screen.get_size()

        # Always sync current window size
        W, H = screen.get_size()

        # ── Joystick read — direct poll, works regardless of window focus ──
        connected = joy is not None and pygame.joystick.get_count() > 0
        if connected:
            try:
                pygame.event.pump()   # keep SDL internals alive without blocking
                num_axes    = joy.get_numaxes()
                num_buttons = joy.get_numbuttons()
                num_hats    = joy.get_numhats()
                axes    = [joy.get_axis(i)   for i in range(num_axes)]
                buttons = [joy.get_button(i) for i in range(num_buttons)]
                hats    = [joy.get_hat(i)    for i in range(num_hats)]
                ax_x = axes[0] if num_axes > 0 else 0.0
                ax_y = axes[1] if num_axes > 1 else 0.0
                ax_z = axes[2] if num_axes > 2 else 0.0
                ax_t = axes[3] if num_axes > 3 else -1.0
                hat_x, hat_y = hats[0] if num_hats > 0 else (0, 0)
                joy_name = joy.get_name()
            except Exception:
                connected = False

        if not connected:
            ax_x = ax_y = ax_z = 0.0; ax_t = -1.0
            buttons = [False] * 16
            hat_x = hat_y = 0
            joy_name = ""

        # ── Send arm commands ─────────────────────────────────────────────
        if connected and (now - last_send) >= SEND_RATE:
            last_send = now

            def maybe_send(key, cmd, val=255):
                nonlocal packets_sent
                if last_cmd.get(key) != cmd:
                    arm.send(cmd, val)
                    last_cmd[key] = cmd
                    packets_sent += 1

            if hat_y > 0:   c1 = CMD_MOTOR1_FWD;  motor_states[0] = "FWD"
            elif hat_y < 0: c1 = CMD_MOTOR1_REV;  motor_states[0] = "REV"
            else:           c1 = CMD_MOTOR1_STOP; motor_states[0] = "STOP"
            maybe_send("m1", c1)

            if hat_x > 0:   c2 = CMD_MOTOR2_FWD;  motor_states[1] = "FWD"
            elif hat_x < 0: c2 = CMD_MOTOR2_REV;  motor_states[1] = "REV"
            else:           c2 = CMD_MOTOR2_STOP; motor_states[1] = "STOP"
            maybe_send("m2", c2)

            if len(buttons) > 3:
                if buttons[2]:   c3 = CMD_MOTOR3_FWD;  motor_states[2] = "FWD"
                elif buttons[3]: c3 = CMD_MOTOR3_REV;  motor_states[2] = "REV"
                else:            c3 = CMD_MOTOR3_STOP; motor_states[2] = "STOP"
                maybe_send("m3", c3)

            if len(buttons) > 5:
                if buttons[4]:   c4 = CMD_MOTOR4A_FWD;  motor_states[3] = "FWD"
                elif buttons[5]: c4 = CMD_MOTOR4A_REV;  motor_states[3] = "REV"
                else:            c4 = CMD_MOTOR4A_STOP; motor_states[3] = "STOP"
                maybe_send("m4", c4)

            if len(buttons) > 7:
                if buttons[6]:   c5 = CMD_MOTOR4B_FWD;  motor_states[4] = "FWD"
                elif buttons[7]: c5 = CMD_MOTOR4B_REV;  motor_states[4] = "REV"
                else:            c5 = CMD_MOTOR4B_STOP; motor_states[4] = "STOP"
                maybe_send("m5", c5)

            if len(buttons) > 9:
                if buttons[8]:  servo_angle += SERVO_STEP
                if buttons[9]:  servo_angle -= SERVO_STEP
            servo_angle = max(SERVO_MIN, min(SERVO_MAX, servo_angle))
            arm.send(CMD_SERVO_ANGLE, servo_angle)
            packets_sent += 1

        # ── Send wheel commands ───────────────────────────────────────────
        if connected and (now - last_wheel_send) >= WHEEL_SEND_RATE:
            last_wheel_send = now
            raw_x = -ax_y
            raw_z =  ax_z
            wheel_x = raw_x if abs(raw_x) > DEADZONE else 0.0
            wheel_z = raw_z if abs(raw_z) > DEADZONE else 0.0
            wheel_thr = (1.0 - ax_t) / 2.0
            wheels.send_drive(wheel_x, wheel_z, wheel_thr)
            wheel_pkts_sent += 1

        # ─────────────── DRAW on fixed canvas ────────────────────────────
        canvas.fill(BG)

        # ── Header ───────────────────────────────────────────────────────
        title = FONT_LG.render("EXTREME 3D PRO  //  ROVER ARM CONTROL", True, ORANGE_HI)
        canvas.blit(title, (16, 12))
        draw_status_badge(canvas, connected, joy_name)
        draw_hline(canvas, 16, 44, BASE_W - 32)

        # ── LEFT PANEL: Axes ──────────────────────────────────────────────
        LP = pygame.Rect(14, 52, 440, 360)
        draw_panel(canvas, LP, "AXES & STICK")

        ay = LP.y + 40
        draw_axis_bar(canvas, LP.x+14, ay,     LP.w-28, 14, ax_x, "X AXIS   L/R")
        draw_axis_bar(canvas, LP.x+14, ay+42,  LP.w-28, 14, ax_y, "Y AXIS   FWD/BACK")
        draw_axis_bar(canvas, LP.x+14, ay+84,  LP.w-28, 14, ax_z, "Z TWIST  RUDDER")

        draw_xy_pad(canvas, LP.x + 14, ay + 126, 140, ax_x, ax_y)
        draw_throttle_v(canvas, LP.x + 172, ay + 126, 34, 140, ax_t)

        # Raw values strip
        raw_y = ay + 286
        rr = pygame.Rect(LP.x + 14, raw_y, LP.w - 28, 42)
        pygame.draw.rect(canvas, (6, 2, 0), rr, border_radius=2)
        pygame.draw.rect(canvas, ORANGE_DIM, rr, width=1, border_radius=2)
        canvas.blit(FONT_XS.render("RAW AXIS VALUES", True, ORANGE_MID),
                    (rr.x + 8, rr.y + 4))
        for i, (n, v) in enumerate(zip(["AX0","AX1","AX2","AX3"],
                                        [ax_x, ax_y, ax_z, ax_t])):
            rx = rr.x + 10 + i * (rr.w // 4)
            canvas.blit(FONT_XS.render(n, True, ORANGE_DIM), (rx, rr.y + 17))
            canvas.blit(FONT_XS.render(f"{v:+.4f}", True, ORANGE_HI), (rx, rr.y + 28))

        # ── RIGHT PANEL: Buttons ──────────────────────────────────────────
        RP = pygame.Rect(466, 52, 480, 360)
        draw_panel(canvas, RP, "BUTTONS")

        bw, bh = 98, 42
        bx0 = RP.x + 14; by0 = RP.y + 40
        for i in range(min(12, len(BUTTON_LABELS))):
            col = i % 4; row = i // 4
            bx = bx0 + col * (bw + 6)
            by = by0 + row * (bh + 8)
            pressed = buttons[i] if i < len(buttons) else False
            draw_button_widget(canvas, bx, by, bw, bh, pressed, BUTTON_LABELS[i])

        pov_cy = RP.y + 295
        draw_pov(canvas, RP.x + 110, pov_cy, 90, hat_x, hat_y)
        draw_trigger(canvas, RP.x + 340, pov_cy,
                     buttons[0] if len(buttons) > 0 else False)

        draw_hline(canvas, 14, 422, BASE_W - 28)

        # ── ROVER ARM PANEL ───────────────────────────────────────────────
        CP = pygame.Rect(14, 428, 930, 112)
        net_ok = not bool(arm.last_error)
        draw_rover_panel(canvas, CP, motor_states, servo_angle, net_ok, packets_sent)

        draw_hline(canvas, 14, 548, BASE_W - 28)

        # ── WHEEL DRIVE PANEL ─────────────────────────────────────────────
        WP = pygame.Rect(14, 554, 930, 118)
        wheel_net_ok = not bool(wheels.last_error)
        draw_wheel_panel(canvas, WP, wheel_x, wheel_z, wheel_thr,
                         wheel_net_ok, wheel_pkts_sent)

        # ── Footer ────────────────────────────────────────────────────────
        foot = FONT_XS.render(
            "ESC: quit   B9/B10: servo ▲▼   B3-B8: motors   HAT: motors 1/2",
            True, ORANGE_MID)
        canvas.blit(foot, (BASE_W // 2 - foot.get_width() // 2, BASE_H - 14))

        # ── Scale canvas to current window size (integer-aware) ───────────
        # Use smoothscale for non-integer scales but only if window differs
        if (W, H) != (BASE_W, BASE_H):
            # Maintain aspect ratio with letterboxing
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