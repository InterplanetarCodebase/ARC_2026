#!/usr/bin/env python3
"""
gui_node.py — Base Station Teleop GUI (ROS2 Humble, ament_python)

Publishes:
  /cmd_vel   geometry_msgs/Twist        @ 20 Hz  QoS: Best Effort
  /arm_cmd   std_msgs/Int16MultiArray   @ 20 Hz  QoS: Reliable

  arm_cmd layout: [m1, m2, m3, m4a, m4b, servo_angle, motor_speed]

Mouse controls (when no joystick connected):
  XY pad        — click + drag  → fwd/rev + turn  (springs to centre on release)
  Throttle bar  — click + drag  OR scroll wheel   → speed (persists on release)
  B3–B10 btns   — hold mouse    → motor active, release → stop
  POV cells     — hold mouse    → base / shoulder active, release → stop
"""

import os, sys, time
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
import pygame

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

# ── ARM CMD constants (match ESP32 firmware) ──────────────────────────────────
CMD_MOTOR1_FWD,  CMD_MOTOR1_REV,  CMD_MOTOR1_STOP  = 0x11, 0x12, 0x13
CMD_MOTOR2_FWD,  CMD_MOTOR2_REV,  CMD_MOTOR2_STOP  = 0x21, 0x22, 0x23
CMD_MOTOR3_FWD,  CMD_MOTOR3_REV,  CMD_MOTOR3_STOP  = 0x31, 0x32, 0x33
CMD_MOTOR4A_FWD, CMD_MOTOR4A_REV, CMD_MOTOR4A_STOP = 0x41, 0x42, 0x43
CMD_MOTOR4B_FWD, CMD_MOTOR4B_REV, CMD_MOTOR4B_STOP = 0x51, 0x52, 0x53

SERVO_MIN, SERVO_MAX, SERVO_STEP = 0, 180, 2
DEADZONE        = 0.08
SEND_RATE       = 0.05   # 20 Hz
WHEEL_SEND_RATE = 0.05

# ── Motor display metadata ────────────────────────────────────────────────────
MOTOR_NAMES = ["Base", "Shoulder", "Elbow", "Roller", "Gripper"]
MOTOR_DIR_LABELS = {
    "Base":     ("CW",   "CCW"),
    "Shoulder": ("Up",   "Down"),
    "Elbow":    ("Up",   "Down"),
    "Roller":   ("CW",   "CCW"),
    "Gripper":  ("Open", "Close"),
}
MOTOR_VERBOSE = {
    "Base":     ("Base CW",         "Base CCW"),
    "Shoulder": ("Shoulder Up",     "Shoulder Down"),
    "Elbow":    ("Elbow Up",        "Elbow Down"),
    "Roller":   ("Roller CW",       "Roller CCW"),
    "Gripper":  ("Gripper Opening", "Gripper Closing"),
}

# ── Palette ───────────────────────────────────────────────────────────────────
BG          = (  0,   0,   0)
ORANGE_HI   = (255, 140,   0)
ORANGE_MID  = (200,  90,   0)
ORANGE_DIM  = (150,  75,   0)
ORANGE_GLOW = (255, 180,  60)
BORDER_LINE = (120,  55,   0)
RED_ERR     = (255,  50,  20)
GREEN_OK    = ( 80, 200,  60)
BLUE_MOUSE  = ( 60, 160, 255)   # interactive highlight in mouse mode

BUTTON_LABELS = [
    "TRIGGER", "THUMB",
    "B3 SRV↓", "B4 ELB↓",
    "B5 SRV↑", "B6 ELB↑",
    "B7 ROL↺", "B8 ROL↻",
    "B9 GRP✕", "B10 GRP○",
    "BTN 11",  "BTN 12",
]

BASE_W, BASE_H = 960, 760
FONT_XS = FONT_SM = FONT_MD = FONT_LG = None

# ── Fixed canvas geometry (must match draw calls below) ──────────────────────
# Left panel origin
_LP_X, _LP_Y = 14, 52
_AY           = _LP_Y + 40          # ay inside left panel = 92

# XY pad & throttle (drawn at LP.x+14 / LP.x+172 from ay+126)
XY_X,  XY_Y,  XY_SIZE         = _LP_X + 14,  _AY + 126, 140
THR_X, THR_Y, THR_W, THR_H    = _LP_X + 172, _AY + 126, 34, 140

XY_CX = XY_X + XY_SIZE // 2   # 98
XY_CY = XY_Y + XY_SIZE // 2   # 288

# Right panel buttons (bx0=RP.x+14=480, by0=RP.y+40=92, w=98, h=42)
_RP_X, _RP_Y   = 466, 52
BTN_X0, BTN_Y0 = _RP_X + 14, _RP_Y + 40
BTN_W,  BTN_H  = 98, 42

# POV hat centre
POV_CX, POV_CY, POV_SIZE = _RP_X + 110, _RP_Y + 295, 90


# ── Widget geometry helpers ───────────────────────────────────────────────────
def btn_rect(i: int) -> pygame.Rect:
    col, row = i % 4, i // 4
    return pygame.Rect(BTN_X0 + col * (BTN_W + 6),
                       BTN_Y0 + row * (BTN_H + 8),
                       BTN_W, BTN_H)

def pov_cell_rect(col: int, row: int) -> pygame.Rect:
    cell = POV_SIZE // 3
    return pygame.Rect(POV_CX - POV_SIZE // 2 + col * cell + 2,
                       POV_CY - POV_SIZE // 2 + row * cell + 2,
                       cell - 4, cell - 4)

def canvas_pos(screen_pos, W: int, H: int):
    """Map a screen (window) coordinate back to fixed 960×760 canvas coordinates."""
    if W == BASE_W and H == BASE_H:
        return screen_pos
    scale = min(W / BASE_W, H / BASE_H)
    ow = int(BASE_W * scale); oh = int(BASE_H * scale)
    ox = (W - ow) // 2;       oy = (H - oh) // 2
    return ((screen_pos[0] - ox) / scale,
            (screen_pos[1] - oy) / scale)


# ── Virtual (mouse-driven) input ──────────────────────────────────────────────
class VirtualInput:
    """
    Mirrors joystick state, driven entirely by mouse events.
    Conventions match the real joystick:
      ax_y  : -1 = full forward,  +1 = full back
      ax_z  : -1 = full left,     +1 = full right
      ax_t  : -1 = 100% throttle, +1 = 0% throttle  (default 0.0 = 50%)
    """
    def __init__(self):
        self.ax_y    = 0.0
        self.ax_z    = 0.0
        self.ax_t    = 0.0          # 50% throttle default
        self.hat_x   = 0
        self.hat_y   = 0
        self.buttons = [False] * 16

        self._xy_down  = False
        self._thr_down = False
        self._held_btn = None
        self._held_pov = False

    @property
    def throttle_pct(self):
        return (1.0 - self.ax_t) / 2.0

    # ── Mouse event handlers ──────────────────────────────────────────────
    def on_down(self, cx, cy):
        if pygame.Rect(XY_X, XY_Y, XY_SIZE, XY_SIZE).collidepoint(cx, cy):
            self._xy_down = True
            self._update_xy(cx, cy)
            return
        if pygame.Rect(THR_X, THR_Y, THR_W, THR_H).collidepoint(cx, cy):
            self._thr_down = True
            self._update_thr(cy)
            return
        for i in range(12):
            if btn_rect(i).collidepoint(cx, cy):
                self._held_btn = i
                self.buttons[i] = True
                return
        for row in range(3):
            for col in range(3):
                if pov_cell_rect(col, row).collidepoint(cx, cy):
                    self.hat_x =  col - 1
                    self.hat_y = -(row - 1)   # pygame hat: +y = up
                    self._held_pov = True
                    return

    def on_up(self):
        # XY springs back to centre on release
        self._xy_down = False
        self.ax_y = 0.0
        self.ax_z = 0.0
        # throttle persists
        self._thr_down = False
        if self._held_btn is not None:
            self.buttons[self._held_btn] = False
            self._held_btn = None
        if self._held_pov:
            self.hat_x = 0
            self.hat_y = 0
            self._held_pov = False

    def on_motion(self, cx, cy):
        if self._xy_down:  self._update_xy(cx, cy)
        if self._thr_down: self._update_thr(cy)

    def on_scroll(self, cx, cy, dy):
        """Scroll wheel nudges throttle when cursor is anywhere in the left panel."""
        if pygame.Rect(_LP_X, _LP_Y, 440, 360).collidepoint(cx, cy):
            pct = max(0.0, min(1.0, self.throttle_pct + dy * 0.05))
            self.ax_t = 1.0 - 2.0 * pct

    def _update_xy(self, cx, cy):
        half = XY_SIZE // 2 - 6
        self.ax_z = max(-1.0, min(1.0, (cx - XY_CX) / half))
        self.ax_y = max(-1.0, min(1.0, (cy - XY_CY) / half))

    def _update_thr(self, cy):
        pct = 1.0 - (cy - (THR_Y + 2)) / (THR_H - 4)
        self.ax_t = 1.0 - 2.0 * max(0.0, min(1.0, pct))


# ── Draw helpers ──────────────────────────────────────────────────────────────
def draw_panel(surf, rect, title=None):
    pygame.draw.rect(surf, (4, 4, 4), rect, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, rect, width=1, border_radius=3)
    if title:
        pygame.draw.rect(surf, ORANGE_HI,
                         (rect.x + 10, rect.y + 10, 2, 14), border_radius=1)
        surf.blit(FONT_SM.render(title, True, ORANGE_HI), (rect.x + 18, rect.y + 10))

def draw_hline(surf, x, y, w):
    pygame.draw.line(surf, BORDER_LINE, (x, y), (x + w, y))

def draw_axis_bar(surf, x, y, w, h, value, label="", show_val=True):
    pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)
    cx = x + w // 2
    pygame.draw.line(surf, ORANGE_DIM, (cx, y + 2), (cx, y + h - 2))
    pct = (value + 1) / 2.0
    fill_x = cx if value >= 0 else x + int(pct * w)
    fill_w = int((pct - 0.5) * w) if value >= 0 else cx - fill_x
    if fill_w > 0:
        pygame.draw.rect(surf, ORANGE_MID,
                         pygame.Rect(fill_x, y + 2, fill_w, h - 4), border_radius=2)
    if label:
        surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x, y - 15))
    if show_val:
        v = FONT_XS.render(f"{value:+.3f}", True, ORANGE_HI)
        surf.blit(v, (x + w - v.get_width(), y - 15))

def draw_throttle_v(surf, x, y, w, h, value, mouse_mode=False):
    """Vertical throttle bar. Blue tint when mouse_mode=True."""
    pct   = 1.0 - (value + 1) / 2.0
    bg_c  = (8,  8, 20) if mouse_mode else (8, 8, 8)
    brd_c = BLUE_MOUSE  if mouse_mode else ORANGE_DIM
    fil_c = BLUE_MOUSE  if mouse_mode else ORANGE_MID
    pygame.draw.rect(surf, bg_c,  (x, y, w, h), border_radius=2)
    pygame.draw.rect(surf, brd_c, (x, y, w, h), width=1, border_radius=2)
    fh = int(pct * (h - 4))
    if fh > 0:
        pygame.draw.rect(surf, fil_c, (x + 2, y + h - 2 - fh, w - 4, fh), border_radius=2)
    lbl = FONT_XS.render("THR", True, brd_c)
    surf.blit(lbl, (x + w // 2 - lbl.get_width() // 2, y - 15))
    val = FONT_XS.render(f"{int(pct * 100)}%", True, ORANGE_HI)
    surf.blit(val, (x + w // 2 - val.get_width() // 2, y + h + 4))

def draw_xy_pad(surf, x, y, size, ax, ay, mouse_mode=False):
    """XY joystick pad. Blue tint and hint when mouse_mode=True."""
    bg_c  = (8,  8, 20) if mouse_mode else (8, 8, 8)
    brd_c = BLUE_MOUSE  if mouse_mode else ORANGE_DIM
    dot_c = BLUE_MOUSE  if mouse_mode else ORANGE_HI
    pygame.draw.rect(surf, bg_c,  (x, y, size, size), border_radius=3)
    pygame.draw.rect(surf, brd_c, (x, y, size, size), width=1, border_radius=3)
    cx2, cy2 = x + size // 2, y + size // 2
    for t in (0.25, 0.5, 0.75):
        gx = x + int(t * size); gy = y + int(t * size)
        pygame.draw.line(surf, (20, 8, 0), (gx, y + 1), (gx, y + size - 1))
        pygame.draw.line(surf, (20, 8, 0), (x + 1, gy), (x + size - 1, gy))
    pygame.draw.line(surf, brd_c, (cx2, y + 2),   (cx2, y + size - 2))
    pygame.draw.line(surf, brd_c, (x + 2, cy2),   (x + size - 2, cy2))
    dx = cx2 + int(ax * (size // 2 - 6))
    dy = cy2 + int(ay * (size // 2 - 6))
    pygame.draw.circle(surf, dot_c,       (dx, dy), 6)
    pygame.draw.circle(surf, ORANGE_GLOW, (dx, dy), 2)
    hint = "X/Y  DRAG" if mouse_mode else "X/Y"
    surf.blit(FONT_XS.render(hint, True, brd_c),
              (x + size // 2 - FONT_XS.size(hint)[0] // 2, y - 15))

def draw_button_widget(surf, x, y, w, h, pressed, label, clickable=False):
    bg  = ORANGE_DIM if pressed else (8, 8, 8)
    bc  = ORANGE_HI  if pressed else (BLUE_MOUSE if clickable else ORANGE_DIM)
    pygame.draw.rect(surf, bg, (x, y, w, h), border_radius=3)
    pygame.draw.rect(surf, bc, (x, y, w, h), width=1, border_radius=3)
    pygame.draw.circle(surf,
                       ORANGE_GLOW if pressed else (30, 12, 0),
                       (x + w // 2, y + 9), 4)
    lbl = FONT_XS.render(label, True, ORANGE_HI if pressed else bc)
    surf.blit(lbl, (x + w // 2 - lbl.get_width() // 2, y + 18))

def draw_pov(surf, cx, cy, size, hat_x, hat_y, mouse_mode=False):
    ad   = (1 if hat_x > 0 else (-1 if hat_x < 0 else 0),
            1 if hat_y > 0 else (-1 if hat_y < 0 else 0))
    dirs = {(0,0):"·",(0,-1):"N",(0,1):"S",(-1,0):"W",(1,0):"E",
            (-1,-1):"NW",(1,-1):"NE",(-1,1):"SW",(1,1):"SE"}
    cell = size // 3
    for row in range(3):
        for col in range(3):
            dx = col - 1; dy = row - 1
            active = (dx, dy) == (ad[0], -ad[1]) and ad != (0, 0)
            rx = cx - size // 2 + col * cell + 2
            ry = cy - size // 2 + row * cell + 2
            rw = cell - 4; rh = cell - 4
            bg_c  = ORANGE_DIM if active else (8, 8, 8)
            brd_c = ORANGE_HI  if active else (BLUE_MOUSE if mouse_mode else ORANGE_DIM)
            pygame.draw.rect(surf, bg_c,  (rx, ry, rw, rh), border_radius=2)
            pygame.draw.rect(surf, brd_c, (rx, ry, rw, rh), width=1, border_radius=2)
            if dx == 0 and dy == 0:
                pygame.draw.circle(surf,
                    ORANGE_HI if ad != (0, 0) else (30, 12, 0),
                    (rx + rw // 2, ry + rh // 2), 4)
    lbl_txt = "POV  HOLD" if mouse_mode else "POV HAT"
    surf.blit(FONT_XS.render(lbl_txt, True, BLUE_MOUSE if mouse_mode else ORANGE_DIM),
              (cx - FONT_XS.size(lbl_txt)[0] // 2, cy - size // 2 - 16))
    d = FONT_MD.render(dirs.get((ad[0], ad[1]), "·"), True, ORANGE_HI)
    surf.blit(d, (cx - d.get_width() // 2, cy + size // 2 + 4))

def draw_trigger(surf, cx, cy, pressed):
    c = RED_ERR if pressed else (8, 8, 8)
    pygame.draw.circle(surf, c, (cx, cy), 24)
    pygame.draw.circle(surf, RED_ERR if pressed else ORANGE_DIM, (cx, cy), 24, width=2)
    pygame.draw.circle(surf, ORANGE_GLOW if pressed else (30, 12, 0), (cx, cy), 8)
    surf.blit(FONT_XS.render("TRIGGER", True, ORANGE_DIM),
              (cx - FONT_XS.size("TRIGGER")[0] // 2, cy - 42))
    s = FONT_XS.render("FIRE" if pressed else "SAFE",
                        True, RED_ERR if pressed else ORANGE_DIM)
    surf.blit(s, (cx - s.get_width() // 2, cy + 30))

def draw_status_badge(surf, joy_connected, mouse_mode, name=""):
    if joy_connected:
        color = ORANGE_HI;  label = "CTRL CONNECTED"
    elif mouse_mode:
        color = BLUE_MOUSE; label = "MOUSE CONTROL"
    else:
        color = RED_ERR;    label = "NO CONTROLLER"
    badge = FONT_XS.render(label, True, color)
    bw = badge.get_width() + 26; bx = BASE_W - bw - 18
    pygame.draw.rect(surf, (6, 2, 0), (bx, 12, bw, 22), border_radius=2)
    pygame.draw.rect(surf, color,     (bx, 12, bw, 22), width=1, border_radius=2)
    pygame.draw.circle(surf, color, (bx + 10, 23), 3)
    surf.blit(badge, (bx + 18, 17))
    if name and joy_connected:
        surf.blit(FONT_XS.render(name[:50], True, ORANGE_DIM), (bx, 38))

def draw_arm_panel(surf, rect, motor_states, servo_angle, msgs):
    draw_panel(surf, rect, "ARM / MOTOR COMMANDS")
    x0, y0 = rect.x + 14, rect.y + 32
    all_names = MOTOR_NAMES + ["Wrist"]
    col_w = (rect.w - 28) // len(all_names)
    for i, name in enumerate(all_names):
        cx = x0 + i * col_w + col_w // 2
        surf.blit(FONT_XS.render(name, True, ORANGE_DIM),
                  (cx - FONT_XS.size(name)[0] // 2, y0))
        if name == "Wrist":
            bx = cx - col_w // 2 + 4; bw = col_w - 8; by = y0 + 16
            pygame.draw.rect(surf, (8, 8, 8),   (bx, by, bw, 14), border_radius=2)
            pygame.draw.rect(surf, ORANGE_DIM,  (bx, by, bw, 14), width=1, border_radius=2)
            fill = int((servo_angle / 180) * (bw - 4))
            if fill > 0:
                pygame.draw.rect(surf, ORANGE_MID, (bx + 2, by + 2, fill, 10), border_radius=2)
            ang = FONT_XS.render(f"{servo_angle}°", True, ORANGE_HI)
            surf.blit(ang, (cx - ang.get_width() // 2, by + 17))
        else:
            state = motor_states[i]
            disp, sc = (MOTOR_DIR_LABELS[name][0], ORANGE_HI) if state == "FWD" else \
                       (MOTOR_DIR_LABELS[name][1], RED_ERR)   if state == "REV" else \
                       ("STOP", (60, 30, 0))
            bx = cx - col_w // 2 + 4; bw = col_w - 8; by = y0 + 16
            pygame.draw.rect(surf, (6, 2, 0), (bx, by, bw, 16), border_radius=2)
            pygame.draw.rect(surf, sc if state != "STOP" else ORANGE_DIM,
                             (bx, by, bw, 16), width=1, border_radius=2)
            st = FONT_XS.render(disp, True, sc)
            surf.blit(st, (cx - st.get_width() // 2, by + 2))
    surf.blit(FONT_XS.render(
        f"ROS2  /arm_cmd  [Int16MultiArray]  msgs:{msgs}", True, GREEN_OK),
        (x0, y0 + 52))

def draw_wheel_panel(surf, rect, x_val, z_val, thr, msgs):
    draw_panel(surf, rect, "WHEEL DRIVE")
    x0, y0 = rect.x + 14, rect.y + 32
    bw = rect.w - 120
    def row(label, value, y):
        surf.blit(FONT_XS.render(label, True, ORANGE_DIM), (x0, y))
        draw_axis_bar(surf, x0 + 100, y, bw, 12, value, show_val=True)
    row("FWD / REV", x_val, y0)
    row("TURN",      z_val, y0 + 22)
    ty = y0 + 44
    surf.blit(FONT_XS.render("THROTTLE", True, ORANGE_DIM), (x0, ty))
    bx = x0 + 100
    pygame.draw.rect(surf, (8, 8, 8),  (bx, ty, bw, 12), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (bx, ty, bw, 12), width=1, border_radius=2)
    fill = int(thr * (bw - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (bx + 2, ty + 2, fill, 8), border_radius=2)
    surf.blit(FONT_XS.render(f"{int(thr * 100)}%", True, ORANGE_HI), (bx + bw + 6, ty))
    lv = max(-1.0, min(1.0, x_val - z_val))
    rv = max(-1.0, min(1.0, x_val + z_val))
    row("LEFT  MTR", lv, ty + 22)
    row("RIGHT MTR", rv, ty + 44)
    surf.blit(FONT_XS.render(
        f"ROS2  /cmd_vel  [Twist]  msgs:{msgs}", True, GREEN_OK),
        (x0, rect.bottom - 16))


# ── Terminal print helpers ────────────────────────────────────────────────────
_last_motor: dict  = {}
_last_servo: int   = -1
_last_wheel: tuple = (None, None, None)

def maybe_print_motor(name, state):
    if _last_motor.get(name) == state: return
    _last_motor[name] = state
    verb = MOTOR_VERBOSE[name][0] if state == "FWD" else \
           MOTOR_VERBOSE[name][1] if state == "REV" else f"{name} STOP"
    print(f"[ARM]  {name:<12} → {verb}")

def maybe_print_servo(angle):
    global _last_servo
    if angle != _last_servo:
        _last_servo = angle
        print(f"[ARM]  Wrist        → {angle}°")

def maybe_print_wheel(x, z, thr):
    global _last_wheel
    lx, lz, lt = _last_wheel
    if lx is None or abs(x - lx) > 0.05 or abs(z - lz) > 0.05 or abs(thr - lt) > 0.05:
        _last_wheel = (x, z, thr)
        print(f"[WHEEL] fwd={x:+.2f}  turn={z:+.2f}  thr={int(thr * 100):3d}%")


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.cmd_vel_pub = self.create_publisher(Twist,           '/cmd_vel', qos_be)
        self.arm_cmd_pub = self.create_publisher(Int16MultiArray, '/arm_cmd', qos_rel)

    def pub_cmd_vel(self, fwd, turn):
        msg = Twist()
        msg.linear.x  = float(fwd)
        msg.angular.z = float(turn)
        self.cmd_vel_pub.publish(msg)

    def pub_arm_cmd(self, m1, m2, m3, m4a, m4b, servo, speed=255):
        msg = Int16MultiArray()
        msg.data = [int(m1), int(m2), int(m3), int(m4a), int(m4b),
                    int(servo), int(speed)]
        self.arm_cmd_pub.publish(msg)


# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    global FONT_XS, FONT_SM, FONT_MD, FONT_LG

    rclpy.init(args=args)
    node = GuiNode()

    pygame.init()
    screen = pygame.display.set_mode(
        (BASE_W, BASE_H), pygame.RESIZABLE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Interplanetar — Rover Teleop")
    clock  = pygame.time.Clock()
    canvas = pygame.Surface((BASE_W, BASE_H))

    for fname in ("couriernew", "courier new", "monospace", ""):
        try:
            FONT_XS = pygame.font.SysFont(fname, 12, bold=True)
            FONT_SM = pygame.font.SysFont(fname, 13, bold=True)
            FONT_MD = pygame.font.SysFont(fname, 18, bold=True)
            FONT_LG = pygame.font.SysFont(fname, 24, bold=True)
            break
        except Exception:
            continue

    pygame.joystick.init()
    joy  = None
    virt = VirtualInput()

    def try_connect():
        nonlocal joy
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            print(f"[INFO] Joystick: {joy.get_name()}")

    try_connect()

    servo_angle  = 90
    motor_states = ["STOP"] * 5
    arm_msgs     = 0
    wheel_msgs   = 0
    last_arm_t   = 0.0
    last_wheel_t = 0.0
    wheel_x = wheel_z = wheel_thr = 0.0
    W, H = BASE_W, BASE_H

    while rclpy.ok():
        now = time.monotonic()

        for event in pygame.event.get():
            if event.type == pygame.QUIT: break
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: break

            if event.type == pygame.JOYDEVICEADDED:
                try_connect()
            if event.type == pygame.JOYDEVICEREMOVED:
                joy = None
                print("[INFO] Joystick disconnected — switching to mouse control")

            if event.type == pygame.VIDEORESIZE:
                W, H = event.w, event.h
                screen = pygame.display.set_mode(
                    (W, H), pygame.RESIZABLE | pygame.DOUBLEBUF)

            # ── Mouse events ──────────────────────────────────────────────
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                virt.on_down(*canvas_pos(event.pos, W, H))
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                virt.on_up()
            if event.type == pygame.MOUSEMOTION:
                virt.on_motion(*canvas_pos(event.pos, W, H))
            if event.type == pygame.MOUSEWHEEL:
                virt.on_scroll(*canvas_pos(pygame.mouse.get_pos(), W, H), event.y)

        else:
            for _ in pygame.event.get([pygame.JOYAXISMOTION, pygame.JOYBALLMOTION,
                                       pygame.JOYHATMOTION, pygame.JOYBUTTONDOWN,
                                       pygame.JOYBUTTONUP]):
                pass

            W, H = screen.get_size()
            joy_connected = joy is not None and pygame.joystick.get_count() > 0
            mouse_mode    = not joy_connected

            # ── Read inputs ───────────────────────────────────────────────
            if joy_connected:
                try:
                    axes    = [joy.get_axis(i)   for i in range(joy.get_numaxes())]
                    buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
                    hats    = [joy.get_hat(i)    for i in range(joy.get_numhats())]
                    ax_x = axes[0] if len(axes) > 0 else 0.0
                    ax_y = axes[1] if len(axes) > 1 else 0.0
                    ax_z = axes[2] if len(axes) > 2 else 0.0
                    ax_t = axes[3] if len(axes) > 3 else -1.0
                    hat_x, hat_y = hats[0] if hats else (0, 0)
                    joy_name = joy.get_name()
                except Exception:
                    joy_connected = False
                    mouse_mode    = True

            if mouse_mode:
                ax_x    = 0.0           # no analog X-axis in mouse mode
                ax_y    = virt.ax_y
                ax_z    = virt.ax_z
                ax_t    = virt.ax_t
                hat_x   = virt.hat_x
                hat_y   = virt.hat_y
                buttons = list(virt.buttons)
                joy_name = ""

            # ── Arm @ 20 Hz ───────────────────────────────────────────────
            if (now - last_arm_t) >= SEND_RATE:
                last_arm_t = now

                m1 = CMD_MOTOR1_FWD  if hat_x > 0 else \
                     CMD_MOTOR1_REV  if hat_x < 0 else CMD_MOTOR1_STOP
                motor_states[0] = "FWD" if hat_x > 0 else "REV" if hat_x < 0 else "STOP"

                m2 = CMD_MOTOR2_FWD  if hat_y > 0 else \
                     CMD_MOTOR2_REV  if hat_y < 0 else CMD_MOTOR2_STOP
                motor_states[1] = "FWD" if hat_y > 0 else "REV" if hat_y < 0 else "STOP"

                if len(buttons) > 5 and buttons[5]:
                    m3 = CMD_MOTOR3_FWD;  motor_states[2] = "FWD"
                elif len(buttons) > 3 and buttons[3]:
                    m3 = CMD_MOTOR3_REV;  motor_states[2] = "REV"
                else:
                    m3 = CMD_MOTOR3_STOP; motor_states[2] = "STOP"

                if len(buttons) > 4 and buttons[4]:
                    servo_angle = min(SERVO_MAX, servo_angle + SERVO_STEP)
                if len(buttons) > 2 and buttons[2]:
                    servo_angle = max(SERVO_MIN, servo_angle - SERVO_STEP)

                if len(buttons) > 7 and buttons[7]:
                    m4a = CMD_MOTOR4A_FWD;  motor_states[3] = "FWD"
                elif len(buttons) > 6 and buttons[6]:
                    m4a = CMD_MOTOR4A_REV;  motor_states[3] = "REV"
                else:
                    m4a = CMD_MOTOR4A_STOP; motor_states[3] = "STOP"

                if len(buttons) > 9 and buttons[9]:
                    m4b = CMD_MOTOR4B_FWD;  motor_states[4] = "FWD"
                elif len(buttons) > 8 and buttons[8]:
                    m4b = CMD_MOTOR4B_REV;  motor_states[4] = "REV"
                else:
                    m4b = CMD_MOTOR4B_STOP; motor_states[4] = "STOP"

                for name, state in zip(MOTOR_NAMES, motor_states):
                    maybe_print_motor(name, state)
                maybe_print_servo(servo_angle)

                node.pub_arm_cmd(m1, m2, m3, m4a, m4b, servo_angle)
                arm_msgs += 1

            # ── Wheels @ 20 Hz ────────────────────────────────────────────
            if (now - last_wheel_t) >= WHEEL_SEND_RATE:
                last_wheel_t = now
                raw_x = -ax_y; raw_z = ax_z
                wheel_x   = raw_x if abs(raw_x) > DEADZONE else 0.0
                wheel_z   = raw_z if abs(raw_z) > DEADZONE else 0.0
                wheel_thr = (1.0 - ax_t) / 2.0
                node.pub_cmd_vel(
                    max(-1.0, min(1.0, wheel_x * wheel_thr)),
                    max(-1.0, min(1.0, wheel_z * wheel_thr)))
                wheel_msgs += 1
                maybe_print_wheel(wheel_x, wheel_z, wheel_thr)

            rclpy.spin_once(node, timeout_sec=0)

            # ── Draw ──────────────────────────────────────────────────────
            canvas.fill(BG)
            canvas.blit(FONT_LG.render(
                "INTERPLANETAR  //  ROVER TELEOP  //  ROS2", True, ORANGE_HI), (16, 12))
            draw_status_badge(canvas, joy_connected, mouse_mode, joy_name)
            draw_hline(canvas, 16, 44, BASE_W - 32)

            LP = pygame.Rect(14, 52, 440, 360)
            draw_panel(canvas, LP, "AXES & STICK")
            ay = LP.y + 40
            draw_axis_bar(canvas, LP.x + 14, ay,      LP.w - 28, 14, ax_x, "X AXIS   L/R")
            draw_axis_bar(canvas, LP.x + 14, ay + 42, LP.w - 28, 14, ax_y, "Y AXIS   FWD/BACK")
            draw_axis_bar(canvas, LP.x + 14, ay + 84, LP.w - 28, 14, ax_z, "Z TWIST  RUDDER")
            draw_xy_pad(canvas, XY_X, XY_Y, XY_SIZE, ax_x, ax_y, mouse_mode)
            draw_throttle_v(canvas, THR_X, THR_Y, THR_W, THR_H, ax_t, mouse_mode)
            rr = pygame.Rect(LP.x + 14, ay + 286, LP.w - 28, 42)
            pygame.draw.rect(canvas, (6, 2, 0),   rr, border_radius=2)
            pygame.draw.rect(canvas, ORANGE_DIM,  rr, width=1, border_radius=2)
            canvas.blit(FONT_XS.render("RAW AXIS VALUES", True, ORANGE_MID), (rr.x + 8, rr.y + 4))
            for i, (n, v) in enumerate(zip(["AX0", "AX1", "AX2", "AX3"],
                                           [ax_x, ax_y, ax_z, ax_t])):
                rx = rr.x + 10 + i * (rr.w // 4)
                canvas.blit(FONT_XS.render(n,           True, ORANGE_DIM), (rx, rr.y + 17))
                canvas.blit(FONT_XS.render(f"{v:+.4f}", True, ORANGE_HI),  (rx, rr.y + 28))

            RP = pygame.Rect(466, 52, 480, 360)
            draw_panel(canvas, RP, "BUTTONS")
            for i in range(min(12, len(BUTTON_LABELS))):
                r = btn_rect(i)
                # B3–B10 (indices 2–9) are arm controls — highlighted blue in mouse mode
                clickable = mouse_mode and 2 <= i <= 9
                draw_button_widget(canvas, r.x, r.y, r.w, r.h,
                                   buttons[i] if i < len(buttons) else False,
                                   BUTTON_LABELS[i], clickable)
            draw_pov(canvas, POV_CX, POV_CY, POV_SIZE, hat_x, hat_y, mouse_mode)
            draw_trigger(canvas, RP.x + 340, RP.y + 295,
                         buttons[0] if buttons else False)

            draw_hline(canvas, 14, 422, BASE_W - 28)
            draw_arm_panel(canvas, pygame.Rect(14, 428, 930, 128),
                           motor_states, servo_angle, arm_msgs)
            draw_hline(canvas, 14, 562, BASE_W - 28)
            draw_wheel_panel(canvas, pygame.Rect(14, 568, 930, 158),
                             wheel_x, wheel_z, wheel_thr, wheel_msgs)

            if mouse_mode:
                foot = FONT_XS.render(
                    "MOUSE MODE —  XY pad: drag to drive   THR: drag/scroll   "
                    "POV + B3–B10: hold to activate",
                    True, BLUE_MOUSE)
            else:
                foot = FONT_XS.render(
                    "ESC:quit  HAT E/W:Base  HAT N/S:Shoulder  "
                    "B6/B4:Elbow  B5/B3:Servo  B8/B7:Roller  B10/B9:Gripper",
                    True, ORANGE_MID)
            canvas.blit(foot, (BASE_W // 2 - foot.get_width() // 2, BASE_H - 16))

            if (W, H) != (BASE_W, BASE_H):
                scale = min(W / BASE_W, H / BASE_H)
                ow, oh = int(BASE_W * scale), int(BASE_H * scale)
                scaled = pygame.transform.smoothscale(canvas, (ow, oh))
                screen.fill(BG)
                screen.blit(scaled, ((W - ow) // 2, (H - oh) // 2))
            else:
                screen.blit(canvas, (0, 0))

            pygame.display.flip()
            clock.tick(60)
            continue
        break

    print("[INFO] Shutdown — sending stop")
    node.pub_cmd_vel(0.0, 0.0)
    node.pub_arm_cmd(CMD_MOTOR1_STOP, CMD_MOTOR2_STOP, CMD_MOTOR3_STOP,
                     CMD_MOTOR4A_STOP, CMD_MOTOR4B_STOP, servo_angle)
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
