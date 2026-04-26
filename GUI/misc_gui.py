#!/usr/bin/env python3
"""
Misc board test GUI (pygame)

Consistency goals:
- Matches the black/orange style used by existing GUIs
- Uses the same binary packet format and CRC8
- Talks to Misc receiver over UDP

Run:
  python3 GUI/misc_gui.py
"""

import socket
import sys
import time
import os

# Keep joystick events flowing when window is unfocused (same as other GUIs).
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import pygame

# Network
# Use rover/Jetson IP when running GUI from a different machine.
ROVER_IP = "192.168.10.177"
MISC_PORT = 5763

# Protocol
SOF1 = 0xAA
SOF2 = 0xBB
ACK_BYTE = 0xAC

CMD_HEARTBEAT = 0x00
CMD_NIGHT_LED_ON = 0x10
CMD_NIGHT_LED_OFF = 0x11
CMD_INDICATOR_ON = 0x20
CMD_INDICATOR_OFF = 0x21
CMD_INDICATOR_PWM = 0x22
CMD_SERVO1_ANGLE = 0x31
CMD_SERVO2_ANGLE = 0x32
CMD_SERVO3_ANGLE = 0x33
CMD_SERVO4_ANGLE = 0x34
CMD_LASER1_ON = 0x41
CMD_LASER1_OFF = 0x42
CMD_LASER2_ON = 0x43
CMD_LASER2_OFF = 0x44
CMD_ESTOP = 0xFF

CMD_NAMES = {
    CMD_HEARTBEAT: "HEARTBEAT",
    CMD_NIGHT_LED_ON: "NIGHT_ON",
    CMD_NIGHT_LED_OFF: "NIGHT_OFF",
    CMD_INDICATOR_ON: "IND_ON",
    CMD_INDICATOR_OFF: "IND_OFF",
    CMD_INDICATOR_PWM: "IND_PWM",
    CMD_SERVO1_ANGLE: "SERVO1",
    CMD_SERVO2_ANGLE: "SERVO2",
    CMD_SERVO3_ANGLE: "SERVO3",
    CMD_SERVO4_ANGLE: "SERVO4",
    CMD_LASER1_ON: "LASER1_ON",
    CMD_LASER1_OFF: "LASER1_OFF",
    CMD_LASER2_ON: "LASER2_ON",
    CMD_LASER2_OFF: "LASER2_OFF",
    CMD_ESTOP: "ESTOP",
}

LED_MODE_NAMES = {
    0: "OFF",
    1: "RED",
    2: "GREEN",
    3: "YELLOW",
}

STATUS_NAMES = {
    0x00: "OK",
    0x01: "CRC_ERR",
    0x02: "UNK_CMD",
    0x03: "ESTOP",
}

# Theme (same direction as working_gui.py)
BG = (0, 0, 0)
ORANGE_HI = (255, 140, 0)
ORANGE_MID = (200, 90, 0)
ORANGE_DIM = (150, 75, 0)
ORANGE_GLOW = (255, 180, 60)
BORDER_LINE = (120, 55, 0)
RED_ERR = (255, 50, 20)

BASE_W, BASE_H = 960, 760
SEND_GAP_S = 0.04
SEND_RATE = 0.05
SERVO_STEP = 2
HEARTBEAT_INTERVAL = 0.4


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


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

    def local_port(self) -> int:
        return self.sock.getsockname()[1]

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

    def poll_ack(self):
        got = False
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

            seq = (data[1] << 8) | data[2]
            status = data[3]
            self.last_ack_seq = seq
            self.last_ack_status = STATUS_NAMES.get(status, f"0x{status:02X}")
            self.last_ack_time = time.monotonic()
            got = True

        return got


class Button:
    def __init__(self, rect, label, key):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.key = key


def draw_panel(surf, rect, title=None):
    pygame.draw.rect(surf, (4, 4, 4), rect, border_radius=3)
    pygame.draw.rect(surf, ORANGE_DIM, rect, width=1, border_radius=3)
    if title:
        pygame.draw.rect(surf, ORANGE_HI, (rect.x + 10, rect.y + 10, 2, 14), border_radius=1)
        lbl = FONT_SM.render(title, True, ORANGE_HI)
        surf.blit(lbl, (rect.x + 18, rect.y + 10))


def draw_button(surf, btn: Button, active=False, error=False):
    if error:
        fg = RED_ERR
        bg = (30, 8, 8)
    elif active:
        fg = ORANGE_HI
        bg = ORANGE_DIM
    else:
        fg = ORANGE_DIM
        bg = (8, 8, 8)

    pygame.draw.rect(surf, bg, btn.rect, border_radius=3)
    pygame.draw.rect(surf, fg, btn.rect, width=1, border_radius=3)
    txt = FONT_XS.render(btn.label, True, fg)
    surf.blit(txt, (btn.rect.centerx - txt.get_width() // 2, btn.rect.centery - txt.get_height() // 2))


def draw_hline(surf, x, y, w):
    pygame.draw.line(surf, BORDER_LINE, (x, y), (x + w, y))


def draw_slider(surf, x, y, w, h, value, label):
    pygame.draw.rect(surf, (8, 8, 8), (x, y, w, h), border_radius=2)
    pygame.draw.rect(surf, ORANGE_DIM, (x, y, w, h), width=1, border_radius=2)

    fill = int((value / 180.0) * (w - 4))
    if fill > 0:
        pygame.draw.rect(surf, ORANGE_MID, (x + 2, y + 2, fill, h - 4), border_radius=2)

    l = FONT_XS.render(label, True, ORANGE_DIM)
    v = FONT_XS.render(f"{value:3d} deg", True, ORANGE_HI)
    surf.blit(l, (x, y - 16))
    surf.blit(v, (x + w + 8, y - 1))


def main():
    global FONT_XS, FONT_SM, FONT_MD, FONT_LG

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((BASE_W, BASE_H), pygame.RESIZABLE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Misc Board Test GUI")
    canvas = pygame.Surface((BASE_W, BASE_H))
    clock = pygame.time.Clock()

    for name in ("couriernew", "courier new", "monospace"):
        try:
            FONT_XS = pygame.font.SysFont(name, 12, bold=True)
            FONT_SM = pygame.font.SysFont(name, 13, bold=True)
            FONT_MD = pygame.font.SysFont(name, 18, bold=True)
            FONT_LG = pygame.font.SysFont(name, 24, bold=True)
            break
        except Exception:
            continue

    sender = MiscSender()
    print(f"[MISC_GUI] UDP target {ROVER_IP}:{MISC_PORT} from local :{sender.local_port()}")

    def send_cmd(cmd: int, val: int = 0, verbose: bool = True):
        nonlocal packets_sent
        _, seq = sender.send(cmd, val)
        packets_sent += 1
        if verbose:
            name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
            print(f"[MISC_GUI] TX {name:<10} val={val:3d} seq={seq}")

    joy = None

    def try_connect_joy():
        nonlocal joy
        if pygame.joystick.get_count() > 0:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            print(f"[MISC] Joystick connected: {joy.get_name()}")

    try_connect_joy()

    state = {
        "night": False,
        "laser1": False,
        "laser2": False,
        "led_mode": 0,
        "servo1": 90,
        "servo2": 90,
        "servo3": 90,
        "servo4": 90,
    }

    def set_led_mode(mode: int):
        state["led_mode"] = max(0, min(3, int(mode)))
        send_cmd(CMD_INDICATOR_PWM, state["led_mode"])

    def toggle_led_mode():
        state["led_mode"] = (state["led_mode"] + 1) % 4
        send_cmd(CMD_INDICATOR_ON, 0)

    servo_cmd = {
        "servo1": CMD_SERVO1_ANGLE,
        "servo2": CMD_SERVO2_ANGLE,
        "servo3": CMD_SERVO3_ANGLE,
        "servo4": CMD_SERVO4_ANGLE,
    }

    buttons = {
        "night": Button((28, 94, 170, 44), "NIGHT LED", "night"),
        "laser1": Button((218, 94, 170, 44), "LASER 1", "laser1"),
        "laser2": Button((408, 94, 170, 44), "LASER 2", "laser2"),
        "estop": Button((598, 94, 170, 44), "E-STOP", "estop"),
        "led_toggle": Button((28, 174, 190, 36), "LED TOGGLE", "led_toggle"),
        "led_off": Button((238, 174, 130, 36), "OFF", "led_off"),
        "led_red": Button((378, 174, 130, 36), "RED", "led_red"),
        "led_green": Button((518, 174, 130, 36), "GREEN", "led_green"),
        "led_yellow": Button((658, 174, 130, 36), "YELLOW", "led_yellow"),
    }

    # Per-servo +/- buttons
    servo_rows = [
        ("servo1", 300),
        ("servo2", 365),
        ("servo3", 430),
        ("servo4", 495),
    ]
    for key, y in servo_rows:
        buttons[f"{key}_dn"] = Button((720, y - 4, 54, 28), "-", f"{key}_dn")
        buttons[f"{key}_up"] = Button((780, y - 4, 54, 28), "+", f"{key}_up")

    packets_sent = 0
    last_printed_ack_seq = -1
    last_send_time = 0.0
    last_send = 0.0
    last_heartbeat = 0.0
    prev_buttons = [0] * 16
    prev_hat = (0, 0)
    joy_name = ""
    ax_x = ax_y = ax_z = 0.0
    hat_x = hat_y = 0

    W, H = BASE_W, BASE_H

    while True:
        now = time.monotonic()

        sender.poll_ack()
        if sender.last_ack_seq != last_printed_ack_seq and sender.last_ack_seq >= 0:
            print(
                f"[MISC_GUI] ACK seq={sender.last_ack_seq} status={sender.last_ack_status}"
            )
            last_printed_ack_seq = sender.last_ack_seq

        # Keep receiver watchdog alive even when no button is pressed.
        if now - last_heartbeat >= HEARTBEAT_INTERVAL:
            send_cmd(CMD_HEARTBEAT, 0, verbose=False)
            last_heartbeat = now

        # Keep joystick state updated even when window is unfocused.
        for _ in pygame.event.get([
            pygame.JOYAXISMOTION,
            pygame.JOYBALLMOTION,
            pygame.JOYHATMOTION,
            pygame.JOYBUTTONDOWN,
            pygame.JOYBUTTONUP,
        ]):
            pass

        click = None
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit(0)
            if event.type == pygame.JOYDEVICEADDED:
                try_connect_joy()
            if event.type == pygame.JOYDEVICEREMOVED:
                joy = None
            if event.type == pygame.VIDEORESIZE:
                W, H = event.w, event.h
                screen = pygame.display.set_mode((W, H), pygame.RESIZABLE | pygame.DOUBLEBUF)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                click = event.pos

        connected = joy is not None and pygame.joystick.get_count() > 0
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
                hat_x, hat_y = hats[0] if n_hat > 0 else (0, 0)
                joy_name = joy.get_name()
            except Exception:
                connected = False

        if not connected:
            buttons_state = [0] * 16
            hat_x = hat_y = 0
            ax_x = ax_y = ax_z = 0.0
            joy_name = ""

        # Joystick controls (edge-triggered for toggles, held for servo trim).
        if connected and (now - last_send) >= SEND_RATE:
            last_send = now

            # Single-button toggles for outputs to reduce required joystick buttons.
            # B11 -> Night toggle, B12 -> Laser1 toggle, B8 -> Laser2 toggle.
            if len(buttons_state) > 10 and buttons_state[10] and not prev_buttons[10]:
                state["night"] = not state["night"]
                send_cmd(CMD_NIGHT_LED_ON if state["night"] else CMD_NIGHT_LED_OFF, 0)

            if len(buttons_state) > 11 and buttons_state[11] and not prev_buttons[11]:
                state["laser1"] = not state["laser1"]
                send_cmd(CMD_LASER1_ON if state["laser1"] else CMD_LASER1_OFF, 0)

            if len(buttons_state) > 7 and buttons_state[7] and not prev_buttons[7]:
                state["laser2"] = not state["laser2"]
                send_cmd(CMD_LASER2_ON if state["laser2"] else CMD_LASER2_OFF, 0)

            # E-stop on trigger edge.
            if len(buttons_state) > 0 and buttons_state[0] and not prev_buttons[0]:
                state["night"] = False
                state["laser1"] = False
                state["laser2"] = False
                state["led_mode"] = 0
                send_cmd(CMD_ESTOP, 0)

            # B13 or B14 cycles LED mode: OFF -> RED -> GREEN -> YELLOW -> OFF.
            if len(buttons_state) > 12 and buttons_state[12] and not prev_buttons[12]:
                toggle_led_mode()
            if len(buttons_state) > 13 and buttons_state[13] and not prev_buttons[13]:
                toggle_led_mode()

            # D-pad direct LED mode controls (edge-triggered):
            # Up=RED, Right=GREEN, Left=YELLOW, Down=OFF
            if (hat_x, hat_y) != prev_hat:
                if hat_y == 1:
                    set_led_mode(1)
                elif hat_x == 1:
                    set_led_mode(2)
                elif hat_x == -1:
                    set_led_mode(3)
                elif hat_y == -1:
                    set_led_mode(0)

            # Servo trims (same button family style as science GUI).
            if len(buttons_state) > 4:
                if buttons_state[4]:
                    state["servo1"] = min(180, state["servo1"] + SERVO_STEP)
                    send_cmd(CMD_SERVO1_ANGLE, state["servo1"])
                if buttons_state[2]:
                    state["servo1"] = max(0, state["servo1"] - SERVO_STEP)
                    send_cmd(CMD_SERVO1_ANGLE, state["servo1"])
                if buttons_state[5]:
                    state["servo2"] = min(180, state["servo2"] + SERVO_STEP)
                    send_cmd(CMD_SERVO2_ANGLE, state["servo2"])
                if buttons_state[3]:
                    state["servo2"] = max(0, state["servo2"] - SERVO_STEP)
                    send_cmd(CMD_SERVO2_ANGLE, state["servo2"])

            if len(buttons_state) > 9:
                if buttons_state[9]:
                    state["servo3"] = min(180, state["servo3"] + SERVO_STEP)
                    send_cmd(CMD_SERVO3_ANGLE, state["servo3"])
                if buttons_state[8]:
                    state["servo3"] = max(0, state["servo3"] - SERVO_STEP)
                    send_cmd(CMD_SERVO3_ANGLE, state["servo3"])

            if len(buttons_state) > 1:
                if buttons_state[1]:
                    state["servo4"] = min(180, state["servo4"] + SERVO_STEP)
                    send_cmd(CMD_SERVO4_ANGLE, state["servo4"])
            if len(buttons_state) > 6:
                if buttons_state[6]:
                    state["servo4"] = max(0, state["servo4"] - SERVO_STEP)
                    send_cmd(CMD_SERVO4_ANGLE, state["servo4"])

            prev_buttons = buttons_state + [0] * max(0, 16 - len(buttons_state))
            prev_hat = (hat_x, hat_y)

        if click is not None:
            wx, wy = click
            if (W, H) != (BASE_W, BASE_H):
                scale = min(W / BASE_W, H / BASE_H)
                out_w = int(BASE_W * scale)
                out_h = int(BASE_H * scale)
                ox = (W - out_w) // 2
                oy = (H - out_h) // 2
                if ox <= wx < ox + out_w and oy <= wy < oy + out_h:
                    wx = int((wx - ox) / scale)
                    wy = int((wy - oy) / scale)
                else:
                    wx, wy = -1, -1

            for btn in buttons.values():
                if not btn.rect.collidepoint((wx, wy)):
                    continue

                if now - last_send_time < SEND_GAP_S:
                    continue

                if btn.key == "night":
                    state["night"] = not state["night"]
                    cmd = CMD_NIGHT_LED_ON if state["night"] else CMD_NIGHT_LED_OFF
                    send_cmd(cmd, 0)
                elif btn.key == "laser1":
                    state["laser1"] = not state["laser1"]
                    cmd = CMD_LASER1_ON if state["laser1"] else CMD_LASER1_OFF
                    send_cmd(cmd, 0)
                elif btn.key == "laser2":
                    state["laser2"] = not state["laser2"]
                    cmd = CMD_LASER2_ON if state["laser2"] else CMD_LASER2_OFF
                    send_cmd(cmd, 0)
                elif btn.key == "estop":
                    state["night"] = False
                    state["laser1"] = False
                    state["laser2"] = False
                    state["led_mode"] = 0
                    send_cmd(CMD_ESTOP, 0)
                elif btn.key == "led_toggle":
                    toggle_led_mode()
                elif btn.key == "led_off":
                    set_led_mode(0)
                elif btn.key == "led_red":
                    set_led_mode(1)
                elif btn.key == "led_green":
                    set_led_mode(2)
                elif btn.key == "led_yellow":
                    set_led_mode(3)
                elif btn.key.endswith("_dn"):
                    skey = btn.key[:-3]
                    state[skey] = max(0, state[skey] - 5)
                    send_cmd(servo_cmd[skey], state[skey])
                elif btn.key.endswith("_up"):
                    skey = btn.key[:-3]
                    state[skey] = min(180, state[skey] + 5)
                    send_cmd(servo_cmd[skey], state[skey])

                last_send_time = now
                break

        # Draw
        canvas.fill(BG)

        title = FONT_LG.render("MISC BOARD TEST PANEL", True, ORANGE_HI)
        canvas.blit(title, (18, 12))
        ctrl = FONT_XS.render(
            f"CTRL: {'CONNECTED' if connected else 'NO CONTROLLER'}",
            True,
            ORANGE_HI if connected else RED_ERR,
        )
        canvas.blit(ctrl, (750, 16))
        if joy_name:
            jn = FONT_XS.render(joy_name[:26], True, ORANGE_DIM)
            canvas.blit(jn, (675, 34))
        draw_hline(canvas, 16, 46, BASE_W - 32)

        # Top controls panel
        p_top = pygame.Rect(14, 58, 932, 176)
        draw_panel(canvas, p_top, "Power and Safety")

        draw_button(canvas, buttons["night"], active=state["night"])
        draw_button(canvas, buttons["laser1"], active=state["laser1"])
        draw_button(canvas, buttons["laser2"], active=state["laser2"])
        draw_button(canvas, buttons["estop"], error=True)

        ind_lbl = FONT_XS.render("LED MATRIX MODES:", True, ORANGE_DIM)
        canvas.blit(ind_lbl, (28, 152))
        draw_button(canvas, buttons["led_toggle"])
        draw_button(canvas, buttons["led_off"], active=state["led_mode"] == 0)
        draw_button(canvas, buttons["led_red"], active=state["led_mode"] == 1)
        draw_button(canvas, buttons["led_green"], active=state["led_mode"] == 2)
        draw_button(canvas, buttons["led_yellow"], active=state["led_mode"] == 3)
        led_lbl = FONT_XS.render(f"Current: {LED_MODE_NAMES[state['led_mode']]}", True, ORANGE_GLOW)
        canvas.blit(led_lbl, (808, 184))

        # Joystick monitor panel
        p_in = pygame.Rect(14, 246, 332, 318)
        draw_panel(canvas, p_in, "Joystick Monitor")
        draw_slider(canvas, p_in.x + 14, p_in.y + 64, 220, 14, int((ax_x + 1.0) * 90), "AXIS X")
        draw_slider(canvas, p_in.x + 14, p_in.y + 128, 220, 14, int((ax_y + 1.0) * 90), "AXIS Y")
        draw_slider(canvas, p_in.x + 14, p_in.y + 192, 220, 14, int((ax_z + 1.0) * 90), "AXIS Z")
        hats_txt = FONT_SM.render(f"HAT: ({hat_x:+d}, {hat_y:+d})", True, ORANGE_HI)
        canvas.blit(hats_txt, (p_in.x + 14, p_in.bottom - 42))

        # Servo panel
        p_sv = pygame.Rect(356, 246, 590, 318)
        draw_panel(canvas, p_sv, "Servo Test")

        for skey, y in servo_rows:
            draw_slider(canvas, 378, y, 320, 14, state[skey], skey.upper())
            draw_button(canvas, buttons[f"{skey}_dn"])
            draw_button(canvas, buttons[f"{skey}_up"])

        # Network/status panel
        p_net = pygame.Rect(14, 576, 932, 170)
        draw_panel(canvas, p_net, "Link and ACK")

        net_color = ORANGE_HI if not sender.last_error else RED_ERR
        net = FONT_SM.render(f"UDP target: {ROVER_IP}:{MISC_PORT}", True, net_color)
        canvas.blit(net, (32, 618))

        sent = FONT_SM.render(f"Packets sent: {packets_sent}", True, ORANGE_HI)
        canvas.blit(sent, (32, 646))

        ack_age = now - sender.last_ack_time if sender.last_ack_time > 0 else -1.0
        ack_color = ORANGE_HI if ack_age >= 0 and ack_age < 2.0 else ORANGE_DIM
        ack_text = f"Last ACK: {sender.last_ack_status}   seq={sender.last_ack_seq}"
        ack = FONT_SM.render(ack_text, True, ack_color)
        canvas.blit(ack, (32, 674))

        age_text = "ACK age: --" if ack_age < 0 else f"ACK age: {ack_age:.2f}s"
        age = FONT_SM.render(age_text, True, ack_color)
        canvas.blit(age, (32, 702))

        if sender.last_error:
            err = FONT_SM.render(f"Socket error: {sender.last_error}", True, RED_ERR)
            canvas.blit(err, (420, 646))

        foot = FONT_XS.render(
            "B11:Night  B12:Laser1  B8:Laser2  B13/B14:LED Toggle  DPad U/R/L/D:RED/GREEN/YELLOW/OFF  Trigger:ESTOP",
            True,
            ORANGE_MID,
        )
        canvas.blit(foot, (BASE_W // 2 - foot.get_width() // 2, BASE_H - 16))

        # Scale for window size
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
