#!/usr/bin/env python3
"""
odrive_transmitter.py  —  Runs on Base Station (laptop / desktop)
==================================================================
Terminal UI using curses. Sends drive commands to Jetson via UDP.
Receives and displays telemetry from Jetson.

Controls:
  W / S       Forward / Reverse
  A / D       Turn left / Turn right (differential)
  Q / E       Throttle down / up  (+/- 10% per press)
  SPACE       Emergency stop (zero vel, latches until released)
  R           Release e-stop latch
  ESC / CTRL+C  Quit

Drive model (differential):
  left_vel  = (forward + turn) * throttle
  right_vel = (forward - turn) * throttle
  Clamped to [-MAX_VEL .. MAX_VEL] turns/s
  Scaled to i8 [-127..127] before packing

Packet format (transmitter → Jetson):
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]
  x_i8 → axis0 (left)
  z_i8 → axis1 (right)

Telemetry packet format (Jetson → transmitter, 22 bytes):
  [0xC1][dev_idx][axis_idx][SEQ_H][SEQ_L]
  [input_vel f32][pos_estimate f32][iq_measured f32][vbus_voltage f32]
  [CRC8]
"""

import curses
import socket
import threading
import time
import struct
import logging
import sys
import os

# ─── CONFIG ────────────────────────────────────────────────────────
JETSON_IP        = "192.168.1.100"   # ← change to your Jetson's IP
JETSON_PORT      = 5761
BIND_PORT        = 5762              # local port to receive telemetry on

SEND_RATE        = 20                # Hz — packets per second
MAX_VEL          = 10.0             # turns/s — must match receiver
THROTTLE_STEP    = 0.1              # per Q/E keypress
TURN_FACTOR      = 0.6              # how much A/D reduces opposite side

TELEM_PACKET_LEN = 22               # bytes

# ─── LOGGING (file only — curses owns the terminal) ────────────────
log_path = os.path.join(os.path.dirname(__file__), "transmitter.log")
logging.basicConfig(
    filename=log_path,
    level=logging.DEBUG,
    format="%(asctime)s [TX] %(levelname)s %(message)s"
)
log = logging.getLogger(__name__)

# ─── CRC8 (poly 0x07) ──────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

# ─── PACKET BUILDER ────────────────────────────────────────────────
def vel_to_i8(vel: float) -> int:
    """Scale [-MAX_VEL..MAX_VEL] → [-127..127]."""
    scaled = int((vel / MAX_VEL) * 127)
    return max(-127, min(127, scaled))

def build_packet(seq: int, left_vel: float, right_vel: float) -> bytes:
    x_i8 = vel_to_i8(left_vel)
    z_i8 = vel_to_i8(right_vel)
    x_b = struct.pack('b', x_i8)[0]
    z_b = struct.pack('b', z_i8)[0]
    body = bytes([0xAA, 0xBB, (seq >> 8) & 0xFF, seq & 0xFF,
                  x_b, z_b, 0xFF])
    return body + bytes([crc8(body)])

# ─── SHARED DRIVE STATE ────────────────────────────────────────────
class DriveState:
    def __init__(self):
        self.lock        = threading.Lock()
        self.forward     = 0.0    # -1.0 .. 1.0
        self.turn        = 0.0    # -1.0 .. 1.0
        self.throttle    = 0.5    # 0.0 .. 1.0
        self.estop       = False
        self.seq         = 0
        self.packets_sent = 0

    def compute_vels(self):
        with self.lock:
            if self.estop:
                return 0.0, 0.0
            fwd = self.forward
            trn = self.turn
            thr = self.throttle

        left  = (fwd + trn * TURN_FACTOR) * thr * MAX_VEL
        right = (fwd - trn * TURN_FACTOR) * thr * MAX_VEL
        left  = max(-MAX_VEL, min(MAX_VEL, left))
        right = max(-MAX_VEL, min(MAX_VEL, right))
        return left, right

    def next_seq(self):
        with self.lock:
            s = self.seq
            self.seq = (self.seq + 1) & 0xFFFF
            self.packets_sent += 1
            return s

# ─── TELEMETRY STATE ───────────────────────────────────────────────
class TelemetryStore:
    def __init__(self):
        self.lock    = threading.Lock()
        self.axes    = {}          # (dev, axis) → dict of latest readings
        self.packets = 0
        self.last_rx = None

    def update(self, dev, axis, input_vel, pos_est, iq, vbus):
        with self.lock:
            self.axes[(dev, axis)] = {
                "input_vel": input_vel,
                "pos_est":   pos_est,
                "iq":        iq,
                "vbus":      vbus,
                "ts":        time.time(),
            }
            self.packets += 1
            self.last_rx = time.time()

    def snapshot(self):
        with self.lock:
            return dict(self.axes), self.packets, self.last_rx

# ─── SENDER THREAD ─────────────────────────────────────────────────
def sender_thread(ds: DriveState, sock: socket.socket):
    interval = 1.0 / SEND_RATE
    while True:
        t0 = time.time()
        left, right = ds.compute_vels()
        seq = ds.next_seq()
        pkt = build_packet(seq, left, right)
        try:
            sock.sendto(pkt, (JETSON_IP, JETSON_PORT))
        except Exception as e:
            log.error(f"Send error: {e}")
        elapsed = time.time() - t0
        sleep = interval - elapsed
        if sleep > 0:
            time.sleep(sleep)

# ─── RECEIVER THREAD ───────────────────────────────────────────────
def receiver_thread(ts: TelemetryStore, sock: socket.socket):
    while True:
        try:
            data, _ = sock.recvfrom(64)
        except socket.timeout:
            continue
        except Exception as e:
            log.error(f"Recv error: {e}")
            continue

        if len(data) != TELEM_PACKET_LEN:
            continue
        if data[0] != 0xC1:
            continue
        if crc8(data[:-1]) != data[-1]:
            log.debug("Telemetry CRC mismatch")
            continue

        try:
            _, dev, axis, seq, iv, pos, iq, vbus = struct.unpack('>BBBHffff', data[:-1])
            ts.update(dev, axis, iv, pos, iq, vbus)
        except struct.error as e:
            log.debug(f"Telemetry unpack error: {e}")

# ─── CURSES UI ─────────────────────────────────────────────────────
KEY_QUIT    = (27, ord('q'))   # ESC or q
KEY_W       = ord('w')
KEY_S       = ord('s')
KEY_A       = ord('a')
KEY_D       = ord('d')
KEY_Q       = ord('q')
KEY_E       = ord('e')
KEY_SPACE   = ord(' ')
KEY_R       = ord('r')
KEY_THROTTLE_UP   = ord('e')
KEY_THROTTLE_DOWN = ord('q')

def bar(val, lo, hi, width=20, fill='█', empty='░') -> str:
    """Render a horizontal bar for val in [lo..hi]."""
    norm  = (val - lo) / (hi - lo) if hi != lo else 0
    norm  = max(0.0, min(1.0, norm))
    filled = int(norm * width)
    return fill * filled + empty * (width - filled)

def signed_bar(val, maxv, width=20) -> str:
    """Render a centered bar for signed value in [-maxv..maxv]."""
    half = width // 2
    norm = val / maxv if maxv else 0
    norm = max(-1.0, min(1.0, norm))
    if norm >= 0:
        filled = int(norm * half)
        return '░' * half + '█' * filled + '░' * (half - filled)
    else:
        filled = int(-norm * half)
        return '░' * (half - filled) + '█' * filled + '░' * half

def draw_ui(stdscr, ds: DriveState, ts: TelemetryStore):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)   # ms — refresh rate

    # Colors
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)   # active / ok
    curses.init_pair(2, curses.COLOR_RED,    -1)   # estop / error
    curses.init_pair(3, curses.COLOR_YELLOW, -1)   # warning / label
    curses.init_pair(4, curses.COLOR_CYAN,   -1)   # telemetry values
    curses.init_pair(5, curses.COLOR_WHITE,  -1)   # normal
    curses.init_pair(6, curses.COLOR_MAGENTA,-1)   # header

    C_OK    = curses.color_pair(1) | curses.A_BOLD
    C_STOP  = curses.color_pair(2) | curses.A_BOLD
    C_LABEL = curses.color_pair(3)
    C_VAL   = curses.color_pair(4)
    C_NORM  = curses.color_pair(5)
    C_HEAD  = curses.color_pair(6) | curses.A_BOLD

    # Key state — track which keys are currently held
    held = set()

    def clamp(v, lo, hi): return max(lo, min(hi, v))

    while True:
        key = stdscr.getch()

        # ── Key down ──
        if key != -1:
            # Quit
            if key == 27:   # ESC
                return

            # E-stop
            elif key == KEY_SPACE:
                with ds.lock:
                    ds.estop = True

            # Release e-stop
            elif key == ord('r'):
                with ds.lock:
                    ds.estop = False

            # Throttle
            elif key == KEY_THROTTLE_UP:
                with ds.lock:
                    ds.throttle = clamp(ds.throttle + THROTTLE_STEP, 0.0, 1.0)
            elif key == KEY_THROTTLE_DOWN:
                with ds.lock:
                    ds.throttle = clamp(ds.throttle - THROTTLE_STEP, 0.0, 1.0)

            # Movement — track held keys
            elif key in (KEY_W, KEY_S, KEY_A, KEY_D):
                held.add(key)

        # ── Compute forward/turn from held keys ──
        fwd  = 0.0
        trn  = 0.0
        if KEY_W in held: fwd  += 1.0
        if KEY_S in held: fwd  -= 1.0
        if KEY_A in held: trn  -= 1.0
        if KEY_D in held: trn  += 1.0

        # Simulate key release — re-read held keys every frame
        # curses nodelay means we only see keydown, so we use a
        # simple approach: clear held if no movement key pressed
        # this frame and re-detect next frame.
        # For smoother control we read all pending keys in one pass.
        held.clear()
        stdscr.nodelay(True)
        while True:
            k2 = stdscr.getch()
            if k2 == -1:
                break
            if k2 in (KEY_W, KEY_S, KEY_A, KEY_D):
                held.add(k2)
            elif k2 == KEY_SPACE:
                with ds.lock:
                    ds.estop = True
            elif k2 == ord('r'):
                with ds.lock:
                    ds.estop = False
            elif k2 == KEY_THROTTLE_UP:
                with ds.lock:
                    ds.throttle = clamp(ds.throttle + THROTTLE_STEP, 0.0, 1.0)
            elif k2 == KEY_THROTTLE_DOWN:
                with ds.lock:
                    ds.throttle = clamp(ds.throttle - THROTTLE_STEP, 0.0, 1.0)
            elif k2 == 27:
                return

        # Recompute from held after draining
        fwd = 0.0; trn = 0.0
        if KEY_W in held: fwd += 1.0
        if KEY_S in held: fwd -= 1.0
        if KEY_A in held: trn -= 1.0
        if KEY_D in held: trn += 1.0

        with ds.lock:
            ds.forward = fwd
            ds.turn    = trn

        # ── Snapshot state for drawing ──
        with ds.lock:
            estop    = ds.estop
            throttle = ds.throttle
            forward  = ds.forward
            turn     = ds.turn
            sent     = ds.packets_sent

        left_vel, right_vel = ds.compute_vels()
        telem_snap, telem_pkts, last_rx = ts.snapshot()

        now = time.time()
        link_age = (now - last_rx) if last_rx else None
        link_ok  = link_age is not None and link_age < 2.0

        # ── Draw ──
        stdscr.erase()
        rows, cols = stdscr.getmaxyx()
        row = 0

        def put(r, c, text, attr=C_NORM):
            try:
                stdscr.addstr(r, c, text, attr)
            except curses.error:
                pass

        # Header
        put(row, 0, "━" * min(cols - 1, 60), C_HEAD); row += 1
        put(row, 2, "  ODrive Transmitter  ", C_HEAD)
        put(row, 26, f"Jetson: {JETSON_IP}:{JETSON_PORT}", C_LABEL); row += 1
        put(row, 0, "━" * min(cols - 1, 60), C_HEAD); row += 1

        # E-stop banner
        if estop:
            put(row, 2, "██  E-STOP ACTIVE — press R to release  ██", C_STOP)
        else:
            put(row, 2, "   RUNNING — SPACE to e-stop               ", C_OK)
        row += 2

        # Controls section
        put(row, 2, "CONTROLS", C_LABEL | curses.A_UNDERLINE); row += 1
        put(row, 4, "W/S", C_VAL);  put(row, 8,  "Forward / Reverse", C_NORM); row += 1
        put(row, 4, "A/D", C_VAL);  put(row, 8,  "Turn left / right (differential)", C_NORM); row += 1
        put(row, 4, "Q/E", C_VAL);  put(row, 8,  "Throttle  −/+", C_NORM); row += 1
        put(row, 4, "SPC", C_VAL);  put(row, 8,  "Emergency stop", C_NORM); row += 1
        put(row, 4, "R  ", C_VAL);  put(row, 8,  "Release e-stop", C_NORM); row += 1
        put(row, 4, "ESC", C_VAL);  put(row, 8,  "Quit", C_NORM); row += 2

        # Drive state
        put(row, 2, "DRIVE STATE", C_LABEL | curses.A_UNDERLINE); row += 1

        throttle_pct = int(throttle * 100)
        put(row, 4, f"Throttle : {throttle_pct:3d}%  [", C_NORM)
        put(row, 22, bar(throttle, 0, 1, width=20), C_OK if throttle > 0.1 else C_LABEL)
        put(row, 42, "]", C_NORM); row += 1

        put(row, 4, f"Forward  : {forward:+.1f}  ", C_NORM)
        put(row, 22, signed_bar(forward, 1.0), C_VAL); row += 1

        put(row, 4, f"Turn     : {turn:+.1f}  ", C_NORM)
        put(row, 22, signed_bar(turn, 1.0), C_VAL); row += 2

        # Computed velocities
        put(row, 2, "OUTPUT VELOCITY", C_LABEL | curses.A_UNDERLINE); row += 1
        lbar = signed_bar(left_vel, MAX_VEL)
        rbar = signed_bar(right_vel, MAX_VEL)
        lc = C_OK if abs(left_vel) > 0.1 else C_NORM
        rc = C_OK if abs(right_vel) > 0.1 else C_NORM
        put(row, 4, f"Left  (axis0): {left_vel:+6.2f} t/s  [", C_NORM)
        put(row, 32, lbar, lc)
        put(row, 52, "]", C_NORM); row += 1
        put(row, 4, f"Right (axis1): {right_vel:+6.2f} t/s  [", C_NORM)
        put(row, 32, rbar, rc)
        put(row, 52, "]", C_NORM); row += 2

        # Link stats
        put(row, 2, "LINK", C_LABEL | curses.A_UNDERLINE); row += 1
        put(row, 4, f"Packets sent : {sent}", C_NORM); row += 1
        put(row, 4, f"Telem pkts   : {telem_pkts}", C_NORM); row += 1
        if link_ok:
            put(row, 4, f"Link         : OK  ({link_age*1000:.0f} ms ago)", C_OK)
        elif last_rx is None:
            put(row, 4, "Link         : waiting for telemetry...", C_LABEL)
        else:
            put(row, 4, f"Link         : LOST ({link_age:.1f}s ago)", C_STOP)
        row += 2

        # Telemetry
        put(row, 2, "TELEMETRY  (from Jetson)", C_LABEL | curses.A_UNDERLINE); row += 1

        if not telem_snap:
            put(row, 4, "No telemetry received yet.", C_LABEL); row += 1
        else:
            for (dev, axis), data in sorted(telem_snap.items()):
                age = now - data["ts"]
                stale = age > 1.0
                tag = f"odrv{dev}/axis{axis}"
                color = C_LABEL if stale else C_VAL

                put(row, 4,  f"{tag}", C_LABEL | curses.A_BOLD)
                put(row, 18, f"{'[STALE]' if stale else ''}", C_STOP if stale else C_NORM)
                row += 1

                put(row, 6,  f"Velocity    : {data['input_vel']:+7.3f} t/s", color); row += 1
                put(row, 6,  f"Position    : {data['pos_est']:+10.3f} turns", color); row += 1
                put(row, 6,  f"Current Iq  : {data['iq']:+7.3f} A", color); row += 1

                vbus = data['vbus']
                vbus_color = C_STOP if vbus < 10.0 else (C_LABEL if vbus < 11.0 else color)
                put(row, 6,  f"Vbus        : {vbus:6.2f} V", vbus_color); row += 1
                row += 1

        # Footer
        put(min(row, rows - 2), 0, "━" * min(cols - 1, 60), C_HEAD)

        stdscr.refresh()
        time.sleep(0.02)   # ~50 fps UI refresh

# ─── MAIN ──────────────────────────────────────────────────────────
def main():
    log.info("ODrive transmitter starting")

    ds = DriveState()
    ts = TelemetryStore()

    # Shared UDP socket for sending
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_sock.setblocking(False)

    # Dedicated socket for receiving telemetry
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("0.0.0.0", BIND_PORT))
    recv_sock.settimeout(0.5)

    threading.Thread(target=sender_thread,   args=(ds, send_sock), daemon=True).start()
    threading.Thread(target=receiver_thread, args=(ts, recv_sock), daemon=True).start()

    log.info(f"Sending to {JETSON_IP}:{JETSON_PORT}, receiving on :{BIND_PORT}")

    try:
        curses.wrapper(draw_ui, ds, ts)
    except KeyboardInterrupt:
        pass
    finally:
        # Zero velocities on exit
        try:
            pkt = build_packet(0, 0.0, 0.0)
            for _ in range(5):
                send_sock.sendto(pkt, (JETSON_IP, JETSON_PORT))
                time.sleep(0.02)
        except Exception:
            pass
        log.info("Transmitter exited cleanly")
        print("Transmitter stopped. Motors zeroed.")

if __name__ == "__main__":
    main()
