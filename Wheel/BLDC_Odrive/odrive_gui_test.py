#!/usr/bin/env python3
"""
ODrive GUI Velocity Controller
────────────────────────────────────────────────────────────────
Dark navy-blue / orange themed dashboard with live Matplotlib
plotting and large, readable telemetry readouts.

Keys (when the window has focus):
  W  →  Forward  (at set velocity magnitude)
  S  →  Reverse  (negative of set velocity magnitude)
  Q  →  Quit and idle motors
  Any other key / no key held → Stop (0 turn/s)

Velocity input field:
  Enter any value between -5 and 5 turns/s.
  Values above 5 are clamped to 5. Negatives are accepted.

Position calibration:
  The first position reading (averaged over ~0.15 s) is stored as
  an offset and subtracted from all subsequent readings so the
  display always starts at 0.

Run:
  python3 odrive_gui.py
"""

import tkinter as tk
from tkinter import font as tkfont
import threading
import time
import collections
import warnings
import logging

# Suppress matplotlib font-not-found warnings
warnings.filterwarnings("ignore", message="findfont")
logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

# ── Simulation switch ──────────────────────────────────────────
SIMULATE = False         # True = no hardware needed

# ── Control constants ──────────────────────────────────────────
VEL_MAX       =  5.0    # hard clamp (turns/s)
VELOCITY_STOP =  0.0
REFRESH_HZ    = 20
HISTORY_LEN   = 200
PLOT_WINDOW_S = 10.0
POS_CAL_SECS  = 0.15    # seconds to collect samples for position zero

# ── Font — DejaVu Sans Mono is guaranteed on Ubuntu/Debian ────
MONO_FONT = "DejaVu Sans Mono"

# ── Colour palette ─────────────────────────────────────────────
BG_DEEP  = "#0A0E1A"
BG_CARD  = "#0F1628"
BG_PANEL = "#111D38"
ACCENT   = "#FF6B00"
ACCENT2  = "#FF9A45"
TEAL     = "#00D4FF"
LIME     = "#39FF14"
FG_WHITE = "#F0F4FF"
FG_GRAY  = "#5A6A8A"
FG_DIM   = "#2A3A5A"
RED_ERR  = "#FF3355"
GREEN_OK = "#33FF88"

# ── Matplotlib setup ───────────────────────────────────────────
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np

plt.rcParams.update({
    "font.family":    "monospace",
    "font.monospace": [MONO_FONT, "Liberation Mono", "FreeMono", "monospace"],
    "text.color":     FG_WHITE,
    "axes.facecolor": BG_CARD,
    "figure.facecolor": BG_DEEP,
    "axes.edgecolor": FG_DIM,
    "axes.labelcolor": FG_GRAY,
    "xtick.color":    FG_GRAY,
    "ytick.color":    FG_GRAY,
    "grid.color":     FG_DIM,
    "grid.alpha":     0.5,
    "lines.linewidth": 2.0,
})


# ══════════════════════════════════════════════════════════════
#  Simulation layer
# ══════════════════════════════════════════════════════════════
class SimAxis:
    def __init__(self, init_pos=0.0, noise=0.008):
        self._pos = init_pos
        self._vel = 0.0
        self._cmd = 0.0
        self._t   = time.time()
        self._noise = noise
        self.error  = 0

    def step(self):
        now = time.time(); dt = now - self._t; self._t = now
        self._vel += (self._cmd - self._vel) * min(1.0, dt * 4.0)
        self._pos += self._vel * dt

    def set_cmd(self, v): self._cmd = v

    @property
    def pos_estimate(self):
        return self._pos + np.random.normal(0, self._noise)

    @property
    def vel_estimate(self):
        return self._vel + np.random.normal(0, self._noise * 2)

    @property
    def Iq_measured(self):
        return (abs(self._vel) * 1.8 + abs(self._cmd - self._vel) * 0.6
                + np.random.normal(0, 0.05))


class SimODrive:
    def __init__(self):
        # Non-zero starting positions to exercise calibration
        self.axis0 = SimAxis(init_pos=3.75)
        self.axis1 = SimAxis(init_pos=-1.22, noise=0.006)
        self.vbus_voltage = 24.0
        self._last_v = time.time()

    def set_velocity(self, v):
        self.axis0.set_cmd(v)
        self.axis1.set_cmd(v)

    def step(self):
        self.axis0.step()
        self.axis1.step()
        if time.time() - self._last_v > 1.0:
            self.vbus_voltage = 24.0 + np.random.normal(0, 0.05)
            self._last_v = time.time()

    def idle(self):
        self.set_velocity(0.0)


# ══════════════════════════════════════════════════════════════
#  ODrive hardware adapter
# ══════════════════════════════════════════════════════════════
def make_odrive():
    if SIMULATE:
        print("[SIMULATION MODE]")
        return SimODrive()

    import odrive
    from odrive.enums import (
        CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP,
        AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
    )
    print("Searching for ODrive …")
    odrv = odrive.find_any()
    print(f"Connected — Serial: {odrv.serial_number}  Vbus: {odrv.vbus_voltage:.2f} V")

    for idx, ax in enumerate((odrv.axis0, odrv.axis1)):
        lbl = f"axis{idx}"
        print(f"  [{lbl}] → IDLE …")
        ax.requested_state = AXIS_STATE_IDLE
        time.sleep(0.3)
        try:
            ax.error = 0; ax.motor.error = 0
            ax.encoder.error = 0; ax.controller.error = 0
        except Exception as e:
            print(f"  [{lbl}] error-clear: {e}")
        ax.controller.input_vel = 0.0
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        ax.controller.config.input_mode   = INPUT_MODE_VEL_RAMP
        ax.controller.config.vel_ramp_rate = 2.0
        print(f"  [{lbl}] → CLOSED_LOOP_CONTROL …")
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.8)
        ok = ax.current_state == 8
        print(f"  [{lbl}] state={ax.current_state} ({'OK' if ok else 'FAILED'})  error={ax.error}")

    return odrv


def get_axis_info(odrv, axis_id):
    ax = odrv.axis0 if axis_id == 0 else odrv.axis1
    if SIMULATE:
        return {"pos": ax.pos_estimate, "vel": ax.vel_estimate,
                "current": ax.Iq_measured, "error": ax.error}
    try:
        return {"pos": ax.encoder.pos_estimate, "vel": ax.encoder.vel_estimate,
                "current": ax.motor.current_control.Iq_measured, "error": ax.error}
    except Exception:
        return {"pos": 0.0, "vel": 0.0, "current": 0.0, "error": -1}


def set_velocity(odrv, v):
    if SIMULATE:
        odrv.set_velocity(v); return
    try: odrv.axis0.controller.input_vel = v
    except Exception as e: print(f"[set_vel ax0] {e}")
    try: odrv.axis1.controller.input_vel = v
    except Exception as e: print(f"[set_vel ax1] {e}")


def idle_motors(odrv):
    if SIMULATE:
        odrv.idle(); return
    from odrive.enums import AXIS_STATE_IDLE
    try: odrv.axis0.requested_state = AXIS_STATE_IDLE
    except Exception: pass
    try: odrv.axis1.requested_state = AXIS_STATE_IDLE
    except Exception: pass


# ══════════════════════════════════════════════════════════════
#  GUI Application
# ══════════════════════════════════════════════════════════════
class ODriveApp:
    def __init__(self, root, odrv):
        self.root      = root
        self.odrv      = odrv
        self.cmd_vel   = 0.0
        self.vel_mag   = 1.0        # magnitude controlled by input field
        self.running   = True
        self.start_t   = time.time()
        self.keys_held = set()

        # Position calibration state
        self._cal_done    = False
        self._cal_samples = {0: [], 1: []}
        self._pos_offset  = {0: 0.0, 1: 0.0}

        N = HISTORY_LEN
        self.t_buf    = collections.deque(maxlen=N)
        self.vel0_buf = collections.deque(maxlen=N)
        self.vel1_buf = collections.deque(maxlen=N)
        self.pos0_buf = collections.deque(maxlen=N)
        self.pos1_buf = collections.deque(maxlen=N)
        self.cur0_buf = collections.deque(maxlen=N)
        self.cur1_buf = collections.deque(maxlen=N)
        self.cmd_buf  = collections.deque(maxlen=N)

        self._build_ui()
        self._bind_keys()
        self._start_data_thread()
        self._start_animation()

    # ── font helper ────────────────────────────────────────────
    def _f(self, size, weight="normal"):
        return tkfont.Font(family=MONO_FONT, size=size, weight=weight)

    # ── UI construction ────────────────────────────────────────
    def _build_ui(self):
        r = self.root
        r.title("ODrive  ⟁  Velocity Controller")
        r.configure(bg=BG_DEEP)
        r.geometry("1300x840")
        r.minsize(1100, 720)

        # Header
        hdr = tk.Frame(r, bg=BG_DEEP, pady=8)
        hdr.pack(side=tk.TOP, fill=tk.X, padx=20)
        tk.Label(hdr, text="⟁ ODRIVE", font=self._f(22, "bold"),
                 bg=BG_DEEP, fg=ACCENT).pack(side=tk.LEFT, padx=(0, 14))
        tk.Label(hdr, text="VELOCITY CONTROLLER", font=self._f(13),
                 bg=BG_DEEP, fg=FG_GRAY).pack(side=tk.LEFT)
        mode_txt = "◉ SIMULATION" if SIMULATE else "◉ HARDWARE"
        mode_col = ACCENT2 if SIMULATE else GREEN_OK
        tk.Label(hdr, text=mode_txt, font=self._f(11, "bold"),
                 bg=BG_DEEP, fg=mode_col).pack(side=tk.RIGHT)
        self.timer_var = tk.StringVar(value="00:00")
        tk.Label(hdr, textvariable=self.timer_var, font=self._f(14),
                 bg=BG_DEEP, fg=FG_GRAY).pack(side=tk.RIGHT, padx=20)

        tk.Frame(r, bg=ACCENT, height=2).pack(fill=tk.X)

        body = tk.Frame(r, bg=BG_DEEP)
        body.pack(fill=tk.BOTH, expand=True)

        left = tk.Frame(body, bg=BG_DEEP, width=380)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(14, 6), pady=10)
        left.pack_propagate(False)

        right = tk.Frame(body, bg=BG_DEEP)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 14), pady=10)

        self._build_control_panel(left)
        self._build_telemetry(left)
        self._build_plots(right)

    def _card(self, parent, title, pady_inner=10):
        outer = tk.Frame(parent, bg=BG_CARD,
                         highlightbackground=FG_DIM, highlightthickness=1)
        outer.pack(fill=tk.X, pady=5)
        tk.Label(outer, text=title, font=self._f(9, "bold"),
                 bg=BG_CARD, fg=FG_GRAY).pack(anchor=tk.W, padx=12, pady=(8, 0))
        inner = tk.Frame(outer, bg=BG_CARD)
        inner.pack(fill=tk.X, padx=12, pady=(4, pady_inner))
        return inner

    # ── Control panel (keys + velocity input) ──────────────────
    def _build_control_panel(self, parent):
        frame = self._card(parent, "KEY CONTROL  +  VELOCITY INPUT")

        # W / S key tiles
        key_row = tk.Frame(frame, bg=BG_CARD)
        key_row.pack(anchor=tk.W, pady=6)
        self.w_btn = tk.Label(key_row, text=" W ", font=self._f(26, "bold"),
                              bg=FG_DIM, fg=FG_WHITE, padx=4, pady=2)
        self.w_btn.pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(key_row, text="FWD", font=self._f(10),
                 bg=BG_CARD, fg=FG_GRAY).pack(side=tk.LEFT, padx=(0, 20))
        self.s_btn = tk.Label(key_row, text=" S ", font=self._f(26, "bold"),
                              bg=FG_DIM, fg=FG_WHITE, padx=4, pady=2)
        self.s_btn.pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(key_row, text="REV", font=self._f(10),
                 bg=BG_CARD, fg=FG_GRAY).pack(side=tk.LEFT)

        # Velocity input row
        tk.Label(frame, text="Velocity magnitude  (−5 … +5 turn/s):",
                 font=self._f(9), bg=BG_CARD, fg=FG_GRAY).pack(anchor=tk.W, pady=(6, 2))

        inp_row = tk.Frame(frame, bg=BG_CARD)
        inp_row.pack(fill=tk.X, pady=(0, 4))

        self.vel_input_var = tk.StringVar(value="1.00")
        self._vel_entry = tk.Entry(
            inp_row, textvariable=self.vel_input_var,
            font=self._f(18, "bold"), width=7,
            bg=BG_PANEL, fg=ACCENT, insertbackground=ACCENT,
            relief=tk.FLAT, bd=0,
            highlightbackground=ACCENT, highlightthickness=1,
        )
        self._vel_entry.pack(side=tk.LEFT, padx=(0, 8), ipady=4)

        tk.Button(
            inp_row, text="SET", font=self._f(11, "bold"),
            bg=ACCENT, fg=BG_DEEP,
            activebackground=ACCENT2, activeforeground=BG_DEEP,
            relief=tk.FLAT, bd=0, padx=12, pady=4, cursor="hand2",
            command=self._apply_vel_input,
        ).pack(side=tk.LEFT)

        self._vel_hint = tk.Label(inp_row, text="", font=self._f(9),
                                  bg=BG_CARD, fg=RED_ERR)
        self._vel_hint.pack(side=tk.LEFT, padx=8)

        tk.Label(frame, text="Values > 5 clamped to 5  |  Enter or SET to apply",
                 font=self._f(8), bg=BG_CARD, fg=FG_DIM).pack(anchor=tk.W, pady=(0, 4))

        self._vel_entry.bind("<Return>", lambda _: self._apply_vel_input())

        # Big commanded velocity readout
        self.vel_var = tk.StringVar(value="+0.00")
        self.vel_label = tk.Label(frame, textvariable=self.vel_var,
                                  font=self._f(46, "bold"), bg=BG_CARD, fg=ACCENT)
        self.vel_label.pack(anchor=tk.W, pady=(6, 0))
        tk.Label(frame, text="COMMANDED  turn/s", font=self._f(9),
                 bg=BG_CARD, fg=FG_GRAY).pack(anchor=tk.W)

        # Velocity bar
        self.vel_canvas = tk.Canvas(frame, bg=BG_PANEL, height=22,
                                    highlightthickness=0)
        self.vel_canvas.pack(fill=tk.X, pady=(10, 4))
        self.vel_canvas.bind("<Configure>", self._draw_vel_bar)

        tk.Label(frame, text="Q → Quit & idle motors", font=self._f(9),
                 bg=BG_CARD, fg=FG_DIM).pack(anchor=tk.W, pady=(4, 0))

    def _apply_vel_input(self):
        raw = self.vel_input_var.get().strip()
        try:
            val = float(raw)
        except ValueError:
            self._vel_hint.config(text="⚠ not a number", fg=RED_ERR)
            return
        clamped = abs(val) > VEL_MAX
        if clamped:
            val = VEL_MAX * (1 if val >= 0 else -1)
        self.vel_mag = abs(val)
        self.vel_input_var.set(f"{self.vel_mag:.2f}")
        if clamped:
            self._vel_hint.config(text=f"clamped → {self.vel_mag:.1f}", fg=ACCENT2)
        else:
            self._vel_hint.config(text="✔ applied", fg=GREEN_OK)
        self._update_cmd()

    def _draw_vel_bar(self, event=None):
        c = self.vel_canvas
        w = c.winfo_width(); h = c.winfo_height()
        if w < 2: return
        c.delete("all")
        mid = w // 2
        ratio = max(-1.0, min(1.0, self.cmd_vel / VEL_MAX))
        fill_w = int(abs(ratio) * mid)
        col = ACCENT if ratio >= 0 else RED_ERR
        if ratio >= 0:
            c.create_rectangle(mid, 0, mid + fill_w, h, fill=col, outline="")
        else:
            c.create_rectangle(mid - fill_w, 0, mid, h, fill=col, outline="")
        c.create_line(mid, 0, mid, h, fill=FG_GRAY, width=1)

    # ── Telemetry panel ────────────────────────────────────────
    def _build_telemetry(self, parent):
        lf = self._f(9); vf = self._f(20, "bold"); uf = self._f(9)

        def telem_row(p, label, color, unit):
            row = tk.Frame(p, bg=BG_CARD); row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, font=lf, bg=BG_CARD,
                     fg=FG_GRAY, width=10, anchor=tk.W).pack(side=tk.LEFT)
            var = tk.StringVar(value="─────")
            tk.Label(row, textvariable=var, font=vf,
                     bg=BG_CARD, fg=color, anchor=tk.E).pack(side=tk.LEFT, padx=(4, 2))
            tk.Label(row, text=unit, font=uf,
                     bg=BG_CARD, fg=FG_GRAY).pack(side=tk.LEFT)
            return var

        def err_row(p):
            row = tk.Frame(p, bg=BG_CARD); row.pack(fill=tk.X, pady=2)
            tk.Label(row, text="Error", font=lf, bg=BG_CARD,
                     fg=FG_GRAY, width=10, anchor=tk.W).pack(side=tk.LEFT)
            var = tk.StringVar(value="OK")
            lbl = tk.Label(row, textvariable=var, font=self._f(14, "bold"),
                           bg=BG_CARD, fg=GREEN_OK)
            lbl.pack(side=tk.LEFT, padx=4)
            return lbl, var

        # Calibration status
        cal_f = self._card(parent, "POSITION CALIBRATION", pady_inner=6)
        self.cal_var = tk.StringVar(value="⏳ Acquiring zero offset …")
        tk.Label(cal_f, textvariable=self.cal_var, font=self._f(9, "bold"),
                 bg=BG_CARD, fg=ACCENT2).pack(anchor=tk.W)

        a0 = self._card(parent, "◈  AXIS 0", pady_inner=8)
        self.a0_pos = telem_row(a0, "Pos (cal)", TEAL,    "trn")
        self.a0_vel = telem_row(a0, "Velocity",  TEAL,    "t/s")
        self.a0_cur = telem_row(a0, "Current",   ACCENT2, "A")
        self.a0_err_lbl, self.a0_err_var = err_row(a0)

        a1 = self._card(parent, "◈  AXIS 1", pady_inner=8)
        self.a1_pos = telem_row(a1, "Pos (cal)", LIME,    "trn")
        self.a1_vel = telem_row(a1, "Velocity",  LIME,    "t/s")
        self.a1_cur = telem_row(a1, "Current",   ACCENT2, "A")
        self.a1_err_lbl, self.a1_err_var = err_row(a1)

        vbus_f = self._card(parent, "BUS VOLTAGE", pady_inner=8)
        self.vbus_var = tk.StringVar(value="─────")
        tk.Label(vbus_f, textvariable=self.vbus_var, font=self._f(24, "bold"),
                 bg=BG_CARD, fg=ACCENT2).pack(side=tk.LEFT)
        tk.Label(vbus_f, text=" V", font=self._f(12),
                 bg=BG_CARD, fg=FG_GRAY).pack(side=tk.LEFT)

    # ── Plots ──────────────────────────────────────────────────
    def _build_plots(self, parent):
        fig, axes = plt.subplots(3, 1, figsize=(7, 7),
                                 facecolor=BG_DEEP, tight_layout=True)
        self.fig = fig; self.axes = axes
        for ax, ttl in zip(axes, [
            "Velocity (turns/s)",
            "Position — calibrated (turns)",
            "Current (A)",
        ]):
            ax.set_facecolor(BG_CARD)
            ax.set_title(ttl, color=ACCENT, fontsize=10,
                         fontweight="bold", pad=4, loc="left")
            ax.grid(True, linestyle="--", linewidth=0.5)
            for sp in ax.spines.values(): sp.set_edgecolor(FG_DIM)
            ax.tick_params(colors=FG_GRAY, labelsize=8)

        leg_kw = dict(facecolor=BG_PANEL, edgecolor=FG_DIM, labelcolor=FG_WHITE)
        self.line_vel0, = axes[0].plot([], [], color=TEAL,   lw=2, label="Axis 0")
        self.line_vel1, = axes[0].plot([], [], color=LIME,   lw=2, label="Axis 1")
        self.line_cmd,  = axes[0].plot([], [], color=ACCENT, lw=1.5, ls="--", label="Cmd")
        axes[0].legend(loc="upper right", fontsize=8, **leg_kw)

        self.line_pos0, = axes[1].plot([], [], color=TEAL, lw=2, label="Axis 0")
        self.line_pos1, = axes[1].plot([], [], color=LIME, lw=2, label="Axis 1")
        axes[1].legend(loc="upper right", fontsize=8, **leg_kw)

        self.line_cur0, = axes[2].plot([], [], color=ACCENT,  lw=2, label="Axis 0")
        self.line_cur1, = axes[2].plot([], [], color=ACCENT2, lw=2, label="Axis 1")
        axes[2].legend(loc="upper right", fontsize=8, **leg_kw)

        self.canvas = FigureCanvasTkAgg(fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ── Key bindings ───────────────────────────────────────────
    def _bind_keys(self):
        self.root.bind("<KeyPress>",   self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    def _on_key_press(self, event):
        if event.widget is self._vel_entry:
            return                          # let the entry handle its own keys
        k = event.keysym.lower()
        if k == "q": self._quit(); return
        self.keys_held.add(k)
        self._update_cmd()

    def _on_key_release(self, event):
        if event.widget is self._vel_entry:
            return
        self.keys_held.discard(event.keysym.lower())
        self._update_cmd()

    def _update_cmd(self):
        w = "w" in self.keys_held
        s = "s" in self.keys_held
        if   w and not s: self.cmd_vel = +self.vel_mag
        elif s and not w: self.cmd_vel = -self.vel_mag
        else:             self.cmd_vel =  0.0

    # ── Position calibration ───────────────────────────────────
    def _calibrate_pos(self, axis_id, raw_pos):
        """
        Collect raw position samples for the first POS_CAL_SECS seconds.
        Once both axes have samples, compute the mean offset for each
        and subtract it from every subsequent reading.
        """
        if not self._cal_done:
            self._cal_samples[axis_id].append(raw_pos)
            elapsed = time.time() - self.start_t
            if (elapsed >= POS_CAL_SECS
                    and self._cal_samples[0]
                    and self._cal_samples[1]):
                self._pos_offset[0] = (sum(self._cal_samples[0])
                                       / len(self._cal_samples[0]))
                self._pos_offset[1] = (sum(self._cal_samples[1])
                                       / len(self._cal_samples[1]))
                self._cal_done = True
                self.root.after(0, self._mark_cal_done)
        return raw_pos - self._pos_offset[axis_id]

    def _mark_cal_done(self):
        o0 = self._pos_offset[0]; o1 = self._pos_offset[1]
        self.cal_var.set(f"✔ Zeroed   ax0={o0:+.3f}  ax1={o1:+.3f}")

    # ── Background data thread ─────────────────────────────────
    def _start_data_thread(self):
        threading.Thread(target=self._data_loop, daemon=True).start()

    def _data_loop(self):
        interval = 1.0 / REFRESH_HZ
        while self.running:
            t0 = time.time()
            try:
                if SIMULATE: self.odrv.step()
                set_velocity(self.odrv, self.cmd_vel)
                now  = time.time() - self.start_t
                a0   = get_axis_info(self.odrv, 0)
                a1   = get_axis_info(self.odrv, 1)
                vbus = self.odrv.vbus_voltage

                cal0 = self._calibrate_pos(0, a0["pos"])
                cal1 = self._calibrate_pos(1, a1["pos"])

                self.t_buf.append(now)
                self.vel0_buf.append(a0["vel"])
                self.vel1_buf.append(a1["vel"])
                self.pos0_buf.append(cal0)
                self.pos1_buf.append(cal1)
                self.cur0_buf.append(a0["current"])
                self.cur1_buf.append(a1["current"])
                self.cmd_buf.append(self.cmd_vel)

                self.root.after(0, self._update_labels,
                                a0, a1, cal0, cal1, vbus, now)
            except Exception as e:
                print(f"[data_loop] {e}")
            time.sleep(max(0.0, interval - (time.time() - t0)))

    def _update_labels(self, a0, a1, cal0, cal1, vbus, now):
        mins, secs = divmod(int(now), 60)
        self.timer_var.set(f"{mins:02d}:{secs:02d}")

        col = (GREEN_OK if self.cmd_vel > 0
               else RED_ERR if self.cmd_vel < 0 else FG_GRAY)
        sym = "" if self.cmd_vel < 0 else "+"
        self.vel_var.set(f"{sym}{self.cmd_vel:.2f}")
        self.vel_label.configure(fg=col)
        self._draw_vel_bar()

        if self.cmd_vel > 0:
            self.w_btn.configure(bg=GREEN_OK, fg=BG_DEEP)
            self.s_btn.configure(bg=FG_DIM,   fg=FG_WHITE)
        elif self.cmd_vel < 0:
            self.w_btn.configure(bg=FG_DIM,  fg=FG_WHITE)
            self.s_btn.configure(bg=RED_ERR, fg=BG_DEEP)
        else:
            self.w_btn.configure(bg=FG_DIM, fg=FG_WHITE)
            self.s_btn.configure(bg=FG_DIM, fg=FG_WHITE)

        def fmt(v): return f"{v:+.3f}"
        self.a0_pos.set(fmt(cal0));              self.a1_pos.set(fmt(cal1))
        self.a0_vel.set(fmt(a0["vel"]));         self.a1_vel.set(fmt(a1["vel"]))
        self.a0_cur.set(f"{a0['current']:.3f}"); self.a1_cur.set(f"{a1['current']:.3f}")

        for err_var, err_lbl, info in [
            (self.a0_err_var, self.a0_err_lbl, a0),
            (self.a1_err_var, self.a1_err_lbl, a1),
        ]:
            if info["error"]:
                err_var.set(str(info["error"])); err_lbl.configure(fg=RED_ERR)
            else:
                err_var.set("OK"); err_lbl.configure(fg=GREEN_OK)

        self.vbus_var.set(f"{vbus:.2f}")

    # ── Matplotlib animation ───────────────────────────────────
    def _start_animation(self):
        self.anim = FuncAnimation(
            self.fig, self._animate,
            interval=int(1000 / REFRESH_HZ),
            blit=False, cache_frame_data=False,
        )

    def _animate(self, _frame):
        if not self.t_buf: return
        t_arr = np.array(self.t_buf)
        now   = t_arr[-1]
        t_min = now - PLOT_WINDOW_S

        def refresh(ax, *pairs):
            for line, buf in pairs:
                line.set_data(t_arr, np.array(buf))
            ax.set_xlim(t_min, now + 0.2)
            all_v = [v for _, b in pairs for v in b]
            if all_v:
                mn, mx = min(all_v), max(all_v)
                pad = max(0.1, (mx - mn) * 0.15)
                ax.set_ylim(mn - pad, mx + pad)

        refresh(self.axes[0],
                (self.line_vel0, self.vel0_buf),
                (self.line_vel1, self.vel1_buf),
                (self.line_cmd,  self.cmd_buf))
        refresh(self.axes[1],
                (self.line_pos0, self.pos0_buf),
                (self.line_pos1, self.pos1_buf))
        refresh(self.axes[2],
                (self.line_cur0, self.cur0_buf),
                (self.line_cur1, self.cur1_buf))

        self.canvas.draw_idle()

    # ── Quit ───────────────────────────────────────────────────
    def _quit(self):
        self.running = False
        try:
            set_velocity(self.odrv, 0.0)
            time.sleep(0.15)
            idle_motors(self.odrv)
        except Exception:
            pass
        self.root.destroy()


# ══════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════
def main():
    odrv = make_odrive()
    root = tk.Tk()
    ODriveApp(root, odrv)
    root.mainloop()

if __name__ == "__main__":
    main()
