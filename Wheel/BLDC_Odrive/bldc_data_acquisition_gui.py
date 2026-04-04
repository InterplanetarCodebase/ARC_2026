#!/usr/bin/env python3
"""
BLDC Data Acquisition GUI — PyQtGraph with robust OpenGL fallback

The current graph panels use PyQtGraph's standard renderer for reliability
across Linux driver stacks. The 3D pose view still uses OpenGL via
pyqtgraph.opengl.GLViewWidget.

Install:
    pip install PyQt5 pyqtgraph PyOpenGL PyOpenGL_accelerate websockets numpy

Run:
    python3 bldc_data_acquisition_gui.py
"""
from __future__ import annotations

import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QPalette, QFont
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFrame,
    QGroupBox,
    QGridLayout, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton,
    QSlider, QStatusBar, QVBoxLayout, QWidget,
)

import pyqtgraph as pg
import pyqtgraph.opengl as gl

try:
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import WebSocketException
    WEBSOCKETS_OK = True
except Exception:
    ws_connect = None
    WebSocketException = Exception
    WEBSOCKETS_OK = False


# ─── Force OpenGL acceleration ────────────────────────────────────────────────
pg.setConfigOptions(
    # Keep 2D plots on the stable painter path; some systems fail to expose
    # required desktop OpenGL helpers for PlotCurveItem paintGL.
    useOpenGL=False,
    enableExperimental=False,
    antialias=True,
    foreground="#e2e8f0",
    background="#080b10",
)

# ─── Constants ────────────────────────────────────────────────────────────────
MOTOR_KEYS = ["FL", "RL", "FR", "RR"]
MOTOR_INDEX = {
    "FL": ("odrv0", "axis0"),
    "RL": ("odrv0", "axis1"),
    "FR": ("odrv1", "axis0"),
    "RR": ("odrv1", "axis1"),
}
MOTOR_COLORS_PG = {
    "FL": (255, 107,  53, 220),
    "RL": (255, 165,  82, 220),
    "FR": (  0, 200, 255, 220),
    "RR": (126, 232, 255, 220),
}

ODRV_KEYS = ["ODRV0", "ODRV1"]
ODRV_COLORS_PG = {
    "ODRV0": (255, 148, 70, 220),
    "ODRV1": (0, 210, 255, 220),
}

VALUE_FIELDS = [
    ("ibus", "Ibus (A)"),
    ("fet_temp", "FET Temp (C)"),
    ("axis_error", "Axis Error"),
    ("motor_error", "Motor Error"),
    ("encoder_error", "Encoder Error"),
    ("controller_error", "Controller Error"),
]

C_BG         = "#080b10"
C_SURFACE    = "#0d1320"
C_PANEL      = "#111827"
C_BORDER     = "#1e293b"
C_ACCENT     = "#ff6b35"
C_ACCENT2    = "#00c8ff"
C_TEXT       = "#e2e8f0"
C_TEXT_DIM   = "#64748b"
C_TEXT_MUTED = "#334155"
C_OK         = "#22c55e"
C_ERR        = "#ef4444"

MAX_POINTS = 3000
RENDER_FPS = 60
RENDER_MS  = 1000 // RENDER_FPS


# ─── Data types ───────────────────────────────────────────────────────────────
@dataclass
class TelemetryFrame:
    ts_host:   float
    ts_source: Optional[float]
    seq:       Optional[int]
    vbus0:     Optional[float]
    vbus1:     Optional[float]
    pos:       dict
    iq:        dict
    id_m:      dict
    ibus:      dict
    fet_temp:  dict
    axis_error: dict
    motor_error: dict
    encoder_error: dict
    controller_error: dict


# ─── WebSocket worker ─────────────────────────────────────────────────────────
class TelemetryWorker(QThread):
    frame_ready    = pyqtSignal(object)
    status_changed = pyqtSignal(str, str)

    def __init__(self, url: str, parent=None):
        super().__init__(parent)
        self._url  = url
        self._stop = False

    def stop(self):
        self._stop = True
        self.quit()

    @staticmethod
    def _f(v: Any) -> Optional[float]:
        try:   return float(v)
        except: return None

    @staticmethod
    def _i(v: Any) -> Optional[int]:
        try:
            if v is None:
                return None
            return int(v)
        except Exception:
            return None

    def _parse(self, data: dict) -> Optional[TelemetryFrame]:
        if not isinstance(data, dict) or data.get("type") != "odrive_telemetry":
            return None
        odrv0 = data.get("odrv0") or {}
        odrv1 = data.get("odrv1") or {}

        def axis_obj(board, axis):
            return (board.get("axes") or {}).get(axis, {})

        def pick_f(board, axis, field):
            return self._f(axis_obj(board, axis).get(field))

        def pick_i(board, axis, field):
            return self._i(axis_obj(board, axis).get(field))

        return TelemetryFrame(
            ts_host   = time.monotonic(),
            ts_source = self._f(data.get("t_monotonic")),
            seq       = int(data["seq"]) if isinstance(data.get("seq"), int) else None,
            vbus0     = self._f(odrv0.get("vbus_voltage")),
            vbus1     = self._f(odrv1.get("vbus_voltage")),
            pos  = {m: pick_f(odrv0 if b=="odrv0" else odrv1, a, "pos_estimate") for m,(b,a) in MOTOR_INDEX.items()},
            iq   = {m: pick_f(odrv0 if b=="odrv0" else odrv1, a, "iq_measured") for m,(b,a) in MOTOR_INDEX.items()},
            id_m = {m: pick_f(odrv0 if b=="odrv0" else odrv1, a, "id_measured") for m,(b,a) in MOTOR_INDEX.items()},
            ibus = {m: pick_f(odrv0 if b=="odrv0" else odrv1, a, "ibus") for m,(b,a) in MOTOR_INDEX.items()},
            fet_temp = {m: pick_f(odrv0 if b=="odrv0" else odrv1, a, "fet_thermistor_temp") for m,(b,a) in MOTOR_INDEX.items()},
            axis_error = {m: pick_i(odrv0 if b=="odrv0" else odrv1, a, "axis_error") for m,(b,a) in MOTOR_INDEX.items()},
            motor_error = {m: pick_i(odrv0 if b=="odrv0" else odrv1, a, "motor_error") for m,(b,a) in MOTOR_INDEX.items()},
            encoder_error = {m: pick_i(odrv0 if b=="odrv0" else odrv1, a, "encoder_error") for m,(b,a) in MOTOR_INDEX.items()},
            controller_error = {m: pick_i(odrv0 if b=="odrv0" else odrv1, a, "controller_error") for m,(b,a) in MOTOR_INDEX.items()},
        )

    def run(self):
        backoff = 1.0
        while not self._stop:
            self.status_changed.emit("info", f"Connecting → {self._url} …")
            ws = None
            try:
                ws = ws_connect(self._url, open_timeout=2.0, close_timeout=1.0)
                backoff = 1.0
                self.status_changed.emit("ok", f"Connected  {self._url}")
                while not self._stop:
                    try:
                        msg = ws.recv(timeout=0.1)
                    except TimeoutError:
                        continue
                    if isinstance(msg, bytes):
                        msg = msg.decode("utf-8", errors="ignore")
                    if not isinstance(msg, str):
                        continue
                    try:
                        f = self._parse(json.loads(msg))
                        if f:
                            self.frame_ready.emit(f)
                    except Exception:
                        pass
            except Exception as exc:
                self.status_changed.emit("err", f"WS: {str(exc)[:100]}")
            finally:
                try:
                    if ws: ws.close()
                except Exception:
                    pass

            if self._stop:
                break
            self.status_changed.emit("info", f"Retry in {backoff:.0f}s …")
            t0 = time.monotonic()
            while time.monotonic() - t0 < backoff and not self._stop:
                time.sleep(0.05)
            backoff = min(backoff * 1.5, 10.0)
        self.status_changed.emit("info", "Disconnected")


# ─── Recorder ─────────────────────────────────────────────────────────────────
class DataRecorder:
    def __init__(self):
        self._lock  = threading.Lock()
        self._fh    = None
        self.path   = ""
        self.active = False
        self.count  = 0

    def start(self, path: str, meta: dict) -> tuple[bool, str]:
        with self._lock:
            if self.active:
                return False, "Already recording"
            try:
                dirpath = os.path.dirname(path)
                if dirpath:
                    os.makedirs(dirpath, exist_ok=True)
                self._fh   = open(path, "w", encoding="utf-8", buffering=1)
                self.path  = path
                self.count = 0
                self._fh.write(json.dumps({
                    "type": "session_meta",
                    "created_unix": time.time(),
                    "meta": meta,
                }) + "\n")
                self.active = True
                return True, ""
            except Exception as exc:
                self._fh = None; self.active = False
                return False, str(exc)

    def append(self, rec: dict):
        with self._lock:
            if not self.active or self._fh is None:
                return
            try:
                self._fh.write(json.dumps(rec) + "\n")
                self.count += 1
            except Exception:
                self.active = False; self._fh = None

    def stop(self) -> tuple[bool, str]:
        with self._lock:
            try:
                if self._fh:
                    self._fh.flush(); self._fh.close()
            except Exception:
                pass
            saved, n = self.path, self.count
            self._fh = None; self.active = False
            return True, f"Saved {n} frames → {saved}"


# ─── Rolling numpy buffer ─────────────────────────────────────────────────────
class RingBuffer:
    """Fixed-size circular numpy buffer. O(1) append, O(n) ordered view."""
    def __init__(self, maxlen: int):
        self._buf  = np.zeros(maxlen, dtype=np.float32)
        self._n    = 0
        self._head = 0
        self._max  = maxlen

    def append(self, v: float):
        self._buf[self._head] = float(v) if v is not None else (
            self._buf[self._head - 1] if self._n else 0.0)
        self._head = (self._head + 1) % self._max
        if self._n < self._max:
            self._n += 1

    def array(self) -> np.ndarray:
        if self._n < self._max:
            return self._buf[:self._n].copy()
        return np.roll(self._buf, -self._head)

    def clear(self):
        self._buf[:] = 0; self._n = 0; self._head = 0

    def __len__(self): return self._n


# ─── Style helpers ────────────────────────────────────────────────────────────
def _lbl(text, color=C_TEXT, bold=False, size=9):
    w = QLabel(text)
    w.setStyleSheet(
        f"color:{color};font-weight:{'600' if bold else '400'};font-size:{size}pt;")
    return w

def _btn(text, accent=C_ACCENT, enabled=True):
    b = QPushButton(text)
    b.setEnabled(enabled)
    b.setStyleSheet(f"""
        QPushButton{{background:{C_PANEL};color:{C_TEXT};border:1px solid {C_BORDER};
            border-radius:4px;padding:5px 14px;font-size:9pt;font-weight:500;}}
        QPushButton:hover{{border-color:{accent};color:{accent};}}
        QPushButton:pressed{{background:{C_SURFACE};}}
        QPushButton:disabled{{color:{C_TEXT_MUTED};border-color:{C_TEXT_MUTED};}}""")
    return b

def _sep():
    f = QFrame(); f.setFrameShape(QFrame.VLine)
    f.setStyleSheet(f"color:{C_BORDER};"); return f


# ─── Main window ──────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLDC · ACQ  [GPU / OpenGL]")
        self.resize(1600, 960)

        # Pose integration params
        self.wheel_circum_m = math.pi * 0.20
        self.track_width_m  = 0.762
        self.right_sign     = -1.0
        self.fwd_sign       = -1.0
        self._prev_l = self._prev_r = None
        self._last_ts = 0.0
        self.x = self.y = self.yaw = 0.0

        # GPU-friendly ring buffers
        self._t_buf = RingBuffer(MAX_POINTS)
        self._px_buf: list[float] = [0.0]
        self._py_buf: list[float] = [0.0]
        self._metric: dict[str, dict[str, RingBuffer]] = {
            k: {o: RingBuffer(MAX_POINTS) for o in ODRV_KEYS}
            for k in ["iq", "id_m", "ibus"]
        }

        self._value_latest: dict[str, dict[str, Optional[float | int]]] = {
            field: {m: None for m in MOTOR_KEYS} for field, _ in VALUE_FIELDS
        }

        self.vbus0 = self.vbus1 = None
        self.last_seq = None
        self._hz_buf: list[float] = []
        self._frame_count = 0
        self._dirty = False

        # Live
        self._worker: Optional[TelemetryWorker] = None

        # Replay
        self._recs: list[dict] = []
        self._ridx   = 0
        self._rplay  = False
        self._rspeed = 1.0
        self._rtick  = time.monotonic()
        self._replay_mode = False

        # Recorder
        self._rec = DataRecorder()

        self._build_ui()
        self._build_graphs()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(RENDER_MS)

    # ── UI ───────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.setStyleSheet(f"""
        QMainWindow,QWidget{{background:{C_BG};color:{C_TEXT};
            font-family:'JetBrains Mono','Share Tech Mono','Consolas',monospace;
            font-size:9pt;}}
        QGroupBox{{border:1px solid {C_BORDER};border-radius:6px;margin-top:18px;
            padding:8px;font-size:8pt;font-weight:600;color:{C_TEXT_DIM};}}
        QGroupBox::title{{subcontrol-origin:margin;left:10px;padding:0 4px;
            color:{C_ACCENT};font-size:8pt;letter-spacing:.08em;}}
        QLineEdit{{background:{C_SURFACE};border:1px solid {C_BORDER};border-radius:4px;
            color:{C_TEXT};padding:4px 8px;}}
        QLineEdit:focus{{border-color:{C_ACCENT};}}
        QSlider::groove:horizontal{{background:{C_SURFACE};height:4px;border-radius:2px;}}
        QSlider::handle:horizontal{{background:{C_ACCENT};width:12px;height:12px;
            border-radius:6px;margin:-4px 0;}}
        QSlider::sub-page:horizontal{{background:{C_ACCENT};border-radius:2px;}}
        QComboBox{{background:{C_SURFACE};border:1px solid {C_BORDER};border-radius:4px;
            color:{C_TEXT};padding:4px 8px;}}
        QComboBox::drop-down{{border:none;}}
        QStatusBar{{background:{C_PANEL};border-top:1px solid {C_BORDER};
            color:{C_TEXT_DIM};font-size:8pt;}}
        """)

        root = QWidget(); self.setCentralWidget(root)
        vbox = QVBoxLayout(root); vbox.setSpacing(6); vbox.setContentsMargins(10,8,10,6)

        # ── Toolbar ──
        tb = QHBoxLayout(); tb.setSpacing(8)
        title = QLabel("BLDC·ACQ")
        title.setStyleSheet(
            f"color:{C_ACCENT};font-size:13pt;font-weight:700;letter-spacing:.12em;")
        tb.addWidget(title); tb.addWidget(_sep())

        tb.addWidget(_lbl("IP", C_TEXT_DIM))
        self.ip_edit = QLineEdit("192.168.10.176"); self.ip_edit.setFixedWidth(148)
        tb.addWidget(self.ip_edit)
        tb.addWidget(_lbl("Port", C_TEXT_DIM))
        self.port_edit = QLineEdit("8765"); self.port_edit.setFixedWidth(64)
        tb.addWidget(self.port_edit)

        self.connect_btn = _btn("Connect", C_OK)
        self.connect_btn.clicked.connect(self._connect)
        tb.addWidget(self.connect_btn)
        self.disc_btn = _btn("Disconnect", C_ERR, enabled=False)
        self.disc_btn.clicked.connect(self._disconnect)
        tb.addWidget(self.disc_btn)
        tb.addWidget(_sep())

        self.rec_btn = _btn("⏺  Record", C_ACCENT)
        self.rec_btn.clicked.connect(self._toggle_rec)
        tb.addWidget(self.rec_btn)
        self.load_btn = _btn("📂  Load")
        self.load_btn.clicked.connect(self._load)
        tb.addWidget(self.load_btn)
        tb.addWidget(_sep())

        self._mode_lbl = _lbl("LIVE", C_TEXT_DIM)
        self._dot      = _lbl("●", C_ERR, size=11)
        self._conn_lbl = _lbl("DISCONNECTED", C_TEXT_DIM)
        self._hz_lbl   = _lbl("-- Hz", C_TEXT_DIM)
        for w in [self._mode_lbl, self._dot, self._conn_lbl, self._hz_lbl]:
            tb.addWidget(w)
        tb.addStretch(1)

        self._v0_lbl  = _lbl("ODRV0: --V", C_TEXT_DIM)
        self._v1_lbl  = _lbl("ODRV1: --V", C_TEXT_DIM)
        self._seq_lbl = _lbl("SEQ: --", C_TEXT_DIM)
        for w in [self._v0_lbl, self._v1_lbl, self._seq_lbl]:
            tb.addWidget(w)
        vbox.addLayout(tb)

        # ── Centre grid ──
        grid = QGridLayout(); grid.setSpacing(6)
        vbox.addLayout(grid, stretch=1)

        # 3D pose — pure OpenGL scene
        self._gl_view = gl.GLViewWidget()
        self._gl_view.setMinimumWidth(560)
        self._gl_view.setBackgroundColor(pg.mkColor(C_SURFACE))
        self._gl_view.opts["distance"] = 10
        grid.addWidget(self._gl_view, 0, 0)

        # Graph area
        right = QGridLayout(); right.setSpacing(4)
        self._pw_iq   = self._make_plot("Iq (A)")
        self._pw_id   = self._make_plot("Id (A)")
        self._pw_ibus = self._make_plot("Ibus (A)", x_label=True)
        right.addWidget(self._pw_iq, 0, 0)
        right.addWidget(self._pw_id, 0, 1)
        right.addWidget(self._pw_ibus, 1, 0, 1, 2)
        right.setColumnStretch(0, 1)
        right.setColumnStretch(1, 1)
        right.setRowStretch(0, 1)
        right.setRowStretch(1, 1)
        rw = QWidget(); rw.setLayout(right)
        grid.addWidget(rw, 0, 1)

        # Value area
        self._values_group = self._build_values_panel()
        grid.addWidget(self._values_group, 1, 0, 1, 2)

        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 2)
        grid.setColumnStretch(0, 5); grid.setColumnStretch(1, 5)

        # ── Replay bar ──
        rb = QHBoxLayout(); rb.setSpacing(8)
        self._play_btn = _btn("▶  Play", C_ACCENT2, enabled=False)
        self._play_btn.clicked.connect(self._toggle_play)
        rb.addWidget(self._play_btn)
        rb.addWidget(_lbl("Speed", C_TEXT_DIM))
        self._spd = QComboBox()
        self._spd.addItems(["0.25×","0.5×","1×","2×","4×"])
        self._spd.setCurrentText("1×"); self._spd.setEnabled(False)
        self._spd.currentTextChanged.connect(self._on_speed)
        rb.addWidget(self._spd)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0,0); self._slider.setEnabled(False)
        self._slider.sliderMoved.connect(self._on_slide)
        rb.addWidget(self._slider, stretch=1)
        self._rt_lbl = _lbl("t: -- / --", C_TEXT_DIM)
        rb.addWidget(self._rt_lbl)
        vbox.addLayout(rb)

        self._status = QStatusBar(); self._status.setSizeGripEnabled(False)
        self.setStatusBar(self._status)
        self._status.showMessage(
            "Ready — 3D OpenGL pose + fast 2D plots. Install: pip install PyQt5 pyqtgraph PyOpenGL websockets numpy")

    def _make_plot(self, title: str, x_label=False) -> pg.PlotWidget:
        pw = pg.PlotWidget(title=title)
        pw.setBackground(C_SURFACE)
        pw.getPlotItem().titleLabel.item.setDefaultTextColor(pg.mkColor(C_ACCENT))
        pw.getPlotItem().titleLabel.item.setFont(QFont("JetBrains Mono", 8, QFont.Bold))
        pw.showGrid(x=True, y=True, alpha=0.18)
        pw.getAxis("left").setPen(pg.mkPen(C_BORDER))
        pw.getAxis("left").setTextPen(pg.mkPen(C_TEXT_DIM))
        pw.getAxis("bottom").setPen(pg.mkPen(C_BORDER))
        pw.getAxis("bottom").setTextPen(pg.mkPen(C_TEXT_DIM))
        if x_label:
            pw.setLabel("bottom", "time (s)", color=C_TEXT_DIM)
        pw.getViewBox().setMouseEnabled(x=False, y=True)
        return pw

    def _build_values_panel(self) -> QGroupBox:
        group = QGroupBox("Telemetry Values")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(4)

        layout.addWidget(_lbl("Metric", C_ACCENT, bold=True), 0, 0)
        for col, motor in enumerate(MOTOR_KEYS, start=1):
            layout.addWidget(_lbl(motor, C_ACCENT2, bold=True), 0, col)

        self._value_cells: dict[str, dict[str, QLabel]] = {}
        for row, (field, label) in enumerate(VALUE_FIELDS, start=1):
            layout.addWidget(_lbl(label, C_TEXT_DIM), row, 0)
            self._value_cells[field] = {}
            for col, motor in enumerate(MOTOR_KEYS, start=1):
                cell = _lbl("--", C_TEXT)
                self._value_cells[field][motor] = cell
                layout.addWidget(cell, row, col)

        return group

    # ── OpenGL scene ─────────────────────────────────────────────────────────

    def _build_graphs(self):
        # Floor grid
        grid = gl.GLGridItem()
        grid.setSize(20, 20); grid.setSpacing(1, 1)
        grid.setColor(pg.mkColor(C_BORDER))
        self._gl_view.addItem(grid)

        # Path trail
        self._gl_path = gl.GLLinePlotItem(
            pos=np.zeros((1,3), dtype=np.float32),
            color=pg.mkColor(C_ACCENT),
            width=2.0, antialias=True, mode="line_strip",
        )
        self._gl_view.addItem(self._gl_path)

        # Arrow — shaft
        self._gl_shaft = gl.GLLinePlotItem(
            pos=np.zeros((2,3), dtype=np.float32),
            color=pg.mkColor(C_ACCENT2), width=3.0, antialias=True,
        )
        self._gl_view.addItem(self._gl_shaft)

        # Arrow — head (two wing lines)
        self._gl_wing_l = gl.GLLinePlotItem(
            pos=np.zeros((2,3), dtype=np.float32),
            color=pg.mkColor(C_ACCENT2), width=2.5, antialias=True,
        )
        self._gl_wing_r = gl.GLLinePlotItem(
            pos=np.zeros((2,3), dtype=np.float32),
            color=pg.mkColor(C_ACCENT2), width=2.5, antialias=True,
        )
        self._gl_view.addItem(self._gl_wing_l)
        self._gl_view.addItem(self._gl_wing_r)

        # Current graph lines — one PlotDataItem per ODrive per metric
        # setData() uploads directly to the GPU each frame
        self._lines: dict[str, dict[str, pg.PlotDataItem]] = {}
        pw_map = {"iq": self._pw_iq, "id_m": self._pw_id, "ibus": self._pw_ibus}
        for key, pw in pw_map.items():
            self._lines[key] = {}
            leg = pw.addLegend(offset=(5,5),
                               labelTextColor=pg.mkColor(C_TEXT),
                               brush=pg.mkBrush(C_PANEL))
            for odrv in ODRV_KEYS:
                r,g,b,a = ODRV_COLORS_PG[odrv]
                pen  = pg.mkPen(color=(r,g,b,a), width=1.6)
                item = pw.plot(pen=pen, name=odrv)
                self._lines[key][odrv] = item

    # ── Live connection ───────────────────────────────────────────────────────

    def _connect(self):
        if self._worker:
            self._status.showMessage("Already connected — disconnect first."); return
        if not WEBSOCKETS_OK:
            QMessageBox.critical(self, "Missing library",
                "Run:  pip install websockets"); return
        ip   = self.ip_edit.text().strip()
        port = self.port_edit.text().strip()
        try: int(port)
        except ValueError:
            QMessageBox.warning(self, "Bad port", f"Port must be integer, got: {port!r}"); return

        self._worker = TelemetryWorker(f"ws://{ip}:{port}", parent=self)
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.status_changed.connect(self._on_ws_status)
        self._worker.start()
        self._replay_mode = False
        self._mode_lbl.setText("LIVE")
        self.connect_btn.setEnabled(False)
        self.disc_btn.setEnabled(True)

    def _disconnect(self):
        if self._worker:
            self._worker.stop(); self._worker.wait(2000); self._worker = None
        self._dot.setStyleSheet(f"color:{C_ERR};font-size:11pt;")
        self._conn_lbl.setText("DISCONNECTED")
        self.connect_btn.setEnabled(True); self.disc_btn.setEnabled(False)
        self._status.showMessage("Disconnected.")

    def _on_ws_status(self, state: str, msg: str):
        c = {"ok": C_OK, "err": C_ERR, "info": C_TEXT_DIM}.get(state, C_TEXT_DIM)
        self._dot.setStyleSheet(f"color:{c};font-size:11pt;")
        self._conn_lbl.setText(
            {"ok":"CONNECTED","err":"ERROR","info":"…"}.get(state, state.upper()))
        self._status.showMessage(msg)

    # ── Recording ─────────────────────────────────────────────────────────────

    def _toggle_rec(self):
        if self._rec.active:
            ok, msg = self._rec.stop()
            self.rec_btn.setText("⏺  Record")
            if ok:
                self._status.showMessage(msg)
                QMessageBox.information(self, "Recording stopped", msg)
            else:
                QMessageBox.warning(self, "Recording error", msg)
            return

        default = os.path.join(os.path.expanduser("~"), f"bldc_{int(time.time())}.jsonl")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save session", default, "JSONL (*.jsonl)")
        if not path: return

        ok, err = self._rec.start(path, {
            "rover_ip": self.ip_edit.text().strip(),
            "ws_port":  self.port_edit.text().strip(),
        })
        if ok:
            self.rec_btn.setText("⏹  Stop Recording")
            self._status.showMessage(f"Recording → {path}")
        else:
            QMessageBox.critical(self, "Recording error", err)

    # ── Session loading ───────────────────────────────────────────────────────

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load session", os.path.expanduser("~"), "JSONL (*.jsonl)")
        if not path or not os.path.isfile(path):
            return
        recs, bad = [], 0
        try:
            with open(path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line: continue
                    try:
                        o = json.loads(line)
                        if o.get("type") == "telemetry_record":
                            recs.append(o)
                    except Exception:
                        bad += 1
        except Exception as exc:
            QMessageBox.critical(self, "Load error", str(exc)); return

        if not recs:
            QMessageBox.warning(self, "Load", "No telemetry_record entries found."); return

        self._recs = recs; self._ridx = 0; self._rplay = False
        self._replay_mode = True
        self._mode_lbl.setText("REPLAY")
        self._play_btn.setEnabled(True); self._spd.setEnabled(True)
        self._slider.setEnabled(True)
        self._slider.setRange(0, len(recs)-1); self._slider.setValue(0)
        self._reset_buffers()
        self._apply_rec(recs[0])
        self._dirty = True
        note = f"Loaded {len(recs)} records"
        if bad: note += f" ({bad} skipped)"
        self._status.showMessage(note + f"  ·  {os.path.basename(path)}")

    # ── Replay controls ───────────────────────────────────────────────────────

    def _toggle_play(self):
        if not self._replay_mode: return
        self._rplay = not self._rplay
        self._play_btn.setText("⏸  Pause" if self._rplay else "▶  Play")
        self._rtick = time.monotonic()

    def _on_speed(self, text: str):
        try: self._rspeed = float(text.replace("×",""))
        except: self._rspeed = 1.0

    def _on_slide(self, idx: int):
        if not self._recs: return
        idx = max(0, min(idx, len(self._recs)-1))
        self._ridx = idx
        self._reset_buffers()
        for i in range(idx+1):
            self._apply_rec(self._recs[i])
        self._dirty = True

    # ── Frame ingestion ───────────────────────────────────────────────────────

    def _on_frame(self, frame: TelemetryFrame):
        self._ingest(frame)

    def _ingest(self, frame: TelemetryFrame):
        t = frame.ts_host
        self._t_buf.append(t)

        def avg_pair(data: dict, a: str, b: str) -> float:
            va = data.get(a)
            vb = data.get(b)
            vals = [v for v in [va, vb] if v is not None]
            return float(sum(vals) / len(vals)) if vals else 0.0

        self._metric["iq"]["ODRV0"].append(avg_pair(frame.iq, "FL", "RL"))
        self._metric["iq"]["ODRV1"].append(avg_pair(frame.iq, "FR", "RR"))
        self._metric["id_m"]["ODRV0"].append(avg_pair(frame.id_m, "FL", "RL"))
        self._metric["id_m"]["ODRV1"].append(avg_pair(frame.id_m, "FR", "RR"))
        self._metric["ibus"]["ODRV0"].append(avg_pair(frame.ibus, "FL", "RL"))
        self._metric["ibus"]["ODRV1"].append(avg_pair(frame.ibus, "FR", "RR"))

        for m in MOTOR_KEYS:
            self._value_latest["ibus"][m] = frame.ibus.get(m)
            self._value_latest["fet_temp"][m] = frame.fet_temp.get(m)
            self._value_latest["axis_error"][m] = frame.axis_error.get(m)
            self._value_latest["motor_error"][m] = frame.motor_error.get(m)
            self._value_latest["encoder_error"][m] = frame.encoder_error.get(m)
            self._value_latest["controller_error"][m] = frame.controller_error.get(m)

        self.vbus0 = frame.vbus0; self.vbus1 = frame.vbus1
        self.last_seq = frame.seq
        self._hz_buf.append(t)
        if len(self._hz_buf) > 120: self._hz_buf = self._hz_buf[-120:]

        self._integrate(frame)
        self._dirty = True
        self._frame_count += 1

        if self._rec.active:
            self._rec.append({
                "type":"telemetry_record",
                "ts_host":frame.ts_host,"ts_source":frame.ts_source,
                "seq":frame.seq,"vbus0":frame.vbus0,"vbus1":frame.vbus1,
                "pos":frame.pos,
                "iq":frame.iq,
                "id_m":frame.id_m,
                "ibus":frame.ibus,
                "fet_temp":frame.fet_temp,
                "axis_error":frame.axis_error,
                "motor_error":frame.motor_error,
                "encoder_error":frame.encoder_error,
                "controller_error":frame.controller_error,
                "pose":{"x":self.x,"y":self.y,"yaw":self.yaw},
            })

    def _integrate(self, frame: TelemetryFrame):
        fl = frame.pos.get("FL"); rl = frame.pos.get("RL")
        fr = frame.pos.get("FR"); rr = frame.pos.get("RR")
        if any(v is None for v in [fl,rl,fr,rr]): return

        L = ((fl+rl)*0.5) * self.fwd_sign
        R = ((fr+rr)*0.5) * self.right_sign * self.fwd_sign
        if self._prev_l is None:
            self._prev_l, self._prev_r, self._last_ts = L, R, frame.ts_host; return

        dt = frame.ts_host - self._last_ts
        if dt <= 0: return
        dL = (L - self._prev_l) * self.wheel_circum_m
        dR = (R - self._prev_r) * self.wheel_circum_m
        self._prev_l, self._prev_r, self._last_ts = L, R, frame.ts_host

        dc   = (dL+dR)*0.5
        dyaw = (dR-dL)/self.track_width_m
        ym   = self.yaw + 0.5*dyaw
        self.x   += dc * math.cos(ym)
        self.y   += dc * math.sin(ym)
        self.yaw  = math.atan2(math.sin(self.yaw+dyaw), math.cos(self.yaw+dyaw))
        self._px_buf.append(self.x); self._py_buf.append(self.y)
        if len(self._px_buf) > MAX_POINTS:
            self._px_buf = self._px_buf[-MAX_POINTS:]
            self._py_buf = self._py_buf[-MAX_POINTS:]

    def _apply_rec(self, rec: dict):
        def _f(v): return float(v) if isinstance(v,(int,float)) else None
        self._ingest(TelemetryFrame(
            ts_host=float(rec.get("ts_host",0.0)),
            ts_source=_f(rec.get("ts_source")),
            seq=int(rec["seq"]) if isinstance(rec.get("seq"),int) else None,
            vbus0=_f(rec.get("vbus0")), vbus1=_f(rec.get("vbus1")),
            pos=rec.get("pos") or {},
            iq=rec.get("iq") or {},
            id_m=rec.get("id_m") or {},
            ibus=rec.get("ibus") or {},
            fet_temp=rec.get("fet_temp") or {},
            axis_error=rec.get("axis_error") or {},
            motor_error=rec.get("motor_error") or {},
            encoder_error=rec.get("encoder_error") or {},
            controller_error=rec.get("controller_error") or {},
        ))

    def _reset_buffers(self):
        self._t_buf.clear()
        self._px_buf, self._py_buf = [0.0], [0.0]
        self.x = self.y = self.yaw = 0.0
        self._prev_l = self._prev_r = None; self._last_ts = 0.0
        for k in self._metric:
            for odrv in ODRV_KEYS: self._metric[k][odrv].clear()
        for field in self._value_latest:
            for m in MOTOR_KEYS:
                self._value_latest[field][m] = None
        self._hz_buf.clear(); self._frame_count = 0

    # ── Main timer tick ───────────────────────────────────────────────────────

    def _tick(self):
        # Advance replay
        if self._replay_mode and self._rplay and self._recs:
            now = time.monotonic()
            elapsed = now - self._rtick; self._rtick = now
            step   = max(1, int(round(elapsed * self._rspeed * 30.0)))
            target = min(len(self._recs)-1, self._ridx + step)
            for i in range(self._ridx+1, target+1):
                self._apply_rec(self._recs[i])
            self._ridx = target
            self._slider.blockSignals(True)
            self._slider.setValue(self._ridx)
            self._slider.blockSignals(False)
            if self._ridx >= len(self._recs)-1:
                self._rplay = False; self._play_btn.setText("▶  Play")

        # Replay time
        if self._recs:
            t0 = float(self._recs[0].get("ts_host",0.0))
            ti = float(self._recs[self._ridx].get("ts_host",t0))
            tN = float(self._recs[-1].get("ts_host",t0))
            self._rt_lbl.setText(f"t: {ti-t0:6.2f}s / {tN-t0:6.2f}s")

        # Hz estimate
        if len(self._hz_buf) >= 2:
            span = self._hz_buf[-1] - self._hz_buf[0]
            if span > 1e-6:
                self._hz_lbl.setText(f"{(len(self._hz_buf)-1)/span:.1f} Hz")

        # Text labels (negligible cost)
        v0 = f"{self.vbus0:.2f}V" if self.vbus0 is not None else "--V"
        v1 = f"{self.vbus1:.2f}V" if self.vbus1 is not None else "--V"
        self._v0_lbl.setText(f"ODRV0: {v0}")
        self._v1_lbl.setText(f"ODRV1: {v1}")
        self._seq_lbl.setText(f"SEQ: {'--' if self.last_seq is None else self.last_seq}")
        self._update_value_panel()
        if self._rec.active:
            self.rec_btn.setText(f"⏹  Stop [{self._rec.count}]")

        # GPU render — skip entirely if nothing changed
        if self._dirty:
            self._render_pose()
            self._render_currents()
            self._dirty = False

    # ── GPU render: OpenGL pose ───────────────────────────────────────────────

    def _format_value(self, field: str, value: Optional[float | int]) -> str:
        if value is None:
            return "--"
        if "error" in field:
            return f"0x{int(value):08X}"
        return f"{float(value):.2f}"

    def _update_value_panel(self):
        for field, _ in VALUE_FIELDS:
            for motor in MOTOR_KEYS:
                value = self._value_latest[field][motor]
                self._value_cells[field][motor].setText(self._format_value(field, value))

    def _render_pose(self):
        n = len(self._px_buf)
        if n >= 2:
            pts = np.column_stack([
                np.array(self._px_buf, dtype=np.float32),
                np.array(self._py_buf, dtype=np.float32),
                np.zeros(n, dtype=np.float32),
            ])
            self._gl_path.setData(pos=pts)  # uploads to GPU VRAM

        # Arrow shaft
        L  = 0.30
        ax = self.x + L * math.cos(self.yaw)
        ay = self.y + L * math.sin(self.yaw)
        self._gl_shaft.setData(
            pos=np.array([[self.x,self.y,0.0],[ax,ay,0.0]], dtype=np.float32))

        # Arrowhead chevron wings
        hw = 0.08; hb = 0.12
        bx = ax - hb*math.cos(self.yaw)
        by = ay - hb*math.sin(self.yaw)
        px = -math.sin(self.yaw); py = math.cos(self.yaw)
        self._gl_wing_l.setData(
            pos=np.array([[ax,ay,0],[bx+hw*px,by+hw*py,0]], dtype=np.float32))
        self._gl_wing_r.setData(
            pos=np.array([[ax,ay,0],[bx-hw*px,by-hw*py,0]], dtype=np.float32))

        # Camera tracks robot
        self._gl_view.opts["center"] = pg.Vector(self.x, self.y, 0)

    # ── GPU render: current plots ─────────────────────────────────────────────

    def _render_currents(self):
        t_arr = self._t_buf.array()
        if len(t_arr) < 2: return
        tx = (t_arr - t_arr[0]).astype(np.float32)

        for key in ["iq", "id_m", "ibus"]:
            for odrv in ODRV_KEYS:
                ys = self._metric[key][odrv].array().astype(np.float32)
                n  = min(len(tx), len(ys))
                if n > 1:
                    # setData() uploads vertex buffer to GPU — no CPU redraw
                    self._lines[key][odrv].setData(x=tx[-n:], y=ys[-n:])

    # ── Close ─────────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._timer.stop()
        if self._rec.active:
            r = QMessageBox.question(
                self, "Recording active",
                "Stop and save before closing?",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                QMessageBox.Yes)
            if r == QMessageBox.Cancel:
                self._timer.start(RENDER_MS); event.ignore(); return
            if r == QMessageBox.Yes:
                _, msg = self._rec.stop()
                self._status.showMessage(msg)
        if self._worker:
            self._worker.stop(); self._worker.wait(2000)
        event.accept()


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QPalette()
    for role, c in [
        (QPalette.Window,          C_BG),
        (QPalette.WindowText,      C_TEXT),
        (QPalette.Base,            C_SURFACE),
        (QPalette.AlternateBase,   C_PANEL),
        (QPalette.Text,            C_TEXT),
        (QPalette.Button,          C_PANEL),
        (QPalette.ButtonText,      C_TEXT),
        (QPalette.Highlight,       C_ACCENT),
        (QPalette.HighlightedText, C_BG),
    ]:
        pal.setColor(role, QColor(c))
    app.setPalette(pal)

    win = MainWindow()
    win.show()

    signal.signal(signal.SIGINT, lambda *_: (win.close(), app.quit()))
    pulse = QTimer(); pulse.start(150); pulse.timeout.connect(lambda: None)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()