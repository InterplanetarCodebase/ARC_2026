#!/usr/bin/env python3
"""
Pose Data Acquisition GUI
=========================
ROS2 pose logger/visualizer for wheel-vs-ZED calibration.

Live inputs:
- wheel pose topic (geometry_msgs/PoseStamped)
- zed pose topic (geometry_msgs/PoseStamped)

Plots:
- ZED Position (x, y, z)
- ZED Orientation (qx, qy, qz, qw)
- Wheel Position (x, y, z)
- Wheel Orientation (qx, qy, qz, qw)

Records JSONL for later replay/visualization.

Run:
    python3 pose_data_acquisition_gui.py
"""
from __future__ import annotations

import json
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

import pyqtgraph as pg
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped

    ROS_OK = True
except Exception:
    rclpy = None
    PoseStamped = None
    ROS_OK = False


pg.setConfigOptions(
    useOpenGL=False,
    enableExperimental=False,
    antialias=True,
    foreground="#e2e8f0",
    background="#080b10",
)

C_BG = "#080b10"
C_SURFACE = "#0d1320"
C_PANEL = "#111827"
C_BORDER = "#1e293b"
C_ACCENT = "#ff6b35"
C_ACCENT2 = "#00c8ff"
C_TEXT = "#e2e8f0"
C_TEXT_DIM = "#64748b"
C_TEXT_MUTED = "#334155"
C_OK = "#22c55e"
C_ERR = "#ef4444"

MAX_POINTS = 3000
RENDER_FPS = 60
RENDER_MS = 1000 // RENDER_FPS

SRC_KEYS = ["zed", "wheel"]
POS_KEYS = ["x", "y", "z"]
ORI_KEYS = ["qx", "qy", "qz", "qw"]

POS_COLORS = {
    "x": (255, 107, 53, 220),
    "y": (0, 200, 255, 220),
    "z": (126, 232, 255, 220),
}
ORI_COLORS = {
    "qx": (255, 107, 53, 220),
    "qy": (0, 200, 255, 220),
    "qz": (126, 232, 255, 220),
    "qw": (180, 120, 255, 220),
}


@dataclass
class PoseFrame:
    ts_host: float
    source: str
    stamp_sec: Optional[int]
    stamp_nsec: Optional[int]
    frame_id: str
    x: Optional[float]
    y: Optional[float]
    z: Optional[float]
    qx: Optional[float]
    qy: Optional[float]
    qz: Optional[float]
    qw: Optional[float]


class RingBuffer:
    def __init__(self, maxlen: int):
        self._buf = np.zeros(maxlen, dtype=np.float32)
        self._n = 0
        self._head = 0
        self._max = maxlen

    def append(self, value: float):
        self._buf[self._head] = float(value)
        self._head = (self._head + 1) % self._max
        if self._n < self._max:
            self._n += 1

    def array(self) -> np.ndarray:
        if self._n < self._max:
            return self._buf[: self._n].copy()
        return np.roll(self._buf, -self._head)

    def clear(self):
        self._buf[:] = 0
        self._n = 0
        self._head = 0


class DataRecorder:
    def __init__(self):
        self._fh = None
        self._thread = None
        self._queue = deque()
        self._lock = threading.Lock()
        self._run_writer = False
        self.path = ""
        self.count = 0
        self.active = False

    def _writer_loop(self):
        last_flush = time.monotonic()
        while self._run_writer or self._queue:
            batch = []
            with self._lock:
                while self._queue and len(batch) < 512:
                    batch.append(self._queue.popleft())

            if not batch:
                time.sleep(0.01)
                continue

            lines = [json.dumps(rec) for rec in batch]
            try:
                self._fh.write("\n".join(lines) + "\n")
                self.count += len(batch)
                now = time.monotonic()
                if now - last_flush >= 0.25:
                    self._fh.flush()
                    last_flush = now
            except Exception:
                self.active = False
                self._run_writer = False
                break

    def start(self, path: str, meta: dict) -> tuple[bool, str]:
        if self.active:
            return False, "Already recording"
        try:
            folder = os.path.dirname(path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            self._fh = open(path, "w", encoding="utf-8", buffering=1024 * 1024)
            self.path = path
            self.count = 0
            self._queue.clear()
            self._fh.write(
                json.dumps(
                    {
                        "type": "session_meta",
                        "created_unix": time.time(),
                        "meta": meta,
                    }
                )
                + "\n"
            )
            self.active = True
            self._run_writer = True
            self._thread = threading.Thread(target=self._writer_loop, daemon=True)
            self._thread.start()
            return True, ""
        except Exception as exc:
            self._fh = None
            self._thread = None
            self.active = False
            self._run_writer = False
            return False, str(exc)

    def append(self, rec: dict):
        if not self.active:
            return
        with self._lock:
            # Keep memory bounded if producer outruns writer.
            if len(self._queue) > 50000:
                self._queue.popleft()
            self._queue.append(rec)

    def stop(self) -> tuple[bool, str]:
        self.active = False
        self._run_writer = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        try:
            if self._fh:
                self._fh.flush()
                self._fh.close()
        except Exception:
            pass
        saved, n = self.path, self.count
        self._fh = None
        with self._lock:
            self._queue.clear()
        return True, f"Saved {n} frames -> {saved}"


def _lbl(text, color=C_TEXT, bold=False, size=9):
    w = QLabel(text)
    w.setStyleSheet(
        f"color:{color};font-weight:{'600' if bold else '400'};font-size:{size}pt;"
    )
    return w


def _btn(text, accent=C_ACCENT, enabled=True):
    b = QPushButton(text)
    b.setEnabled(enabled)
    b.setStyleSheet(
        f"""
        QPushButton{{background:{C_PANEL};color:{C_TEXT};border:1px solid {C_BORDER};
            border-radius:4px;padding:5px 14px;font-size:9pt;font-weight:500;}}
        QPushButton:hover{{border-color:{accent};color:{accent};}}
        QPushButton:pressed{{background:{C_SURFACE};}}
        QPushButton:disabled{{color:{C_TEXT_MUTED};border-color:{C_TEXT_MUTED};}}"""
    )
    return b


def _sep():
    f = QFrame()
    f.setFrameShape(QFrame.VLine)
    f.setStyleSheet(f"color:{C_BORDER};")
    return f


class PoseWorker(QThread):
    frame_ready = pyqtSignal(object)
    status_changed = pyqtSignal(str, str)

    def __init__(self, wheel_topic: str, zed_topic: str, parent=None):
        super().__init__(parent)
        self._wheel_topic = wheel_topic
        self._zed_topic = zed_topic
        self._stop = False

    def stop(self):
        self._stop = True
        self.quit()

    @staticmethod
    def _f(value) -> Optional[float]:
        try:
            return float(value)
        except Exception:
            return None

    def _emit_pose(self, source: str, msg: Any):
        pose = msg.pose
        frame = PoseFrame(
            ts_host=time.monotonic(),
            source=source,
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nsec=int(msg.header.stamp.nanosec),
            frame_id=msg.header.frame_id,
            x=self._f(pose.position.x),
            y=self._f(pose.position.y),
            z=self._f(pose.position.z),
            qx=self._f(pose.orientation.x),
            qy=self._f(pose.orientation.y),
            qz=self._f(pose.orientation.z),
            qw=self._f(pose.orientation.w),
        )
        self.frame_ready.emit(frame)

    def run(self):
        if not ROS_OK:
            self.status_changed.emit("err", "ROS2 (rclpy/geometry_msgs) import failed")
            return

        node = None
        try:
            if not rclpy.ok():
                rclpy.init()

            node = rclpy.create_node("pose_data_acq_gui_sub")

            def on_wheel(msg):
                self._emit_pose("wheel", msg)

            def on_zed(msg):
                self._emit_pose("zed", msg)

            node.create_subscription(PoseStamped, self._wheel_topic, on_wheel, 50)
            node.create_subscription(PoseStamped, self._zed_topic, on_zed, 50)
            self.status_changed.emit(
                "ok", f"Connected ROS topics: wheel={self._wheel_topic}, zed={self._zed_topic}"
            )

            while not self._stop and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

        except Exception as exc:
            self.status_changed.emit("err", f"ROS subscriber error: {str(exc)[:140]}")
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            except Exception:
                pass
            self.status_changed.emit("info", "Disconnected")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("POSE ACQ GUI")
        self.resize(1600, 960)

        self._worker: Optional[PoseWorker] = None
        self._rec = DataRecorder()

        self._recs: list[dict] = []
        self._ridx = 0
        self._rplay = False
        self._rspeed = 1.0
        self._rtick = time.monotonic()
        self._replay_mode = False

        self._latest = {
            "zed": {k: None for k in (POS_KEYS + ORI_KEYS)},
            "wheel": {k: None for k in (POS_KEYS + ORI_KEYS)},
        }

        self._t_buf = RingBuffer(MAX_POINTS)
        self._hz_buf: list[float] = []

        self._metric = {
            src: {
                "pos": {k: RingBuffer(MAX_POINTS) for k in POS_KEYS},
                "ori": {k: RingBuffer(MAX_POINTS) for k in ORI_KEYS},
            }
            for src in SRC_KEYS
        }

        self._last_frame_id = {"zed": "", "wheel": ""}
        self._frame_count = 0
        self._dirty = False

        self._build_ui()
        self._build_graphs()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(RENDER_MS)

    def _build_ui(self):
        self.setStyleSheet(
            f"""
        QMainWindow,QWidget{{background:{C_BG};color:{C_TEXT};
            font-family:'JetBrains Mono','Share Tech Mono','Consolas',monospace;
            font-size:9pt;}}
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
        """
        )

        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setSpacing(6)
        vbox.setContentsMargins(10, 8, 10, 6)

        tb = QHBoxLayout()
        tb.setSpacing(8)
        title = QLabel("POSE-ACQ")
        title.setStyleSheet(
            f"color:{C_ACCENT};font-size:13pt;font-weight:700;letter-spacing:.12em;"
        )
        tb.addWidget(title)
        tb.addWidget(_sep())

        tb.addWidget(_lbl("Wheel Topic", C_TEXT_DIM))
        self.wheel_topic_edit = QLineEdit("/wheel_pose")
        self.wheel_topic_edit.setFixedWidth(230)
        tb.addWidget(self.wheel_topic_edit)

        tb.addWidget(_lbl("ZED Topic", C_TEXT_DIM))
        self.zed_topic_edit = QLineEdit("/zed/zed_node/pose")
        self.zed_topic_edit.setFixedWidth(260)
        tb.addWidget(self.zed_topic_edit)

        self.connect_btn = _btn("Connect", C_OK)
        self.connect_btn.clicked.connect(self._connect)
        tb.addWidget(self.connect_btn)

        self.disc_btn = _btn("Disconnect", C_ERR, enabled=False)
        self.disc_btn.clicked.connect(self._disconnect)
        tb.addWidget(self.disc_btn)

        tb.addWidget(_sep())
        self.rec_btn = _btn("Record", C_ACCENT)
        self.rec_btn.clicked.connect(self._toggle_rec)
        tb.addWidget(self.rec_btn)

        self.load_btn = _btn("Load")
        self.load_btn.clicked.connect(self._load)
        tb.addWidget(self.load_btn)

        tb.addWidget(_sep())
        self._mode_lbl = _lbl("LIVE", C_TEXT_DIM)
        self._dot = _lbl("●", C_ERR, size=11)
        self._conn_lbl = _lbl("DISCONNECTED", C_TEXT_DIM)
        self._hz_lbl = _lbl("-- Hz", C_TEXT_DIM)
        tb.addWidget(self._mode_lbl)
        tb.addWidget(self._dot)
        tb.addWidget(self._conn_lbl)
        tb.addWidget(self._hz_lbl)
        tb.addStretch(1)

        self._zed_frame_lbl = _lbl("ZED frame: --", C_TEXT_DIM)
        self._wheel_frame_lbl = _lbl("Wheel frame: --", C_TEXT_DIM)
        tb.addWidget(self._zed_frame_lbl)
        tb.addWidget(self._wheel_frame_lbl)
        vbox.addLayout(tb)

        grid = QGridLayout()
        grid.setSpacing(6)

        self._pw_zed_pos = self._make_plot("ZED Position")
        self._pw_zed_ori = self._make_plot("ZED Orientation")
        self._pw_wheel_pos = self._make_plot("Wheel Position")
        self._pw_wheel_ori = self._make_plot("Wheel Orientation", x_label=True)

        grid.addWidget(self._pw_zed_pos, 0, 0)
        grid.addWidget(self._pw_zed_ori, 0, 1)
        grid.addWidget(self._pw_wheel_pos, 1, 0)
        grid.addWidget(self._pw_wheel_ori, 1, 1)

        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        vbox.addLayout(grid, stretch=1)

        rb = QHBoxLayout()
        rb.setSpacing(8)
        self._play_btn = _btn("Play", C_ACCENT2, enabled=False)
        self._play_btn.clicked.connect(self._toggle_play)
        rb.addWidget(self._play_btn)
        rb.addWidget(_lbl("Speed", C_TEXT_DIM))
        self._spd = QComboBox()
        self._spd.addItems(["0.25x", "0.5x", "1x", "2x", "4x"])
        self._spd.setCurrentText("1x")
        self._spd.setEnabled(False)
        self._spd.currentTextChanged.connect(self._on_speed)
        rb.addWidget(self._spd)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, 0)
        self._slider.setEnabled(False)
        self._slider.sliderMoved.connect(self._on_slide)
        rb.addWidget(self._slider, stretch=1)
        self._rt_lbl = _lbl("t: -- / --", C_TEXT_DIM)
        rb.addWidget(self._rt_lbl)
        vbox.addLayout(rb)

        self._status = QStatusBar()
        self._status.setSizeGripEnabled(False)
        self.setStatusBar(self._status)
        self._status.showMessage(
            "Ready - subscribe wheel/zed pose topics, plot live, record JSONL, replay later"
        )

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
        return pw

    def _build_graphs(self):
        self._lines = {
            "zed": {"pos": {}, "ori": {}},
            "wheel": {"pos": {}, "ori": {}},
        }

        self._pw_zed_pos.addLegend(offset=(5, 5), labelTextColor=pg.mkColor(C_TEXT), brush=pg.mkBrush(C_PANEL))
        self._pw_zed_ori.addLegend(offset=(5, 5), labelTextColor=pg.mkColor(C_TEXT), brush=pg.mkBrush(C_PANEL))
        self._pw_wheel_pos.addLegend(offset=(5, 5), labelTextColor=pg.mkColor(C_TEXT), brush=pg.mkBrush(C_PANEL))
        self._pw_wheel_ori.addLegend(offset=(5, 5), labelTextColor=pg.mkColor(C_TEXT), brush=pg.mkBrush(C_PANEL))

        for k in POS_KEYS:
            r, g, b, a = POS_COLORS[k]
            pen = pg.mkPen(color=(r, g, b, a), width=1.8)
            self._lines["zed"]["pos"][k] = self._pw_zed_pos.plot(pen=pen, name=k)
            self._lines["wheel"]["pos"][k] = self._pw_wheel_pos.plot(pen=pen, name=k)

        for k in ORI_KEYS:
            r, g, b, a = ORI_COLORS[k]
            pen = pg.mkPen(color=(r, g, b, a), width=1.8)
            self._lines["zed"]["ori"][k] = self._pw_zed_ori.plot(pen=pen, name=k)
            self._lines["wheel"]["ori"][k] = self._pw_wheel_ori.plot(pen=pen, name=k)

    def _connect(self):
        if self._worker is not None:
            self._status.showMessage("Already connected - disconnect first")
            return
        if not ROS_OK:
            QMessageBox.critical(self, "ROS2 missing", "Could not import rclpy/PoseStamped")
            return

        wheel_topic = self.wheel_topic_edit.text().strip()
        zed_topic = self.zed_topic_edit.text().strip()
        if not wheel_topic or not zed_topic:
            QMessageBox.warning(self, "Bad topics", "Topic names cannot be empty")
            return

        self._worker = PoseWorker(wheel_topic, zed_topic, parent=self)
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.status_changed.connect(self._on_status)
        self._worker.start()

        self._replay_mode = False
        self._mode_lbl.setText("LIVE")
        self.connect_btn.setEnabled(False)
        self.disc_btn.setEnabled(True)

    def _disconnect(self):
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(2000)
            self._worker = None
        self._dot.setStyleSheet(f"color:{C_ERR};font-size:11pt;")
        self._conn_lbl.setText("DISCONNECTED")
        self.connect_btn.setEnabled(True)
        self.disc_btn.setEnabled(False)
        self._status.showMessage("Disconnected")

    def _on_status(self, state: str, msg: str):
        color = {"ok": C_OK, "err": C_ERR, "info": C_TEXT_DIM}.get(state, C_TEXT_DIM)
        self._dot.setStyleSheet(f"color:{color};font-size:11pt;")
        self._conn_lbl.setText({"ok": "CONNECTED", "err": "ERROR", "info": "..."}.get(state, state.upper()))
        self._status.showMessage(msg)

    def _toggle_rec(self):
        if self._rec.active:
            ok, msg = self._rec.stop()
            self.rec_btn.setText("Record")
            if ok:
                self._status.showMessage(msg)
                QMessageBox.information(self, "Recording stopped", msg)
            else:
                QMessageBox.warning(self, "Recording error", msg)
            return

        default = os.path.join(os.path.expanduser("~"), f"pose_acq_{int(time.time())}.jsonl")
        path, _ = QFileDialog.getSaveFileName(self, "Save session", default, "JSONL (*.jsonl)")
        if not path:
            return

        ok, err = self._rec.start(
            path,
            {
                "wheel_topic": self.wheel_topic_edit.text().strip(),
                "zed_topic": self.zed_topic_edit.text().strip(),
            },
        )
        if ok:
            self.rec_btn.setText("Stop Recording")
            self._status.showMessage(f"Recording -> {path}")
        else:
            QMessageBox.critical(self, "Recording error", err)

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load session", os.path.expanduser("~"), "JSONL (*.jsonl)")
        if not path or not os.path.isfile(path):
            return

        recs = []
        bad = 0
        try:
            with open(path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line)
                        if obj.get("type") == "pose_record":
                            recs.append(obj)
                    except Exception:
                        bad += 1
        except Exception as exc:
            QMessageBox.critical(self, "Load error", str(exc))
            return

        if not recs:
            QMessageBox.warning(self, "Load", "No pose_record entries found")
            return

        self._recs = recs
        self._ridx = 0
        self._rplay = False
        self._replay_mode = True
        self._mode_lbl.setText("REPLAY")
        self._play_btn.setEnabled(True)
        self._spd.setEnabled(True)
        self._slider.setEnabled(True)
        self._slider.setRange(0, len(recs) - 1)
        self._slider.setValue(0)

        self._reset_buffers()
        self._apply_rec(recs[0])
        self._dirty = True

        note = f"Loaded {len(recs)} records"
        if bad:
            note += f" ({bad} skipped)"
        self._status.showMessage(note + f"  -  {os.path.basename(path)}")

    def _toggle_play(self):
        if not self._replay_mode:
            return
        self._rplay = not self._rplay
        self._play_btn.setText("Pause" if self._rplay else "Play")
        self._rtick = time.monotonic()

    def _on_speed(self, text: str):
        try:
            self._rspeed = float(text.replace("x", ""))
        except Exception:
            self._rspeed = 1.0

    def _on_slide(self, idx: int):
        if not self._recs:
            return
        idx = max(0, min(idx, len(self._recs) - 1))
        self._ridx = idx
        self._reset_buffers()
        for i in range(idx + 1):
            self._apply_rec(self._recs[i])
        self._dirty = True

    def _on_frame(self, frame: PoseFrame):
        self._ingest(frame, record=True)

    def _ingest(self, frame: PoseFrame, record: bool):
        src = frame.source
        if src not in SRC_KEYS:
            return

        self._latest[src] = {
            "x": frame.x,
            "y": frame.y,
            "z": frame.z,
            "qx": frame.qx,
            "qy": frame.qy,
            "qz": frame.qz,
            "qw": frame.qw,
        }
        self._last_frame_id[src] = frame.frame_id

        self._t_buf.append(frame.ts_host)
        for s in SRC_KEYS:
            for k in POS_KEYS:
                v = self._latest[s][k]
                if v is None:
                    prev = self._metric[s]["pos"][k].array()
                    v = float(prev[-1]) if len(prev) else 0.0
                self._metric[s]["pos"][k].append(float(v))

            for k in ORI_KEYS:
                v = self._latest[s][k]
                if v is None:
                    prev = self._metric[s]["ori"][k].array()
                    v = float(prev[-1]) if len(prev) else (1.0 if k == "qw" else 0.0)
                self._metric[s]["ori"][k].append(float(v))

        self._hz_buf.append(frame.ts_host)
        if len(self._hz_buf) > 120:
            self._hz_buf = self._hz_buf[-120:]

        self._frame_count += 1
        self._dirty = True

        if record and self._rec.active:
            self._rec.append(
                {
                    "type": "pose_record",
                    "ts_host": frame.ts_host,
                    "source": frame.source,
                    "stamp_sec": frame.stamp_sec,
                    "stamp_nsec": frame.stamp_nsec,
                    "frame_id": frame.frame_id,
                    "x": frame.x,
                    "y": frame.y,
                    "z": frame.z,
                    "qx": frame.qx,
                    "qy": frame.qy,
                    "qz": frame.qz,
                    "qw": frame.qw,
                }
            )

    def _apply_rec(self, rec: dict):
        frame = PoseFrame(
            ts_host=float(rec.get("ts_host", 0.0)),
            source=str(rec.get("source", "")),
            stamp_sec=int(rec["stamp_sec"]) if isinstance(rec.get("stamp_sec"), int) else None,
            stamp_nsec=int(rec["stamp_nsec"]) if isinstance(rec.get("stamp_nsec"), int) else None,
            frame_id=str(rec.get("frame_id", "")),
            x=float(rec["x"]) if isinstance(rec.get("x"), (int, float)) else None,
            y=float(rec["y"]) if isinstance(rec.get("y"), (int, float)) else None,
            z=float(rec["z"]) if isinstance(rec.get("z"), (int, float)) else None,
            qx=float(rec["qx"]) if isinstance(rec.get("qx"), (int, float)) else None,
            qy=float(rec["qy"]) if isinstance(rec.get("qy"), (int, float)) else None,
            qz=float(rec["qz"]) if isinstance(rec.get("qz"), (int, float)) else None,
            qw=float(rec["qw"]) if isinstance(rec.get("qw"), (int, float)) else None,
        )
        self._ingest(frame, record=False)

    def _reset_buffers(self):
        self._t_buf.clear()
        self._hz_buf.clear()
        for s in SRC_KEYS:
            for k in POS_KEYS:
                self._metric[s]["pos"][k].clear()
            for k in ORI_KEYS:
                self._metric[s]["ori"][k].clear()
        self._latest = {
            "zed": {k: None for k in (POS_KEYS + ORI_KEYS)},
            "wheel": {k: None for k in (POS_KEYS + ORI_KEYS)},
        }

    def _tick(self):
        if self._replay_mode and self._rplay and self._recs:
            now = time.monotonic()
            elapsed = now - self._rtick
            self._rtick = now
            step = max(1, int(round(elapsed * self._rspeed * 30.0)))
            target = min(len(self._recs) - 1, self._ridx + step)
            for i in range(self._ridx + 1, target + 1):
                self._apply_rec(self._recs[i])
            self._ridx = target

            self._slider.blockSignals(True)
            self._slider.setValue(self._ridx)
            self._slider.blockSignals(False)

            if self._ridx >= len(self._recs) - 1:
                self._rplay = False
                self._play_btn.setText("Play")

        if self._recs:
            t0 = float(self._recs[0].get("ts_host", 0.0))
            ti = float(self._recs[self._ridx].get("ts_host", t0))
            tN = float(self._recs[-1].get("ts_host", t0))
            self._rt_lbl.setText(f"t: {ti - t0:6.2f}s / {tN - t0:6.2f}s")

        if len(self._hz_buf) >= 2:
            span = self._hz_buf[-1] - self._hz_buf[0]
            if span > 1e-6:
                hz = (len(self._hz_buf) - 1) / span
                self._hz_lbl.setText(f"{hz:.1f} Hz")

        self._zed_frame_lbl.setText(
            f"ZED frame: {self._last_frame_id['zed'] or '--'}"
        )
        self._wheel_frame_lbl.setText(
            f"Wheel frame: {self._last_frame_id['wheel'] or '--'}"
        )

        if self._rec.active:
            self.rec_btn.setText(f"Stop [{self._rec.count}]")

        if self._dirty:
            self._render()
            self._dirty = False

    def _render(self):
        t_arr = self._t_buf.array()
        if len(t_arr) < 2:
            return
        tx = (t_arr - t_arr[0]).astype(np.float32)

        for src in SRC_KEYS:
            for k in POS_KEYS:
                ys = self._metric[src]["pos"][k].array().astype(np.float32)
                n = min(len(tx), len(ys))
                if n > 1:
                    self._lines[src]["pos"][k].setData(x=tx[-n:], y=ys[-n:])

            for k in ORI_KEYS:
                ys = self._metric[src]["ori"][k].array().astype(np.float32)
                n = min(len(tx), len(ys))
                if n > 1:
                    self._lines[src]["ori"][k].setData(x=tx[-n:], y=ys[-n:])

    def closeEvent(self, event):
        self._timer.stop()

        if self._rec.active:
            r = QMessageBox.question(
                self,
                "Recording active",
                "Stop and save before closing?",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                QMessageBox.Yes,
            )
            if r == QMessageBox.Cancel:
                self._timer.start(RENDER_MS)
                event.ignore()
                return
            if r == QMessageBox.Yes:
                _, msg = self._rec.stop()
                self._status.showMessage(msg)

        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(2000)

        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QPalette()
    for role, c in [
        (QPalette.Window, C_BG),
        (QPalette.WindowText, C_TEXT),
        (QPalette.Base, C_SURFACE),
        (QPalette.AlternateBase, C_PANEL),
        (QPalette.Text, C_TEXT),
        (QPalette.Button, C_PANEL),
        (QPalette.ButtonText, C_TEXT),
        (QPalette.Highlight, C_ACCENT),
        (QPalette.HighlightedText, C_BG),
    ]:
        pal.setColor(role, QColor(c))
    app.setPalette(pal)

    win = MainWindow()
    win.show()

    signal.signal(signal.SIGINT, lambda *_: (win.close(), app.quit()))
    pulse = QTimer()
    pulse.start(150)
    pulse.timeout.connect(lambda: None)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
