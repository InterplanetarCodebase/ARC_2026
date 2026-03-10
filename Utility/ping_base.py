#!/usr/bin/env python3
"""
PING BASE — Base-station GUI for rover heartbeat monitoring.

Minimal GTK3 + Cairo GUI:
  Left  : Live camera feed from the rover (single H.264 stream via GStreamer).
  Right : Real-time status panel — ping RTT, RSSI, command log, connection state.

Keyboard WASD sends drive commands to the rover.  All interactions are timestamped
and round-trip latency is computed from heartbeat echo.

Protocol (matches ping_rover.py):
  Heartbeat request :  [0xAA 0xBB SEQ_H SEQ_L 0x01 TS8 CRC8]     (14 bytes)
  Heartbeat reply   :  [0xAA 0xBB SEQ_H SEQ_L 0x02 TS8 CRC8]     (14 bytes)
  Command           :  [0xAA 0xBB SEQ_H SEQ_L 0x10 CMD CRC8]     (7 bytes)
  Command ACK       :  [0xAA 0xBB SEQ_H SEQ_L 0x11 CMD TS8 CRC8] (15 bytes)
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('Gtk', '3.0')
from gi.repository import Gst, GstVideo, Gtk, GLib, Gdk

import argparse
import sys
import signal
import time
import struct
import threading
import socket
import math
import datetime
import cairo
import os as _os


# ══════════════════════════════════════════════════════════════════════════════
#  Protocol
# ══════════════════════════════════════════════════════════════════════════════

SOF1 = 0xAA
SOF2 = 0xBB
MSG_HEARTBEAT_REQ = 0x01
MSG_HEARTBEAT_RPL = 0x02
MSG_COMMAND       = 0x10
MSG_COMMAND_ACK   = 0x11

CMD_W = 0x57
CMD_A = 0x41
CMD_S = 0x53
CMD_D = 0x44
CMD_NAMES = {CMD_W: "W", CMD_A: "A", CMD_S: "S", CMD_D: "D"}
KEY_TO_CMD = {
    Gdk.KEY_w: CMD_W, Gdk.KEY_W: CMD_W,
    Gdk.KEY_a: CMD_A, Gdk.KEY_A: CMD_A,
    Gdk.KEY_s: CMD_S, Gdk.KEY_S: CMD_S,
    Gdk.KEY_d: CMD_D, Gdk.KEY_D: CMD_D,
}


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ══════════════════════════════════════════════════════════════════════════════
#  Decoder detection  (same as camera_receiver.py)
# ══════════════════════════════════════════════════════════════════════════════

DECODER_CANDIDATES = [
    {"name": "nvv4l2decoder", "element": "nvv4l2decoder",
     "pipeline": "nvv4l2decoder ! nvvidconv",  "note": "Jetson HW"},
    {"name": "nvdec",         "element": "nvdec",
     "pipeline": "nvdec ! videoconvert",       "note": "NVIDIA HW"},
    {"name": "vaapih264dec",  "element": "vaapih264dec",
     "pipeline": "vaapih264dec ! videoconvert","note": "VA-API HW"},
    {"name": "avdec_h264",    "element": "avdec_h264",
     "pipeline": "avdec_h264 ! videoconvert",  "note": "libav SW"},
    {"name": "openh264dec",   "element": "openh264dec",
     "pipeline": "openh264dec ! videoconvert", "note": "OpenH264 SW"},
]


def detect_decoder():
    for c in DECODER_CANDIDATES:
        if Gst.ElementFactory.find(c["element"]):
            print(f"[decoder] {c['note']}")
            return c
    return None


# ══════════════════════════════════════════════════════════════════════════════
#  Camera backend  (GStreamer → appsink → cairo surface)
# ══════════════════════════════════════════════════════════════════════════════

class CameraBackend:
    def __init__(self, port, width, height, decoder):
        self.port      = port
        self.width     = width
        self.height    = height
        self.decoder   = decoder
        self.pipeline  = None
        self.running   = False
        self.fps       = 0.0
        self.status    = "waiting"
        self._frame_count = 0
        self._fps_ts      = time.monotonic()
        self._last_frame  = None
        self._frame_lock  = threading.Lock()
        self._on_frame    = None  # callback

    def build(self):
        pipeline_str = (
            f"udpsrc port={self.port} buffer-size=2097152 "
            f"caps=\"application/x-rtp,media=video,clock-rate=90000,"
            f"encoding-name=H264,payload=96\" ! "
            f"rtpjitterbuffer latency=100 drop-on-latency=true ! "
            f"rtph264depay ! h264parse ! "
            f"{self.decoder['pipeline']} ! "
            f"videoscale ! videoconvert ! "
            f"video/x-raw,format=BGRx,width={self.width},height={self.height} ! "
            f"appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
        )
        print(f"[cam] {pipeline_str}")
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            print(f"[cam] Pipeline error: {e}")
            self.status = "error"
            return False
        appsink = self.pipeline.get_by_name("sink")
        appsink.connect("new-sample", self._on_new_sample)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        return True

    def start(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.status = "error"
            return False
        self.running = True
        return True

    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.running = False
        self.status = "stopped"

    def _on_new_sample(self, appsink):
        sample = appsink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.OK
        buf    = sample.get_buffer()
        caps   = sample.get_caps()
        struct = caps.get_structure(0)
        w      = struct.get_value("width")
        h      = struct.get_value("height")
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        data_copy = bytes(mapinfo.data)
        buf.unmap(mapinfo)
        surface = cairo.ImageSurface(cairo.FORMAT_RGB24, w, h)
        surface.get_data()[:] = data_copy[:w * h * 4]
        surface.mark_dirty()
        with self._frame_lock:
            self._last_frame = surface
        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_ts
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_ts = now
            self.status = "streaming"
        if self._on_frame:
            GLib.idle_add(self._on_frame)
        return Gst.FlowReturn.OK

    def get_frame(self):
        with self._frame_lock:
            return self._last_frame

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[cam] ERR: {err}")
            self.status = "error"
        elif t == Gst.MessageType.EOS:
            self.status = "stopped"


# ══════════════════════════════════════════════════════════════════════════════
#  Network controller  (heartbeat sender + command sender + reply listener)
# ══════════════════════════════════════════════════════════════════════════════

class NetController:
    HEARTBEAT_INTERVAL = 1.0  # seconds

    def __init__(self, rover_ip, ctrl_port):
        self.rover_ip  = rover_ip
        self.ctrl_port = ctrl_port
        self.sock      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.sock.bind(("0.0.0.0", 0))  # ephemeral port for replies
        self._seq      = 0
        self._running  = False
        self._lock     = threading.Lock()

        # State
        self.rtt_ms        = None        # latest round-trip time
        self.rtt_history   = []          # last 120 samples
        self.rtt_avg       = None
        self.rtt_min       = None
        self.rtt_max       = None
        self.connected     = False
        self.last_rtt_ts   = 0.0
        self.heartbeats_sent = 0
        self.heartbeats_recv = 0
        self.commands_sent   = 0
        self.commands_acked  = 0
        self.command_log     = []        # last 30 entries: (ts_str, cmd_name, ack_ms|None)
        self._pending_cmds   = {}        # seq -> (cmd_byte, send_mono)
        self._last_hb_recv   = 0.0

    def _next_seq(self):
        with self._lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
        return s

    def start(self):
        self._running = True
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def stop(self):
        self._running = False
        self.sock.close()

    def send_command(self, cmd_byte):
        seq = self._next_seq()
        body = bytes([SOF1, SOF2, (seq >> 8) & 0xFF, seq & 0xFF, MSG_COMMAND, cmd_byte])
        pkt = body + bytes([crc8(body)])
        try:
            self.sock.sendto(pkt, (self.rover_ip, self.ctrl_port))
        except OSError:
            return
        self.commands_sent += 1
        self._pending_cmds[seq] = (cmd_byte, time.monotonic())
        # Prune old pending (> 5s)
        cutoff = time.monotonic() - 5.0
        self._pending_cmds = {k: v for k, v in self._pending_cmds.items() if v[1] > cutoff}

    def _heartbeat_loop(self):
        while self._running:
            seq = self._next_seq()
            ts_bytes = struct.pack('!d', time.monotonic())
            body = bytes([SOF1, SOF2, (seq >> 8) & 0xFF, seq & 0xFF,
                          MSG_HEARTBEAT_REQ]) + ts_bytes
            pkt = body + bytes([crc8(body)])
            try:
                self.sock.sendto(pkt, (self.rover_ip, self.ctrl_port))
                self.heartbeats_sent += 1
            except OSError:
                pass
            time.sleep(self.HEARTBEAT_INTERVAL)

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(64)
            except (socket.timeout, OSError):
                # Check connection timeout
                if self._last_hb_recv > 0 and (time.monotonic() - self._last_hb_recv) > 5:
                    self.connected = False
                continue

            if len(data) < 7 or data[0] != SOF1 or data[1] != SOF2:
                continue
            if crc8(data[:-1]) != data[-1]:
                continue

            msg_type = data[4]
            seq = (data[2] << 8) | data[3]

            if msg_type == MSG_HEARTBEAT_RPL and len(data) == 14:
                ts_bytes = data[5:13]
                sent_mono = struct.unpack('!d', ts_bytes)[0]
                rtt = (time.monotonic() - sent_mono) * 1000.0  # ms
                self.rtt_ms = rtt
                self.rtt_history.append(rtt)
                if len(self.rtt_history) > 120:
                    self.rtt_history = self.rtt_history[-120:]
                self.rtt_avg = sum(self.rtt_history) / len(self.rtt_history)
                self.rtt_min = min(self.rtt_history)
                self.rtt_max = max(self.rtt_history)
                self.heartbeats_recv += 1
                self.connected = True
                self._last_hb_recv = time.monotonic()
                self.last_rtt_ts = time.monotonic()

            elif msg_type == MSG_COMMAND_ACK and len(data) == 15:
                cmd_byte = data[5]
                if seq in self._pending_cmds:
                    _, send_mono = self._pending_cmds.pop(seq)
                    ack_ms = (time.monotonic() - send_mono) * 1000.0
                else:
                    ack_ms = None
                cmd_name = CMD_NAMES.get(cmd_byte, f"0x{cmd_byte:02X}")
                ts_str = datetime.datetime.now().strftime("%H:%M:%S")
                ack_str = f"{ack_ms:.1f}ms" if ack_ms is not None else "?"
                self.command_log.append((ts_str, cmd_name, ack_str))
                if len(self.command_log) > 30:
                    self.command_log = self.command_log[-30:]
                self.commands_acked += 1


# ══════════════════════════════════════════════════════════════════════════════
#  GTK GUI
# ══════════════════════════════════════════════════════════════════════════════

CSS = b"""
window           { background: #060a0f; }
.header-box      { background: #070d14; border-bottom: 1px solid #2a1800; }
.title-lbl       { font-family: "Courier New"; font-size: 16px; font-weight: bold;
                   color: #ff8800; letter-spacing: 4px; }
.subtitle-lbl    { font-family: "Courier New"; font-size: 9px; color: #ff8800;
                   letter-spacing: 3px; }
.badge           { font-family: "Courier New"; font-size: 11px; color: #ff8800;
                   background: #120800; border: 1px solid #3a1800;
                   border-radius: 4px; padding: 3px 10px; }
.statusbar       { background: #040608; border-top: 1px solid #1a0e00;
                   padding: 0 12px; }
.status-lbl      { font-family: "Courier New"; font-size: 10px; color: #ff8800; }
"""


class PingBaseApp(Gtk.Window):
    def __init__(self, rover_ip, ctrl_port, cam_port, cam_w, cam_h):
        super().__init__(title="PING BASE — Rover Link Monitor")
        self.rover_ip  = rover_ip
        self.ctrl_port = ctrl_port
        self.cam_port  = cam_port
        self.cam_w     = cam_w
        self.cam_h     = cam_h

        # Keyboard state for WASD
        self._keys_held = set()
        self._key_send_interval = 0.1  # seconds
        self._last_key_send = {}

        Gst.init(None)
        self._apply_css()

        # Network
        self.net = NetController(rover_ip, ctrl_port)
        self.net.start()

        # Camera
        self.cam = None
        decoder = detect_decoder()
        if decoder:
            self.cam = CameraBackend(cam_port, cam_w, cam_h, decoder)
            if self.cam.build():
                self.cam._on_frame = self._on_cam_frame
                self.cam.start()
            else:
                print("[base] Camera pipeline build failed")
                self.cam = None

        self._build_ui()
        self.connect("destroy", self._on_destroy)
        self.connect("key-press-event", self._on_key_press)
        self.connect("key-release-event", self._on_key_release)
        self.set_default_size(1200, 700)
        self.show_all()

        GLib.timeout_add(100, self._tick)

    def _apply_css(self):
        p = Gtk.CssProvider()
        p.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), p, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    # ── UI Layout ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(root)

        # Header
        hdr = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        hdr.get_style_context().add_class("header-box")
        hdr.set_size_request(-1, 48)
        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        left.set_valign(Gtk.Align.CENTER)
        left.set_margin_start(14)
        t = Gtk.Label(label="◈ PING BASE")
        t.get_style_context().add_class("title-lbl")
        t.set_halign(Gtk.Align.START)
        s = Gtk.Label(label="ROVER LINK MONITOR")
        s.get_style_context().add_class("subtitle-lbl")
        s.set_halign(Gtk.Align.START)
        left.pack_start(t, False, False, 0)
        left.pack_start(s, False, False, 0)
        hdr.pack_start(left, False, False, 0)
        hdr.pack_start(Gtk.Box(), True, True, 0)
        self.conn_badge = Gtk.Label(label="DISCONNECTED")
        self.conn_badge.get_style_context().add_class("badge")
        self.conn_badge.set_valign(Gtk.Align.CENTER)
        self.conn_badge.set_margin_end(14)
        hdr.pack_start(self.conn_badge, False, False, 0)
        root.pack_start(hdr, False, False, 0)

        # Main body: left = camera, right = info panel
        body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        body.set_margin_start(4)
        body.set_margin_end(4)
        body.set_margin_top(4)
        body.set_margin_bottom(4)
        body.set_spacing(4)

        # Camera drawing area
        self.cam_area = Gtk.DrawingArea()
        self.cam_area.set_hexpand(True)
        self.cam_area.set_vexpand(True)
        self.cam_area.connect("draw", self._draw_camera)
        body.pack_start(self.cam_area, True, True, 0)

        # Info panel drawing area (fixed width)
        self.info_area = Gtk.DrawingArea()
        self.info_area.set_size_request(420, -1)
        self.info_area.set_vexpand(True)
        self.info_area.connect("draw", self._draw_info)
        body.pack_start(self.info_area, False, False, 0)

        root.pack_start(body, True, True, 0)

        # Status bar
        sb = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        sb.get_style_context().add_class("statusbar")
        sb.set_size_request(-1, 22)
        self.status_lbl = Gtk.Label(label="WASD = drive commands  |  Heartbeat: 1 Hz")
        self.status_lbl.get_style_context().add_class("status-lbl")
        self.status_lbl.set_halign(Gtk.Align.START)
        self.clock_lbl = Gtk.Label(label="")
        self.clock_lbl.get_style_context().add_class("status-lbl")
        self.clock_lbl.set_halign(Gtk.Align.END)
        sb.pack_start(self.status_lbl, True, True, 0)
        sb.pack_end(self.clock_lbl, False, False, 0)
        root.pack_end(sb, False, False, 0)

    # ── Camera draw ───────────────────────────────────────────────────────────
    def _draw_camera(self, widget, cr):
        W = widget.get_allocated_width()
        H = widget.get_allocated_height()
        # Background
        cr.set_source_rgb(0.04, 0.06, 0.09)
        cr.rectangle(0, 0, W, H)
        cr.fill()

        frame = self.cam.get_frame() if self.cam else None
        if frame:
            fw, fh = frame.get_width(), frame.get_height()
            scale = min(W / fw, H / fh)
            dx = (W - fw * scale) / 2
            dy = (H - fh * scale) / 2
            cr.save()
            cr.translate(dx, dy)
            cr.scale(scale, scale)
            cr.set_source_surface(frame, 0, 0)
            cr.paint()
            cr.restore()
            # FPS overlay
            self._draw_cam_overlay(cr, W, H)
        else:
            self._draw_placeholder(cr, W, H)

        # Border
        cr.set_source_rgba(0.12, 0.22, 0.32, 0.7)
        cr.set_line_width(1)
        cr.rectangle(0.5, 0.5, W - 1, H - 1)
        cr.stroke()

    def _draw_cam_overlay(self, cr, W, H):
        bh = max(20, H * 0.06)
        cr.save()
        cr.set_source_rgba(0.0, 0.03, 0.07, 0.84)
        cr.rectangle(0, H - bh, W, bh)
        cr.fill()
        fs = max(9, bh * 0.45)
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(fs)
        ty = H - bh / 2 + fs / 2
        # Port
        cr.set_source_rgba(1.0, 0.55, 0.05, 0.95)
        cr.move_to(8, ty)
        cr.show_text(f":{self.cam_port}")
        # FPS
        fps_val = self.cam.fps if self.cam else 0
        fps_str = f"{fps_val:.1f} FPS"
        if fps_val >= 25:
            fc = (1.0, 0.55, 0.05)
        elif fps_val >= 10:
            fc = (0.92, 0.72, 0.08)
        else:
            fc = (0.95, 0.22, 0.18)
        ext = cr.text_extents(fps_str)
        cr.set_source_rgba(*fc, 0.95)
        cr.move_to(W / 2 - ext.width / 2, ty)
        cr.show_text(fps_str)
        # Status
        cam_st = self.cam.status if self.cam else "N/A"
        cr.set_source_rgba(1.0, 0.55, 0.05, 0.8)
        ext2 = cr.text_extents(cam_st.upper())
        cr.move_to(W - ext2.width - 8, ty)
        cr.show_text(cam_st.upper())
        cr.restore()

    def _draw_placeholder(self, cr, W, H):
        cr.set_source_rgb(0.06, 0.09, 0.13)
        cr.rectangle(0, 0, W, H)
        cr.fill()
        # Grid
        cr.save()
        cr.set_source_rgba(0.14, 0.22, 0.30, 0.5)
        cr.set_line_width(0.5)
        step = max(20, min(W, H) // 16)
        for x in range(0, W, step):
            cr.move_to(x, 0); cr.line_to(x, H)
        for y in range(0, H, step):
            cr.move_to(0, y); cr.line_to(W, y)
        cr.stroke()
        cr.restore()
        # Text
        cx, cy = W / 2, H / 2
        fs = max(10, min(W, H) * 0.04)
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(fs)
        txt = "WAITING FOR ROVER FEED"
        ext = cr.text_extents(txt)
        cr.set_source_rgba(1.0, 0.53, 0.02, 0.4)
        cr.move_to(cx - ext.width / 2, cy)
        cr.show_text(txt)

    # ── Info panel draw ───────────────────────────────────────────────────────
    def _draw_info(self, widget, cr):
        W = widget.get_allocated_width()
        H = widget.get_allocated_height()
        cr.set_source_rgb(0.03, 0.05, 0.08)
        cr.rectangle(0, 0, W, H)
        cr.fill()
        cr.set_source_rgba(0.12, 0.22, 0.32, 0.7)
        cr.set_line_width(1)
        cr.rectangle(0.5, 0.5, W - 1, H - 1)
        cr.stroke()

        pad = 14
        y = pad
        y = self._section_connection(cr, pad, y, W - 2 * pad)
        y = self._section_ping(cr, pad, y, W - 2 * pad)
        y = self._section_rtt_graph(cr, pad, y, W - 2 * pad, min(120, H // 5))
        y = self._section_commands(cr, pad, y, W - 2 * pad)
        self._section_wasd(cr, pad, y, W - 2 * pad)

    def _section_header(self, cr, x, y, title):
        cr.save()
        cr.set_source_rgba(1.0, 0.55, 0.05, 0.9)
        cr.set_line_width(2)
        cr.move_to(x, y + 6)
        cr.line_to(x, y + 18)
        cr.stroke()
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(12)
        cr.move_to(x + 8, y + 16)
        cr.show_text(title)
        cr.restore()
        return y + 26

    def _kv(self, cr, x, y, key, val, val_color=(1.0, 0.55, 0.05)):
        cr.save()
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(11)
        cr.set_source_rgba(0.6, 0.4, 0.15, 0.9)
        cr.move_to(x, y)
        cr.show_text(key)
        cr.set_source_rgba(*val_color, 0.95)
        cr.move_to(x + 140, y)
        cr.show_text(str(val))
        cr.restore()
        return y + 16

    def _section_connection(self, cr, x, y, w):
        y = self._section_header(cr, x, y, "CONNECTION")
        net = self.net
        conn_str = "CONNECTED" if net.connected else "DISCONNECTED"
        conn_c = (0.2, 0.9, 0.2) if net.connected else (0.95, 0.22, 0.18)
        y = self._kv(cr, x, y, "STATUS", conn_str, conn_c)
        y = self._kv(cr, x, y, "ROVER IP", self.rover_ip)
        y = self._kv(cr, x, y, "CTRL PORT", str(self.ctrl_port))
        y = self._kv(cr, x, y, "CAM PORT", str(self.cam_port))
        cam_st = self.cam.status.upper() if self.cam else "N/A"
        y = self._kv(cr, x, y, "CAM STATUS", cam_st)
        return y + 6

    def _section_ping(self, cr, x, y, w):
        y = self._section_header(cr, x, y, "PING / LATENCY")
        net = self.net
        rtt_str = f"{net.rtt_ms:.1f} ms" if net.rtt_ms is not None else "---"
        rtt_c = (1.0, 0.55, 0.05)
        if net.rtt_ms is not None:
            if net.rtt_ms > 80:
                rtt_c = (0.95, 0.22, 0.18)
            elif net.rtt_ms > 30:
                rtt_c = (0.92, 0.72, 0.08)
        y = self._kv(cr, x, y, "RTT", rtt_str, rtt_c)
        avg_str = f"{net.rtt_avg:.1f} ms" if net.rtt_avg is not None else "---"
        y = self._kv(cr, x, y, "RTT AVG", avg_str)
        min_str = f"{net.rtt_min:.1f} ms" if net.rtt_min is not None else "---"
        max_str = f"{net.rtt_max:.1f} ms" if net.rtt_max is not None else "---"
        y = self._kv(cr, x, y, "RTT MIN", min_str)
        y = self._kv(cr, x, y, "RTT MAX", max_str)
        loss = 0
        if net.heartbeats_sent > 0:
            loss = max(0, 100.0 * (1.0 - net.heartbeats_recv / net.heartbeats_sent))
        loss_c = (0.2, 0.9, 0.2) if loss < 5 else ((0.92, 0.72, 0.08) if loss < 20 else (0.95, 0.22, 0.18))
        y = self._kv(cr, x, y, "PACKET LOSS", f"{loss:.1f}%", loss_c)
        y = self._kv(cr, x, y, "HB SENT/RECV", f"{net.heartbeats_sent}/{net.heartbeats_recv}")
        return y + 6

    def _section_rtt_graph(self, cr, x, y, w, h):
        y = self._section_header(cr, x, y, "RTT GRAPH")
        # Graph background
        gx, gy, gw, gh = x, y, w, h
        cr.save()
        cr.set_source_rgba(0.02, 0.04, 0.07, 1.0)
        cr.rectangle(gx, gy, gw, gh)
        cr.fill()
        cr.set_source_rgba(0.12, 0.22, 0.32, 0.5)
        cr.set_line_width(0.5)
        cr.rectangle(gx, gy, gw, gh)
        cr.stroke()
        # Horizontal guides
        cr.set_source_rgba(0.15, 0.25, 0.35, 0.4)
        for frac in (0.25, 0.5, 0.75):
            ly = gy + int(frac * gh)
            cr.move_to(gx, ly)
            cr.line_to(gx + gw, ly)
        cr.stroke()

        data = self.net.rtt_history
        if len(data) >= 2:
            max_rtt = max(data) * 1.2 if max(data) > 0 else 100
            cr.set_source_rgba(1.0, 0.55, 0.05, 0.9)
            cr.set_line_width(1.5)
            n = len(data)
            for i, rtt in enumerate(data):
                px = gx + (i / max(n - 1, 1)) * gw
                py = gy + gh - (rtt / max_rtt) * gh
                py = max(gy, min(gy + gh, py))
                if i == 0:
                    cr.move_to(px, py)
                else:
                    cr.line_to(px, py)
            cr.stroke()
            # Scale label
            cr.set_font_size(9)
            cr.set_source_rgba(0.5, 0.35, 0.1, 0.8)
            cr.move_to(gx + 4, gy + 12)
            cr.show_text(f"{max_rtt:.0f}ms")
            cr.move_to(gx + 4, gy + gh - 4)
            cr.show_text("0ms")

        cr.restore()
        return y + h + 10

    def _section_commands(self, cr, x, y, w):
        y = self._section_header(cr, x, y, "COMMAND LOG")
        net = self.net
        y = self._kv(cr, x, y, "SENT / ACKED", f"{net.commands_sent} / {net.commands_acked}")
        # Show last ~8 entries
        cr.save()
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(10)
        visible = net.command_log[-8:]
        for ts, cmd, ack in visible:
            cr.set_source_rgba(0.5, 0.35, 0.1, 0.8)
            cr.move_to(x, y)
            cr.show_text(f"{ts}")
            cr.set_source_rgba(1.0, 0.55, 0.05, 0.95)
            cr.move_to(x + 80, y)
            cr.show_text(f"CMD {cmd}")
            cr.set_source_rgba(0.2, 0.8, 0.2, 0.9)
            cr.move_to(x + 160, y)
            cr.show_text(f"ACK {ack}")
            y += 14
        cr.restore()
        return y + 6

    def _section_wasd(self, cr, x, y, w):
        y = self._section_header(cr, x, y, "DRIVE CONTROLS")
        # Draw WASD keys
        ks = 36
        gap = 4
        cx = x + w // 2
        keys = [
            ("W", cx - ks // 2,          y,            CMD_W),
            ("A", cx - ks // 2 - ks - gap, y + ks + gap, CMD_A),
            ("S", cx - ks // 2,          y + ks + gap, CMD_S),
            ("D", cx - ks // 2 + ks + gap, y + ks + gap, CMD_D),
        ]
        cr.save()
        for label, kx, ky, cmd in keys:
            pressed = cmd in self._keys_held
            if pressed:
                cr.set_source_rgba(1.0, 0.55, 0.05, 0.8)
            else:
                cr.set_source_rgba(0.06, 0.08, 0.12, 1.0)
            self._rrect(cr, kx, ky, ks, ks, 4)
            cr.fill()
            bc = (1.0, 0.55, 0.05, 0.9) if pressed else (0.3, 0.18, 0.05, 0.8)
            cr.set_source_rgba(*bc)
            cr.set_line_width(1.5)
            self._rrect(cr, kx, ky, ks, ks, 4)
            cr.stroke()
            fs = 14
            cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
            cr.set_font_size(fs)
            ext = cr.text_extents(label)
            tc = (0.0, 0.0, 0.0) if pressed else (1.0, 0.55, 0.05)
            cr.set_source_rgba(*tc, 0.95)
            cr.move_to(kx + ks / 2 - ext.width / 2 - ext.x_bearing,
                       ky + ks / 2 - ext.height / 2 - ext.y_bearing)
            cr.show_text(label)
        cr.restore()

    def _rrect(self, cr, x, y, w, h, r):
        cr.move_to(x + r, y)
        cr.line_to(x + w - r, y)
        cr.arc(x + w - r, y + r,     r, -math.pi / 2, 0)
        cr.line_to(x + w, y + h - r)
        cr.arc(x + w - r, y + h - r, r, 0,             math.pi / 2)
        cr.line_to(x + r, y + h)
        cr.arc(x + r,     y + h - r, r, math.pi / 2,   math.pi)
        cr.line_to(x,     y + r)
        cr.arc(x + r,     y + r,     r, math.pi,       3 * math.pi / 2)
        cr.close_path()

    # ── Keyboard ──────────────────────────────────────────────────────────────
    def _on_key_press(self, widget, event):
        if event.keyval in KEY_TO_CMD:
            cmd = KEY_TO_CMD[event.keyval]
            self._keys_held.add(cmd)
            now = time.monotonic()
            last = self._last_key_send.get(cmd, 0)
            if now - last >= self._key_send_interval:
                self.net.send_command(cmd)
                self._last_key_send[cmd] = now
        if event.keyval == Gdk.KEY_Escape:
            self._on_destroy(None)

    def _on_key_release(self, widget, event):
        if event.keyval in KEY_TO_CMD:
            cmd = KEY_TO_CMD[event.keyval]
            self._keys_held.discard(cmd)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _on_cam_frame(self):
        self.cam_area.queue_draw()

    def _tick(self):
        self.info_area.queue_draw()
        self.cam_area.queue_draw()
        conn_str = "CONNECTED" if self.net.connected else "DISCONNECTED"
        self.conn_badge.set_text(conn_str)
        self.clock_lbl.set_text(datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        # Repeat held keys
        now = time.monotonic()
        for cmd in list(self._keys_held):
            last = self._last_key_send.get(cmd, 0)
            if now - last >= self._key_send_interval:
                self.net.send_command(cmd)
                self._last_key_send[cmd] = now
        return True

    def _on_destroy(self, _):
        self.net.stop()
        if self.cam:
            self.cam.stop()
        Gtk.main_quit()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def signal_handler(sig, frame):
    print("\nStopping...")
    Gtk.main_quit()
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(
        description="PING BASE — Rover Link Monitor GUI",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="""
Example:
  python3 ping_base.py -i 192.168.1.50
  python3 ping_base.py -i 192.168.1.50 --ctrl-port 6000 --cam-port 5000
        """)
    parser.add_argument("-i", "--rover-ip", required=True,
                        help="Rover IP address")
    parser.add_argument("--ctrl-port", type=int, default=6000,
                        help="UDP port for heartbeat / commands")
    parser.add_argument("--cam-port", type=int, default=5000,
                        help="UDP port for inbound camera stream")
    parser.add_argument("-w", "--width", type=int, default=640,
                        help="Decode width for camera feed")
    parser.add_argument("-H", "--height", type=int, default=480,
                        help="Decode height for camera feed")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    app = PingBaseApp(
        rover_ip=args.rover_ip,
        ctrl_port=args.ctrl_port,
        cam_port=args.cam_port,
        cam_w=args.width,
        cam_h=args.height,
    )
    Gtk.main()


if __name__ == "__main__":
    main()
