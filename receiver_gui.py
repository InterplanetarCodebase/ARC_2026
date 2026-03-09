#!/usr/bin/env python3
"""
SENTINEL — GStreamer Multi-Camera GUI Receiver
Jetson-optimized. Auto-detects decoder.
4×4 grid view with per-feed FPS / ping / port overlay.
Click any tile to maximize; ESC or button to return to grid.
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
import threading
import subprocess
import math
import datetime
import cairo


# ══════════════════════════════════════════════════════════════════════════════
#  Decoder detection
# ══════════════════════════════════════════════════════════════════════════════

DECODER_CANDIDATES = [
    {"name": "nvv4l2decoder", "element": "nvv4l2decoder",
     "pipeline": "nvv4l2decoder ! nvvidconv",  "note": "Jetson HW (nvv4l2decoder)"},
    {"name": "nvdec",         "element": "nvdec",
     "pipeline": "nvdec ! videoconvert",       "note": "NVIDIA HW (nvdec)"},
    {"name": "vaapih264dec",  "element": "vaapih264dec",
     "pipeline": "vaapih264dec ! videoconvert","note": "VA-API HW (vaapih264dec)"},
    {"name": "avdec_h264",    "element": "avdec_h264",
     "pipeline": "avdec_h264 ! videoconvert",  "note": "libav SW (avdec_h264)"},
    {"name": "openh264dec",   "element": "openh264dec",
     "pipeline": "openh264dec ! videoconvert", "note": "OpenH264 SW (openh264dec)"},
]

def detect_decoder():
    for c in DECODER_CANDIDATES:
        if Gst.ElementFactory.find(c["element"]):
            print(f"[decoder] {c['note']}")
            return c
    return None


# ══════════════════════════════════════════════════════════════════════════════
#  Ping helper
# ══════════════════════════════════════════════════════════════════════════════

def ping_once(host, timeout=1):
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(timeout), host],
            capture_output=True, text=True, timeout=timeout + 1
        )
        for line in result.stdout.splitlines():
            if "time=" in line:
                return float(line.split("time=")[1].split()[0])
    except Exception:
        pass
    return None


# ══════════════════════════════════════════════════════════════════════════════
#  Camera backend  (GStreamer → appsink → cairo surface)
# ══════════════════════════════════════════════════════════════════════════════

class CameraBackend:
    def __init__(self, camera_id, port, width, height, decoder, host="127.0.0.1"):
        self.camera_id   = camera_id
        self.port        = port
        self.width       = width
        self.height      = height
        self.decoder     = decoder
        self.host        = host

        self.pipeline    = None
        self.running     = False
        self.fps         = 0.0
        self.ping_ms     = None
        self.status      = "waiting"   # waiting | streaming | error | stopped

        self._frame_count = 0
        self._fps_ts      = time.monotonic()
        self._last_frame  = None
        self._frame_lock  = threading.Lock()

        self.on_new_frame    = None
        self.on_stats_update = None

    # ── Build pipeline ────────────────────────────────────────────────────────
    def build(self):
        pipeline_str = (
            f"udpsrc port={self.port} "
            f"caps=\"application/x-rtp,media=video,clock-rate=90000,"
            f"encoding-name=H264,payload=96\" ! "
            f"rtph264depay ! h264parse ! "
            f"{self.decoder['pipeline']} ! "
            f"videoscale ! videoconvert ! "
            f"video/x-raw,format=BGRx,width={self.width},height={self.height} ! "
            f"appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false"
        )
        print(f"[cam{self.camera_id}] {pipeline_str}")
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            print(f"[cam{self.camera_id}] Pipeline error: {e}")
            self.status = "error"
            return False

        appsink = self.pipeline.get_by_name("sink")
        appsink.connect("new-sample", self._on_new_sample)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        return True

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def start(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.status = "error"
            return False
        self.running = True
        threading.Thread(target=self._ping_loop, daemon=True).start()
        return True

    def stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        self.running = False
        self.status  = "stopped"

    # ── Frame handling ────────────────────────────────────────────────────────
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

        # BGRx → cairo FORMAT_RGB24 (same byte layout on LE)
        surface = cairo.ImageSurface(cairo.FORMAT_RGB24, w, h)
        surface.get_data()[:] = data_copy[:w * h * 4]
        surface.mark_dirty()

        with self._frame_lock:
            self._last_frame = surface

        # FPS
        self._frame_count += 1
        now     = time.monotonic()
        elapsed = now - self._fps_ts
        if elapsed >= 1.0:
            self.fps          = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_ts      = now
            self.status       = "streaming"
            if self.on_stats_update:
                GLib.idle_add(self.on_stats_update, self.camera_id)

        if self.on_new_frame:
            GLib.idle_add(self.on_new_frame, self.camera_id)

        return Gst.FlowReturn.OK

    def get_frame(self):
        with self._frame_lock:
            return self._last_frame

    # ── Ping ─────────────────────────────────────────────────────────────────
    def _ping_loop(self):
        while self.running:
            self.ping_ms = ping_once(self.host, timeout=2)
            if self.on_stats_update:
                GLib.idle_add(self.on_stats_update, self.camera_id)
            time.sleep(3)

    # ── Bus ──────────────────────────────────────────────────────────────────
    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[cam{self.camera_id}] ERR: {err} | {dbg}")
            self.status = "error"
            if self.on_stats_update:
                GLib.idle_add(self.on_stats_update, self.camera_id)
        elif t == Gst.MessageType.EOS:
            self.status = "stopped"


# ══════════════════════════════════════════════════════════════════════════════
#  Camera Tile  (Cairo-rendered GTK DrawingArea)
# ══════════════════════════════════════════════════════════════════════════════

STATUS_COLOR = {
    "waiting":   (0.92, 0.72, 0.08),
    "streaming": (1.00, 0.55, 0.05),
    "error":     (0.95, 0.22, 0.18),
    "stopped":   (0.45, 0.45, 0.45),
}

class CameraTile(Gtk.DrawingArea):
    def __init__(self, backend: CameraBackend, is_large=False):
        super().__init__()
        self.backend   = backend
        self.is_large  = is_large
        self.hovered   = False
        self.maximized = False

        self.set_can_focus(True)
        self.add_events(
            Gdk.EventMask.BUTTON_PRESS_MASK |
            Gdk.EventMask.ENTER_NOTIFY_MASK |
            Gdk.EventMask.LEAVE_NOTIFY_MASK
        )
        self.connect("draw",               self._draw)
        self.connect("enter-notify-event", lambda w, e: self._hover(True))
        self.connect("leave-notify-event", lambda w, e: self._hover(False))

        backend.on_new_frame    = lambda cid: self.queue_draw()
        backend.on_stats_update = lambda cid: self.queue_draw()

    def _hover(self, v):
        self.hovered = v
        self.queue_draw()

    # ── Main draw ─────────────────────────────────────────────────────────────
    def _draw(self, widget, cr):
        W = widget.get_allocated_width()
        H = widget.get_allocated_height()

        self._bg(cr, W, H)
        frame = self.backend.get_frame()
        if frame:
            self._blit_frame(cr, frame, W, H)
        else:
            self._placeholder(cr, W, H)

        self._scanlines(cr, W, H)
        self._border(cr, W, H)
        self._cam_badge(cr, W, H)
        self._status_dot(cr, W, H)
        self._stats_bar(cr, W, H)
        if self.hovered and not self.maximized:
            self._expand_hint(cr, W, H)

    # ── Sub-draws ─────────────────────────────────────────────────────────────
    def _bg(self, cr, W, H):
        cr.set_source_rgb(0.04, 0.06, 0.09)
        cr.rectangle(0, 0, W, H)
        cr.fill()

    def _blit_frame(self, cr, frame, W, H):
        fw = frame.get_width()
        fh = frame.get_height()
        scale = min(W / fw, H / fh)
        dx    = (W - fw * scale) / 2
        dy    = (H - fh * scale) / 2
        cr.save()
        cr.translate(dx, dy)
        cr.scale(scale, scale)
        cr.set_source_surface(frame, 0, 0)
        cr.paint()
        cr.restore()

    def _placeholder(self, cr, W, H):
        # Dark grid bg
        cr.set_source_rgb(0.06, 0.09, 0.13)
        cr.rectangle(0, 0, W, H)
        cr.fill()
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

        # Camera icon
        cx, cy = W / 2, H / 2
        r = min(W, H) * 0.15
        cr.save()
        cr.set_source_rgba(0.25, 0.38, 0.50, 0.55)
        cr.set_line_width(max(1.5, r * 0.08))
        cr.arc(cx, cy - r * 0.08, r, 0, 2 * math.pi)
        cr.stroke()
        cr.arc(cx, cy - r * 0.08, r * 0.42, 0, 2 * math.pi)
        cr.fill()
        cr.restore()

        fs = max(7, min(W, H) * 0.042)
        cr.save()
        cr.set_source_rgba(0.35, 0.52, 0.62, 0.65)
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(fs)
        text = "WAITING FOR STREAM"
        ext  = cr.text_extents(text)
        cr.move_to(cx - ext.width / 2, cy + r * 1.65)
        cr.show_text(text)
        cr.restore()

    def _scanlines(self, cr, W, H):
        step = max(2, H // 120)
        cr.save()
        for y in range(0, H, step * 2):
            cr.set_source_rgba(0, 0, 0, 0.06)
            cr.rectangle(0, y, W, step)
            cr.fill()
        cr.restore()

    def _border(self, cr, W, H):
        if self.maximized:
            cr.set_source_rgba(1.0, 0.53, 0.0, 0.9)
            cr.set_line_width(3)
        elif self.hovered:
            cr.set_source_rgba(1.0, 0.53, 0.0, 0.6)
            cr.set_line_width(2)
        else:
            cr.set_source_rgba(0.12, 0.22, 0.32, 0.7)
            cr.set_line_width(1)
        cr.rectangle(0.5, 0.5, W - 1, H - 1)
        cr.stroke()

    def _cam_badge(self, cr, W, H):
        label = f"CAM {self.backend.camera_id:02d}"
        fs    = max(8, min(W, H) * 0.06)
        cr.save()
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(fs)
        ext = cr.text_extents(label)
        pad = max(3, fs * 0.35)
        bw  = ext.width  + pad * 2
        bh  = ext.height + pad * 2
        x, y = 7, 7
        cr.set_source_rgba(0.85, 0.40, 0.0, 0.82)
        self._rrect(cr, x, y, bw, bh, 3)
        cr.fill()
        cr.set_source_rgba(1, 1, 1, 0.95)
        cr.move_to(x + pad, y + bh - pad - 1)
        cr.show_text(label)
        cr.restore()

    def _status_dot(self, cr, W, H):
        sc  = STATUS_COLOR.get(self.backend.status, (0.5, 0.5, 0.5))
        cx  = W - 13
        cy  = 13
        cr.save()
        cr.set_source_rgba(*sc, 0.30)
        cr.arc(cx, cy, 9, 0, 2 * math.pi)
        cr.fill()
        cr.set_source_rgba(*sc, 0.92)
        cr.arc(cx, cy, 5, 0, 2 * math.pi)
        cr.fill()
        cr.restore()

    def _stats_bar(self, cr, W, H):
        bh  = max(20, H * 0.10)
        pad = 6
        fs  = max(8, bh * 0.50)

        cr.save()
        cr.set_source_rgba(0.00, 0.03, 0.07, 0.84)
        cr.rectangle(0, H - bh, W, bh)
        cr.fill()

        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(fs)
        ty = H - bh / 2 + fs / 2

        # Port  (left, orange)
        cr.set_source_rgba(1.00, 0.55, 0.05, 0.95)
        cr.move_to(pad, ty)
        cr.show_text(f":{self.backend.port}")

        # FPS  (centre, colour-coded — orange-hi / amber-mid / red-low)
        fps_str = f"{self.backend.fps:.1f} FPS"
        if self.backend.fps >= 25:
            fc = (1.00, 0.55, 0.05)
        elif self.backend.fps >= 10:
            fc = (0.92, 0.72, 0.08)
        else:
            fc = (0.95, 0.22, 0.18)
        ext = cr.text_extents(fps_str)
        cr.set_source_rgba(*fc, 0.95)
        cr.move_to(W / 2 - ext.width / 2, ty)
        cr.show_text(fps_str)

        # Ping  (right, colour-coded)
        if self.backend.ping_ms is not None:
            ping_str = f"{self.backend.ping_ms:.0f}ms"
            if self.backend.ping_ms < 30:
                pc = (1.00, 0.55, 0.05)
            elif self.backend.ping_ms < 80:
                pc = (0.92, 0.72, 0.08)
            else:
                pc = (0.95, 0.22, 0.18)
        else:
            ping_str = "---ms"
            pc = (0.40, 0.40, 0.40)
        ext = cr.text_extents(ping_str)
        cr.set_source_rgba(*pc, 0.95)
        cr.move_to(W - ext.width - pad, ty)
        cr.show_text(ping_str)

        cr.restore()

    def _expand_hint(self, cr, W, H):
        s, m = 14, 10
        x, y = W - m - s, m
        cr.save()
        cr.set_source_rgba(1, 1, 1, 0.55)
        cr.set_line_width(1.5)
        for dx, dy in [(0,0),(s,0),(0,s),(s,s)]:
            ax, ay = x + dx, y + dy
            ex = 4 * (1 if dx == 0 else -1)
            ey = 4 * (1 if dy == 0 else -1)
            cr.move_to(ax, ay); cr.line_to(ax + ex, ay)
            cr.move_to(ax, ay); cr.line_to(ax,      ay + ey)
        cr.stroke()
        cr.restore()

    def _rrect(self, cr, x, y, w, h, r):
        cr.move_to(x + r, y)
        cr.line_to(x + w - r, y)
        cr.arc(x + w - r, y + r,     r, -math.pi/2, 0)
        cr.line_to(x + w, y + h - r)
        cr.arc(x + w - r, y + h - r, r,  0,          math.pi/2)
        cr.line_to(x + r, y + h)
        cr.arc(x + r,     y + h - r, r,  math.pi/2,  math.pi)
        cr.line_to(x,     y + r)
        cr.arc(x + r,     y + r,     r,  math.pi,    3*math.pi/2)
        cr.close_path()


# ══════════════════════════════════════════════════════════════════════════════
#  Main Application Window
# ══════════════════════════════════════════════════════════════════════════════

CSS = b"""
window          { background: #060a0f; }
#header         { background: #070d14; border-bottom: 1px solid #2a1800; }
#title          { font-family: "Courier New"; font-size: 16px; font-weight: bold;
                  color: #ff8800; letter-spacing: 4px; }
#subtitle       { font-family: "Courier New"; font-size: 9px; color: #4a3010;
                  letter-spacing: 3px; }
#active-badge   { font-family: "Courier New"; font-size: 11px; color: #ff8800;
                  background: #120800; border: 1px solid #3a1800;
                  border-radius: 4px; padding: 3px 10px; }
#back-btn       { font-family: "Courier New"; font-size: 11px; color: #ff8800;
                  background: transparent; border: 1px solid #3a1800;
                  border-radius: 4px; padding: 4px 14px; margin: 6px; }
#back-btn:hover { background: #1a0e00; border-color: #ff8800; }
#statusbar      { background: #040608; border-top: 1px solid #1a0e00; padding: 0 12px; }
#status-lbl     { font-family: "Courier New"; font-size: 10px; color: #4a2e10; }
#clock-lbl      { font-family: "Courier New"; font-size: 10px; color: #4a2e10; }
"""


class ReceiverApp(Gtk.Window):
    COLS = 4

    def __init__(self, ports, width, height, host):
        super().__init__(title="INTERPLANETAR  ◈  Multi-Camera Receiver")
        self.ports    = ports
        self.width    = width
        self.height   = height
        self.host     = host

        self.backends  = []
        self.tiles     = []       # grid tiles
        self.max_tile  = None     # large tile in maximized view
        self.max_id    = None     # currently maximized camera_id

        Gst.init(None)
        self._apply_css()
        self._build_ui()
        self._init_streams()

        self.connect("destroy",        self._on_destroy)
        self.connect("key-press-event",self._on_key)
        self.set_default_size(1280, 820)
        self.show_all()

        GLib.timeout_add(500, self._tick)

    # ── CSS ──────────────────────────────────────────────────────────────────
    def _apply_css(self):
        p = Gtk.CssProvider()
        p.load_from_data(CSS)
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), p,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

    # ── UI ───────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.add(root)

        # Header
        hdr = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        hdr.set_name("header")
        hdr.set_size_request(-1, 52)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        left.set_valign(Gtk.Align.CENTER)
        left.set_margin_start(14)
        left.set_margin_top(5)
        left.set_margin_bottom(5)
        t = Gtk.Label(label="◈ INTERPLANETAR"); t.set_name("title"); t.set_halign(Gtk.Align.START)
        s = Gtk.Label(label="MULTI-CAMERA SURVEILLANCE SYSTEM"); s.set_name("subtitle"); s.set_halign(Gtk.Align.START)
        left.pack_start(t, False, False, 0)
        left.pack_start(s, False, False, 0)
        hdr.pack_start(left, False, False, 0)
        hdr.pack_start(Gtk.Box(), True, True, 0)  # spacer

        self.active_badge = Gtk.Label(label=f"0 / {len(self.ports)} ACTIVE")
        self.active_badge.set_name("active-badge")
        self.active_badge.set_valign(Gtk.Align.CENTER)
        self.active_badge.set_margin_end(12)
        hdr.pack_start(self.active_badge, False, False, 0)

        self.back_btn = Gtk.Button(label="⬡  GRID VIEW")
        self.back_btn.set_name("back-btn")
        self.back_btn.set_valign(Gtk.Align.CENTER)
        self.back_btn.set_no_show_all(True)
        self.back_btn.connect("clicked", lambda _: self._restore_grid())
        hdr.pack_start(self.back_btn, False, False, 0)

        root.pack_start(hdr, False, False, 0)

        # Stack
        self.stack = Gtk.Stack()
        self.stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self.stack.set_transition_duration(200)
        root.pack_start(self.stack, True, True, 0)

        # Grid page — grid fills all available space, no scroll
        self.grid_outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.grid = Gtk.Grid()
        self.grid.set_row_spacing(3)
        self.grid.set_column_spacing(3)
        self.grid.set_margin_top(3); self.grid.set_margin_bottom(3)
        self.grid.set_margin_start(3); self.grid.set_margin_end(3)
        self.grid.set_column_homogeneous(True)
        self.grid.set_row_homogeneous(True)
        self.grid_outer.pack_start(self.grid, True, True, 0)
        self.stack.add_named(self.grid_outer, "grid")

        # Maximized page
        self.max_box = Gtk.Box()
        self.max_box.set_hexpand(True)
        self.max_box.set_vexpand(True)
        self.stack.add_named(self.max_box, "max")

        # Status bar
        sb = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        sb.set_name("statusbar")
        sb.set_size_request(-1, 22)
        self.status_lbl = Gtk.Label(label="INITIALIZING...")
        self.status_lbl.set_name("status-lbl")
        self.status_lbl.set_halign(Gtk.Align.START)
        self.clock_lbl = Gtk.Label(label="")
        self.clock_lbl.set_name("clock-lbl")
        self.clock_lbl.set_halign(Gtk.Align.END)
        sb.pack_start(self.status_lbl, True,  True,  0)
        sb.pack_end  (self.clock_lbl,  False, False, 0)
        root.pack_end(sb, False, False, 0)

        self.stack.set_visible_child_name("grid")

    # ── Streams ───────────────────────────────────────────────────────────────
    def _init_streams(self):
        decoder = detect_decoder()
        if not decoder:
            self.status_lbl.set_text("ERROR: No H.264 decoder found! Install gstreamer1.0-libav")
            return

        # Calculate how many columns to use (max COLS, but fewer if fewer cameras)
        n = len(self.ports)
        cols = min(self.COLS, n)
        rows = math.ceil(n / cols) if cols > 0 else 1

        for idx, port in enumerate(self.ports):
            be = CameraBackend(idx, port, self.width, self.height, decoder, self.host)
            if not be.build():
                continue

            tile = CameraTile(be)
            # Minimum size so tiny windows are still usable; actual size is dynamic
            tile.set_size_request(160, 120)
            # Expand to fill all available space in the grid cell
            tile.set_hexpand(True)
            tile.set_vexpand(True)
            tile.set_halign(Gtk.Align.FILL)
            tile.set_valign(Gtk.Align.FILL)
            tile.connect("button-press-event",
                         lambda w, e, i=idx: self._on_tile_click(i))

            col = idx % self.COLS
            row = idx // self.COLS
            self.grid.attach(tile, col, row, 1, 1)

            self.backends.append(be)
            self.tiles.append(tile)
            be.start()

        self.grid.show_all()
        self.status_lbl.set_text(
            f"SYSTEM ONLINE  ─  {len(self.backends)} PIPELINE(S) ACTIVE  ─  "
            f"Click any feed to maximize  ─  ESC to return"
        )

    # ── Maximize / Restore ───────────────────────────────────────────────────
    def _on_tile_click(self, camera_id):
        if self.max_id == camera_id:
            self._restore_grid()
        else:
            self._maximize(camera_id)

    def _maximize(self, camera_id):
        self.max_id = camera_id

        # Highlight grid tile
        for t in self.tiles:
            t.maximized = (t.backend.camera_id == camera_id)
            t.queue_draw()

        # Remove old max tile
        for child in self.max_box.get_children():
            self.max_box.remove(child)

        # Create big tile — expand to fill entire area
        be = self.backends[camera_id]
        big = CameraTile(be, is_large=True)
        big.maximized = True
        big.set_hexpand(True)
        big.set_vexpand(True)
        big.set_halign(Gtk.Align.FILL)
        big.set_valign(Gtk.Align.FILL)
        be.on_new_frame    = lambda cid: big.queue_draw()
        be.on_stats_update = lambda cid: big.queue_draw()
        self.max_tile = big
        self.max_box.pack_start(big, True, True, 0)
        big.show()

        self.stack.set_visible_child_name("max")
        self.back_btn.show()
        self.status_lbl.set_text(
            f"MAXIMIZED  ─  CAM {camera_id:02d}  ─  PORT {self.ports[camera_id]}"
            f"   [ESC or GRID VIEW to return]"
        )

    def _restore_grid(self):
        self.max_id   = None
        self.max_tile = None

        for be in self.backends:
            # Find matching grid tile and re-hook callbacks
            for t in self.tiles:
                if t.backend is be:
                    be.on_new_frame    = lambda cid, ti=t: ti.queue_draw()
                    be.on_stats_update = lambda cid, ti=t: ti.queue_draw()
                    break

        for t in self.tiles:
            t.maximized = False
            t.queue_draw()

        for child in self.max_box.get_children():
            self.max_box.remove(child)

        self.stack.set_visible_child_name("grid")
        self.back_btn.hide()
        self.status_lbl.set_text(
            f"GRID VIEW  ─  {len(self.backends)} PIPELINE(S) ACTIVE"
        )

    # ── Tick ─────────────────────────────────────────────────────────────────
    def _tick(self):
        active = sum(1 for b in self.backends if b.status == "streaming")
        self.active_badge.set_text(f"{active} / {len(self.ports)} ACTIVE")
        self.clock_lbl.set_text(datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        return True

    # ── Key ──────────────────────────────────────────────────────────────────
    def _on_key(self, w, e):
        if e.keyval == Gdk.KEY_Escape and self.max_id is not None:
            self._restore_grid()

    # ── Destroy ──────────────────────────────────────────────────────────────
    def _on_destroy(self, w):
        for be in self.backends:
            be.stop()
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
        description="SENTINEL — Multi-Camera GUI Receiver (Jetson)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-p", "--ports", nargs="+", type=int, default=[5000],
                        metavar="PORT",
                        help="UDP port(s). E.g: -p 5000 5001 5002 5003")
    parser.add_argument("-w", "--width",  type=int, default=640,
                        help="Decode width per feed")
    parser.add_argument("-H", "--height", type=int, default=480,
                        help="Decode height per feed")
    parser.add_argument("-i", "--host",   type=str, default="127.0.0.1",
                        help="Transmitter IP (used for ping measurement)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    app = ReceiverApp(
        ports=args.ports,
        width=args.width,
        height=args.height,
        host=args.host,
    )
    Gtk.main()


if __name__ == "__main__":
    main()
