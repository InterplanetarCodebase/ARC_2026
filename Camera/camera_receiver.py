#!/usr/bin/env python3
"""
SENTINEL — GStreamer Multi-Camera GUI Receiver
Jetson-optimized. Auto-detects decoder.
Chrome-like tabbed interface with per-feed FPS / ping / port overlay.
Click any tile to maximize; ESC or button to return to grid.
Right-click any feed to add to tabs or pop out into separate windows.
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

        self._frame_listeners = []
        self._stats_listeners = []

    # ── Build pipeline ────────────────────────────────────────────────────────
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
            for _cb in list(self._stats_listeners):
                GLib.idle_add(_cb, self.camera_id)

        for _cb in list(self._frame_listeners):
            GLib.idle_add(_cb, self.camera_id)

        return Gst.FlowReturn.OK

    def get_frame(self):
        with self._frame_lock:
            return self._last_frame

    # ── Ping ─────────────────────────────────────────────────────────────────
    def _ping_loop(self):
        while self.running:
            self.ping_ms = ping_once(self.host, timeout=2)
            for _cb in list(self._stats_listeners):
                GLib.idle_add(_cb, self.camera_id)
            time.sleep(3)

    # ── Bus ──────────────────────────────────────────────────────────────────
    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[cam{self.camera_id}] ERR: {err} | {dbg}")
            self.status = "error"
            for _cb in list(self._stats_listeners):
                GLib.idle_add(_cb, self.camera_id)
        elif t == Gst.MessageType.EOS:
            self.status = "stopped"


import os as _os

_LOGO_SURFACE = None
_LOGO_PATH    = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "image.png")

def _load_logo():
    global _LOGO_SURFACE
    if _LOGO_SURFACE is None:
        try:
            _LOGO_SURFACE = cairo.ImageSurface.create_from_png(_LOGO_PATH)
        except Exception as e:
            print(f"[logo] Could not load {_LOGO_PATH}: {e}")
    return _LOGO_SURFACE


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

        self._frame_cb = lambda cid: self.queue_draw()
        self._stats_cb = lambda cid: self.queue_draw()
        backend._frame_listeners.append(self._frame_cb)
        backend._stats_listeners.append(self._stats_cb)
        self.connect("destroy", self._on_tile_destroy)

    def _on_tile_destroy(self, widget):
        if self._frame_cb in self.backend._frame_listeners:
            self.backend._frame_listeners.remove(self._frame_cb)
        if self._stats_cb in self.backend._stats_listeners:
            self.backend._stats_listeners.remove(self._stats_cb)

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

        # ── Interplanetar logo ───────────────────────────────────────────────
        cx, cy = W / 2, H / 2
        cr.save()

        # Load and draw image.png centred, scaled to ~40% of the tile's shorter side
        try:
            logo = _load_logo()
            if logo:
                lw = logo.get_width()
                lh = logo.get_height()
                max_logo = min(W, H) * 0.40
                scale = min(max_logo / lw, max_logo / lh)
                dx = cx - lw * scale / 2
                dy = (cy - min(W, H) * 0.06) - lh * scale / 2
                cr.translate(dx, dy)
                cr.scale(scale, scale)
                cr.set_source_surface(logo, 0, 0)
                cr.paint_with_alpha(0.88)
        except Exception:
            # Fallback: draw ◈ symbol if image fails
            sym_fs = max(20, min(W, H) * 0.22)
            cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
            cr.set_font_size(sym_fs)
            sym = "◈"
            se  = cr.text_extents(sym)
            cr.set_source_rgba(1.00, 0.53, 0.02, 0.88)
            cr.move_to(cx - se.width / 2 + se.x_bearing,
                       cy - min(W, H) * 0.06 - se.height / 2 - se.y_bearing)
            cr.show_text(sym)

        cr.restore()
        cr.save()

        # "WAITING FOR STREAM" caption
        cap_fs = max(6, min(W, H) * 0.036)
        cr.select_font_face("Courier New", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
        cr.set_font_size(cap_fs)
        cap = "WAITING FOR STREAM"
        ce  = cr.text_extents(cap)
        cr.set_source_rgba(1.00, 0.53, 0.02, 0.35)
        cr.move_to(cx - ce.width / 2 + ce.x_bearing, cy + min(W, H) * 0.32)
        cr.show_text(cap)

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
        bh  = max(18, H * 0.09)
        pad = 6
        fs  = max(7, bh * 0.42)

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
#  Tab Label  (Chrome-style: name + close button, double-click to rename)
# ══════════════════════════════════════════════════════════════════════════════

class TabLabel(Gtk.Box):
    """Chrome-like tab label with close button. Double-click to rename."""

    def __init__(self, name, closeable=True, on_close=None, on_rename=None):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
        self.tab_name    = name
        self._on_rename  = on_rename

        self.evbox = Gtk.EventBox()
        self.evbox.set_visible_window(False)
        self.label = Gtk.Label(label=name)
        self.label.get_style_context().add_class("tab-label")
        self.evbox.add(self.label)
        self.evbox.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self.evbox.connect("button-press-event", self._on_press)
        self.pack_start(self.evbox, True, True, 0)

        if closeable:
            btn = Gtk.Button(label="×")
            btn.get_style_context().add_class("tab-close")
            btn.set_relief(Gtk.ReliefStyle.NONE)
            btn.connect("clicked", lambda _: on_close() if on_close else None)
            self.pack_start(btn, False, False, 0)

        self.show_all()

    def _on_press(self, widget, event):
        if event.type == Gdk.EventType._2BUTTON_PRESS:
            self._start_rename()
            return True
        return False

    def _start_rename(self):
        dialog = Gtk.Dialog(
            title="Rename Tab",
            transient_for=self.get_toplevel(),
            flags=Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
        )
        dialog.add_buttons("Cancel", Gtk.ResponseType.CANCEL,
                           "Rename", Gtk.ResponseType.OK)
        dialog.set_default_response(Gtk.ResponseType.OK)

        entry = Gtk.Entry()
        entry.set_text(self.tab_name)
        entry.set_activates_default(True)
        content = dialog.get_content_area()
        content.set_margin_start(12)
        content.set_margin_end(12)
        content.set_margin_top(8)
        content.set_margin_bottom(8)
        content.pack_start(entry, True, True, 0)
        dialog.show_all()

        if dialog.run() == Gtk.ResponseType.OK:
            new_name = entry.get_text().strip()
            if new_name:
                self.tab_name = new_name
                self.label.set_text(new_name)
                if self._on_rename:
                    self._on_rename(new_name)
        dialog.destroy()


# ══════════════════════════════════════════════════════════════════════════════
#  Tab Page  (independent camera grid with per-tab maximize)
# ══════════════════════════════════════════════════════════════════════════════

class TabPage(Gtk.Box):
    """Tab page with its own camera grid and maximize support."""
    COLS = 4

    def __init__(self, app, name, camera_ids=None, is_main=False):
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        self.app        = app
        self.name       = name
        self.camera_ids = list(camera_ids) if camera_ids else []
        self.is_main    = is_main
        self.tiles      = {}
        self.max_id     = None
        self.max_tile   = None

        # Stack for grid / maximized views
        self.stack = Gtk.Stack()
        self.stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self.stack.set_transition_duration(200)
        self.pack_start(self.stack, True, True, 0)

        self.grid = Gtk.Grid()
        self.grid.set_row_spacing(3)
        self.grid.set_column_spacing(3)
        self.grid.set_margin_top(3)
        self.grid.set_margin_bottom(3)
        self.grid.set_margin_start(3)
        self.grid.set_margin_end(3)
        self.grid.set_column_homogeneous(True)
        self.grid.set_row_homogeneous(True)
        self.stack.add_named(self.grid, "grid")

        self.max_box = Gtk.Box()
        self.max_box.set_hexpand(True)
        self.max_box.set_vexpand(True)
        self.stack.add_named(self.max_box, "max")

        self.stack.set_visible_child_name("grid")
        self.rebuild_grid()

    # ── Grid ──────────────────────────────────────────────────────────────────
    def rebuild_grid(self):
        for child in self.grid.get_children():
            child.destroy()
        self.tiles.clear()

        n = len(self.camera_ids)
        if n == 0:
            lbl = Gtk.Label()
            lbl.set_markup(
                '<span font_family="Courier New" size="11000" '
                'foreground="#4a3010">Right-click a camera to add it here</span>'
            )
            lbl.set_justify(Gtk.Justification.CENTER)
            lbl.set_valign(Gtk.Align.CENTER)
            lbl.set_halign(Gtk.Align.CENTER)
            lbl.set_hexpand(True)
            lbl.set_vexpand(True)
            self.grid.attach(lbl, 0, 0, 1, 1)
            self.grid.show_all()
            return

        cols = min(self.COLS, n)
        for idx, cam_id in enumerate(self.camera_ids):
            if cam_id >= len(self.app.backends):
                continue
            be   = self.app.backends[cam_id]
            tile = CameraTile(be)
            tile.set_size_request(160, 120)
            tile.set_hexpand(True)
            tile.set_vexpand(True)
            tile.set_halign(Gtk.Align.FILL)
            tile.set_valign(Gtk.Align.FILL)
            tile.connect("button-press-event",
                         lambda w, e, i=cam_id: self._on_tile_click(w, e, i))

            col = idx % cols
            row = idx // cols
            self.grid.attach(tile, col, row, 1, 1)
            self.tiles[cam_id] = tile

        self.grid.show_all()

    def _on_tile_click(self, widget, event, camera_id):
        if event.button == 3:  # right-click → context menu
            self.app.show_context_menu(event, camera_id, self)
            return True
        elif event.button == 1:  # left-click → maximize / restore
            if self.max_id == camera_id:
                self.restore_grid()
            else:
                self.maximize(camera_id)
            return True
        return False

    # ── Maximize / Restore ────────────────────────────────────────────────────
    def maximize(self, camera_id):
        self.max_id = camera_id
        for cid, t in self.tiles.items():
            t.maximized = (cid == camera_id)
            t.queue_draw()

        for child in self.max_box.get_children():
            child.destroy()

        be  = self.app.backends[camera_id]
        big = CameraTile(be, is_large=True)
        big.maximized = True
        big.set_hexpand(True)
        big.set_vexpand(True)
        big.set_halign(Gtk.Align.FILL)
        big.set_valign(Gtk.Align.FILL)
        self.max_tile = big
        self.max_box.pack_start(big, True, True, 0)
        big.show()

        self.stack.set_visible_child_name("max")
        self.app.back_btn.show()
        self.app.status_lbl.set_text(
            f"MAXIMIZED  ─  CAM {camera_id:02d}  ─  PORT {self.app.ports[camera_id]}"
            f"   [ESC or GRID VIEW to return]"
        )

    def restore_grid(self):
        self.max_id   = None
        self.max_tile = None

        for child in self.max_box.get_children():
            child.destroy()

        for t in self.tiles.values():
            t.maximized = False
            t.queue_draw()

        self.stack.set_visible_child_name("grid")
        self.app.back_btn.hide()
        self.app.status_lbl.set_text(
            f"TAB: {self.name}  ─  {len(self.camera_ids)} FEED(S)"
        )

    # ── Add / Remove cameras ─────────────────────────────────────────────────
    def add_camera(self, camera_id):
        if camera_id not in self.camera_ids:
            self.camera_ids.append(camera_id)
            self.rebuild_grid()

    def remove_camera(self, camera_id):
        if camera_id in self.camera_ids:
            self.camera_ids.remove(camera_id)
            if self.max_id == camera_id:
                self.restore_grid()
            self.rebuild_grid()


# ══════════════════════════════════════════════════════════════════════════════
#  Pop-Out Window  (standalone floating camera window)
# ══════════════════════════════════════════════════════════════════════════════

class PopOutWindow(Gtk.Window):
    """Standalone floating window for a single camera feed."""

    def __init__(self, app, camera_id):
        be = app.backends[camera_id]
        super().__init__(title=f"CAM {camera_id:02d}  ─  :{be.port}")
        self.app       = app
        self.camera_id = camera_id

        self.set_default_size(640, 480)

        tile = CameraTile(be, is_large=True)
        tile.set_hexpand(True)
        tile.set_vexpand(True)
        self.add(tile)

        self.connect("destroy", self._on_destroy)
        self.show_all()

    def _on_destroy(self, widget):
        if self in self.app.popout_windows:
            self.app.popout_windows.remove(self)


# ══════════════════════════════════════════════════════════════════════════════
#  Main Application Window
# ══════════════════════════════════════════════════════════════════════════════

CSS = b"""
window          { background: #060a0f; }
#header         { background: #070d14; border-bottom: 1px solid #2a1800; }
#title          { font-family: "Courier New"; font-size: 16px; font-weight: bold;
                  color: #ff8800; letter-spacing: 4px; }
#subtitle       { font-family: "Courier New"; font-size: 9px; color: #ff8800;
                  letter-spacing: 3px; }
#active-badge   { font-family: "Courier New"; font-size: 11px; color: #ff8800;
                  background: #120800; border: 1px solid #3a1800;
                  border-radius: 4px; padding: 3px 10px; }
#back-btn       { font-family: "Courier New"; font-size: 11px; color: #ff8800;
                  background: transparent; border: 1px solid #3a1800;
                  border-radius: 4px; padding: 4px 14px; margin: 6px; }
#back-btn:hover { background: #1a0e00; border-color: #ff8800; }
#statusbar      { background: #040608; border-top: 1px solid #1a0e00; padding: 0 12px; }
#status-lbl     { font-family: "Courier New"; font-size: 10px; color: #ff8800; }
#clock-lbl      { font-family: "Courier New"; font-size: 10px; color: #ff8800; }
notebook        { background: #060a0f; }
notebook header { background: #070d14; }
notebook tab    { background: #0a0e14; border: 1px solid #1a0e00;
                  padding: 4px 6px; font-family: "Courier New"; font-size: 11px;
                  color: #8a6020; min-height: 0; }
notebook tab:checked { background: #121820; border-bottom: 2px solid #ff8800;
                       color: #ff8800; }
notebook tab:hover   { background: #151a22; color: #cc7700; }
.tab-label      { font-family: "Courier New"; font-size: 11px; }
.tab-close      { font-size: 14px; color: #5a3a10;
                  background: transparent; border: none; padding: 0 2px;
                  min-width: 16px; min-height: 16px; }
.tab-close:hover { color: #ff4400; }
.tab-add        { font-family: "Courier New"; font-size: 16px; color: #5a3a10;
                  background: transparent; border: 1px solid #1a0e00;
                  border-radius: 3px; padding: 2px 8px; margin: 2px 4px; }
.tab-add:hover  { color: #ff8800; border-color: #3a1800; background: #0e1218; }
menu            { background: #0c1018; border: 1px solid #2a1800; }
menuitem        { font-family: "Courier New"; font-size: 11px; color: #cc7700;
                  padding: 4px 12px; }
menuitem:hover  { background: #1a1200; }
"""


class ReceiverApp(Gtk.Window):
    COLS = 4

    def __init__(self, ports, width, height, host):
        super().__init__(title="INTERPLANETAR  ◈  Multi-Camera Receiver")
        self.ports    = ports
        self.width    = width
        self.height   = height
        self.host     = host

        self.backends       = []
        self.tab_pages      = []
        self.popout_windows = []

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
        self.back_btn.connect("clicked", lambda _: self._restore_current_grid())
        hdr.pack_start(self.back_btn, False, False, 0)

        root.pack_start(hdr, False, False, 0)

        # Notebook (Chrome-like tabs)
        self.notebook = Gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.connect("switch-page", self._on_tab_switch)
        root.pack_start(self.notebook, True, True, 0)

        # "+" button to add new tabs
        add_btn = Gtk.Button(label="+")
        add_btn.get_style_context().add_class("tab-add")
        add_btn.set_tooltip_text("New Tab")
        add_btn.connect("clicked", lambda _: self._create_new_tab_dialog())
        add_btn.show()
        self.notebook.set_action_widget(add_btn, Gtk.PackType.END)

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

    # ── Streams ───────────────────────────────────────────────────────────────
    def _init_streams(self):
        decoder = detect_decoder()
        if not decoder:
            self.status_lbl.set_text("ERROR: No H.264 decoder found! Install gstreamer1.0-libav")
            return

        all_ids = []
        for idx, port in enumerate(self.ports):
            be = CameraBackend(idx, port, self.width, self.height, decoder, self.host)
            if not be.build():
                continue
            self.backends.append(be)
            all_ids.append(idx)
            be.start()

        # Default "All Cameras" tab
        self._add_tab("All Cameras", all_ids, is_main=True, closeable=False)

        self.status_lbl.set_text(
            f"SYSTEM ONLINE  ─  {len(self.backends)} PIPELINE(S) ACTIVE  ─  "
            f"Right-click any feed to manage tabs"
        )

    # ── Tab management ────────────────────────────────────────────────────────
    def _add_tab(self, name, camera_ids=None, is_main=False, closeable=True):
        page = TabPage(self, name, camera_ids, is_main)
        self.tab_pages.append(page)

        tab_label = TabLabel(
            name,
            closeable=closeable,
            on_close=lambda p=page: self._close_tab(p),
            on_rename=lambda new_name, p=page: setattr(p, 'name', new_name),
        )

        idx = self.notebook.append_page(page, tab_label)
        page.show_all()
        self.notebook.set_tab_reorderable(page, True)
        self.notebook.set_current_page(idx)
        return page

    def _close_tab(self, page):
        if page.max_id is not None:
            page.restore_grid()
        idx = self.notebook.page_num(page)
        if idx >= 0:
            self.notebook.remove_page(idx)
        if page in self.tab_pages:
            self.tab_pages.remove(page)
        page.destroy()

    def _create_new_tab_dialog(self):
        dialog = Gtk.Dialog(
            title="New Tab",
            transient_for=self,
            flags=Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
        )
        dialog.add_buttons("Cancel", Gtk.ResponseType.CANCEL,
                           "Create", Gtk.ResponseType.OK)
        dialog.set_default_response(Gtk.ResponseType.OK)

        content = dialog.get_content_area()
        content.set_margin_start(12)
        content.set_margin_end(12)
        content.set_margin_top(8)
        content.set_margin_bottom(8)
        content.set_spacing(8)

        lbl = Gtk.Label(label="Tab Name:")
        lbl.set_halign(Gtk.Align.START)
        content.pack_start(lbl, False, False, 0)

        entry = Gtk.Entry()
        entry.set_text("New Tab")
        entry.set_activates_default(True)
        content.pack_start(entry, False, False, 0)

        dialog.show_all()

        if dialog.run() == Gtk.ResponseType.OK:
            name = entry.get_text().strip()
            if name:
                self._add_tab(name)
        dialog.destroy()

    # ── Context menu (right-click on camera tile) ─────────────────────────────
    def show_context_menu(self, event, camera_id, source_page):
        menu = Gtk.Menu()

        # Pop out into separate window
        item = Gtk.MenuItem(label=f"\u2b17  Pop Out CAM {camera_id:02d}")
        item.connect("activate", lambda _: self._popout_camera(camera_id))
        menu.append(item)

        menu.append(Gtk.SeparatorMenuItem())

        # Existing custom tabs
        for page in self.tab_pages:
            if page.is_main:
                continue
            if camera_id in page.camera_ids:
                item = Gtk.MenuItem(label=f"\u2715  Remove from \"{page.name}\"")
                item.connect("activate",
                             lambda _, p=page: p.remove_camera(camera_id))
                menu.append(item)
            else:
                item = Gtk.MenuItem(label=f"\uff0b  Add to \"{page.name}\"")
                item.connect("activate",
                             lambda _, p=page: p.add_camera(camera_id))
                menu.append(item)

        menu.append(Gtk.SeparatorMenuItem())

        # Add to new tab
        item = Gtk.MenuItem(label="\uff0b  Add to New Tab...")
        item.connect("activate", lambda _: self._add_to_new_tab(camera_id))
        menu.append(item)

        menu.show_all()
        menu.popup_at_pointer(event)

    def _add_to_new_tab(self, camera_id):
        dialog = Gtk.Dialog(
            title="New Tab",
            transient_for=self,
            flags=Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
        )
        dialog.add_buttons("Cancel", Gtk.ResponseType.CANCEL,
                           "Create", Gtk.ResponseType.OK)
        dialog.set_default_response(Gtk.ResponseType.OK)

        content = dialog.get_content_area()
        content.set_margin_start(12)
        content.set_margin_end(12)
        content.set_margin_top(8)
        content.set_margin_bottom(8)

        entry = Gtk.Entry()
        entry.set_text("New Tab")
        entry.set_activates_default(True)
        content.pack_start(entry, True, True, 8)

        dialog.show_all()

        if dialog.run() == Gtk.ResponseType.OK:
            name = entry.get_text().strip()
            if name:
                self._add_tab(name, [camera_id])
        dialog.destroy()

    def _popout_camera(self, camera_id):
        win = PopOutWindow(self, camera_id)
        self.popout_windows.append(win)

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _restore_current_grid(self):
        page = self._current_page()
        if page and hasattr(page, 'restore_grid'):
            page.restore_grid()

    def _current_page(self):
        idx = self.notebook.get_current_page()
        if idx >= 0:
            return self.notebook.get_nth_page(idx)
        return None

    def _on_tab_switch(self, notebook, page, page_num):
        self.back_btn.hide()
        if hasattr(page, 'max_id') and page.max_id is not None:
            self.back_btn.show()
        if hasattr(page, 'name'):
            n = len(page.camera_ids) if hasattr(page, 'camera_ids') else 0
            self.status_lbl.set_text(f"TAB: {page.name}  \u2500  {n} FEED(S)")

    # ── Tick ─────────────────────────────────────────────────────────────────
    def _tick(self):
        active = sum(1 for b in self.backends if b.status == "streaming")
        self.active_badge.set_text(f"{active} / {len(self.ports)} ACTIVE")
        self.clock_lbl.set_text(datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S"))
        return True

    # ── Key ──────────────────────────────────────────────────────────────────
    def _on_key(self, w, e):
        if e.keyval == Gdk.KEY_Escape:
            page = self._current_page()
            if page and hasattr(page, 'max_id') and page.max_id is not None:
                page.restore_grid()

    # ── Destroy ──────────────────────────────────────────────────────────────
    def _on_destroy(self, w):
        for win in list(self.popout_windows):
            win.destroy()
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
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="""
Example usage:
  Single camera:
    python3 camera_receiver.py -p 5000 -i 192.168.1.50
  
  Multiple cameras (4 feeds):
    python3 camera_receiver.py -p 5000 5001 5002 5003 -i 192.168.1.50
  
  Custom resolution:
    python3 camera_receiver.py -p 5000 5001 -i 192.168.1.50 -w 1280 -H 720
        """
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
