#!/usr/bin/env python3
"""
GStreamer Multi-Camera Video Transmitter for Raspberry Pi 4B
Streams multiple USB camera feeds with H.264 encoding.
Each camera streams on its own UDP port.

Key features:
  - Native format detection: probes whether each camera outputs MJPEG or raw
    (YUYV/YUY2) and builds the correct pipeline automatically.
    MJPEG cameras get:  v4l2src ! image/jpeg ! jpegdec ! videoconvert ! encoder
    Raw cameras get:    v4l2src ! video/x-raw ! videoconvert ! encoder
  - io-mode probe: tests mmap/dmabuf/userptr per camera and uses whatever works.
  - Resolution probe: uses v4l2-ctl to find what the camera actually supports.
  - Disconnection monitor: stops stream on unplug, restarts on reconnect.
  - Resolution fallback: steps down if buffer-pool errors persist.
  - Staggered pipeline starts.
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import argparse
import sys
import signal
import threading
import os
import time
import subprocess
import re


# ══════════════════════════════════════════════════════════════════════════════
#  Encoder detection
# ══════════════════════════════════════════════════════════════════════════════

ENCODER_CANDIDATES = [
    {
        "name":    "v4l2h264enc",
        "element": "v4l2h264enc",
        "pipeline": 'v4l2h264enc extra-controls="controls,repeat_sequence_header=1" ! video/x-h264,profile=baseline',
        "note":    "RPi4 HW (v4l2h264enc)"
    },
    {
        "name":    "nvh264enc",
        "element": "nvh264enc",
        "pipeline": "nvh264enc preset=low-latency-hq rc-mode=cbr bitrate=2000 ! video/x-h264,profile=baseline",
        "note":    "NVIDIA HW (nvh264enc)"
    },
    {
        "name":    "x264enc",
        "element": "x264enc",
        "pipeline": "x264enc tune=zerolatency speed-preset=ultrafast ! video/x-h264,profile=baseline",
        "note":    "Software (x264enc)"
    },
    {
        "name":    "openh264enc",
        "element": "openh264enc",
        "pipeline": "openh264enc ! video/x-h264,profile=baseline",
        "note":    "Software (openh264enc)"
    },
]

# Resolution fallback ladder
FALLBACK_PROFILES = [
    (640, 480, 30),
    (640, 480, 15),
    (320, 240, 30),
    (320, 240, 15),
]

# io-mode: 0=mmap(auto), 4=dmabuf-import, 2=userptr
IO_MODE_CANDIDATES = [
    (0, "mmap/auto"),
    (4, "dmabuf-import"),
    (2, "userptr"),
]


def detect_encoder():
    for c in ENCODER_CANDIDATES:
        if Gst.ElementFactory.find(c["element"]):
            print(f"[encoder] Using: {c['note']}")
            return c
    return None


def print_encoder_hints():
    print("\n--- Encoder install hints ---")
    print("x264enc:     sudo apt install gstreamer1.0-plugins-ugly")
    print("openh264enc: sudo apt install gstreamer1.0-plugins-bad")
    print("v4l2h264enc: built-in on RPi4 (update firmware if missing)")


# ══════════════════════════════════════════════════════════════════════════════
#  Camera format + capability probe
# ══════════════════════════════════════════════════════════════════════════════

def probe_native_format(device):
    """
    Query v4l2-ctl to find the camera's native pixel format.
    Returns one of: 'MJPG', 'YUYV', 'YUY2', 'NV12', or None if unknown.
    """
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", device, "--list-formats"],
            capture_output=True, text=True, timeout=5
        )
        output = result.stdout + result.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None

    # Priority: prefer MJPG if available (lower USB bandwidth), else first raw format
    formats_found = re.findall(r"'([A-Z0-9]{2,4})'", output)
    print(f"[probe] {device}  native formats: {formats_found}")

    if "MJPG" in formats_found:
        return "MJPG"
    for fmt in ("YUYV", "YUY2", "NV12", "BGR3", "RGB3"):
        if fmt in formats_found:
            return fmt
    if formats_found:
        return formats_found[0]
    return None


def probe_v4l2_caps(device):
    """
    Use v4l2-ctl to query supported resolutions and fps values.
    Returns list of (width, height, fps) sorted largest first, or None.
    """
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", device, "--list-formats-ext"],
            capture_output=True, text=True, timeout=5
        )
        output = result.stdout + result.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None

    caps = []
    current_size = None
    for line in output.splitlines():
        m = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if m:
            current_size = (int(m.group(1)), int(m.group(2)))
            continue
        m = re.search(r"Interval:.*\((\d+(?:\.\d+)?) fps\)", line)
        if m and current_size:
            fps = int(float(m.group(1)))
            if fps > 0:
                caps.append((current_size[0], current_size[1], fps))

    if not caps:
        return None

    seen, unique = set(), []
    for c in caps:
        if c not in seen:
            seen.add(c)
            unique.append(c)
    unique.sort(key=lambda x: (x[0] * x[1], x[2]), reverse=True)
    return unique


def probe_best_io_mode(device, native_format):
    """
    Test each io-mode with a 5-frame pipeline using the camera's actual
    native format so the probe doesn't fail on negotiation.
    Returns working io-mode int or 0.
    """
    # Build format-appropriate caps for the probe
    if native_format == "MJPG":
        src_caps = "image/jpeg"
        sink_part = "! jpegdec ! fakesink"
    else:
        fmt = native_format if native_format else "YUY2"
        src_caps = f"video/x-raw,format={fmt}"
        sink_part = "! fakesink"

    for io_mode, label in IO_MODE_CANDIDATES:
        test_pipe = (
            f"v4l2src device={device} io-mode={io_mode} num-buffers=5 ! "
            f"{src_caps} {sink_part}"
        )
        try:
            result = subprocess.run(
                ["gst-launch-1.0", "-q", test_pipe],
                capture_output=True, text=True, timeout=8
            )
            combined = (result.stdout + result.stderr).lower()
            bad = any(k in combined for k in [
                "error", "failed", "not-negotiated", "buffer pool"
            ])
            if result.returncode == 0 and not bad:
                print(f"[probe] {device}  io-mode={io_mode} ({label})  ✓")
                return io_mode
            else:
                print(f"[probe] {device}  io-mode={io_mode} ({label})  ✗")
        except subprocess.TimeoutExpired:
            print(f"[probe] {device}  io-mode={io_mode} ({label})  ✗ (timeout)")
        except FileNotFoundError:
            print(f"[probe] gst-launch-1.0 not found — defaulting io-mode=0")
            return 0

    print(f"[probe] {device}  all io-modes failed — defaulting mmap(0)")
    return 0


def probe_camera(device, req_w, req_h, req_fps):
    """
    Full probe: native format → io-mode → best resolution.
    Returns (io_mode, native_format, width, height, fps).
    """
    print(f"[probe] Probing {device}...")

    native_fmt = probe_native_format(device)
    if native_fmt:
        print(f"[probe] {device}  selected format: {native_fmt}")
    else:
        native_fmt = "YUYV"
        print(f"[probe] {device}  format unknown — assuming {native_fmt}")

    io_mode = probe_best_io_mode(device, native_fmt)

    caps = probe_v4l2_caps(device)
    if caps:
        exact = (req_w, req_h, req_fps)
        if exact in caps:
            w, h, fps = exact
            print(f"[probe] {device}  resolution: {w}x{h}@{fps}fps (exact match)")
        else:
            candidates = [c for c in caps if c[0] <= req_w and c[1] <= req_h]
            if candidates:
                w, h, fps = candidates[0]
                print(f"[probe] {device}  resolution: {w}x{h}@{fps}fps "
                      f"(nearest to {req_w}x{req_h}@{req_fps})")
            else:
                w, h, fps = caps[0]
                print(f"[probe] {device}  resolution: {w}x{h}@{fps}fps (camera max)")
    else:
        w, h, fps = req_w, req_h, req_fps
        print(f"[probe] {device}  resolution: {w}x{h}@{fps}fps (v4l2-ctl unavailable)")

    return io_mode, native_fmt, w, h, fps


# ══════════════════════════════════════════════════════════════════════════════
#  USB topology
# ══════════════════════════════════════════════════════════════════════════════

def get_usb_controller_for_device(device):
    try:
        dev_name = os.path.basename(os.path.realpath(device))
        sysfs    = f"/sys/class/video4linux/{dev_name}/device"
        if not os.path.exists(sysfs):
            return None
        real  = os.path.realpath(sysfs)
        parts = real.split("/")
        for i in range(len(parts), 0, -1):
            cand = "/".join(parts[:i])
            if os.path.exists(os.path.join(cand, "idVendor")):
                return parts[i - 1]
    except Exception:
        pass
    return None


def report_usb_topology(devices):
    print("\n[usb] USB topology report:")
    controllers = {}
    for dev in devices:
        key   = get_usb_controller_for_device(dev)
        label = key or "unknown"
        controllers.setdefault(label, []).append(dev)
        print(f"  {dev}  →  controller: {label}")
    shared = {k: v for k, v in controllers.items() if len(v) > 1}
    if shared:
        print("\n[usb] ⚠  WARNING: cameras share a controller — bandwidth limited.")
        for ctrl, devs in shared.items():
            print(f"       {ctrl}: {devs}")
    else:
        print("[usb] ✓  All cameras on separate USB controllers.")
    print()


# ══════════════════════════════════════════════════════════════════════════════
#  Error helpers
# ══════════════════════════════════════════════════════════════════════════════

def is_buffer_pool_error(debug_str):
    keywords = [
        "buffer pool activation failed",
        "failed to allocate required memory",
        "gst_v4l2src_decide_allocation",
        "cannot allocate",
        "enomem",
    ]
    return any(kw in (debug_str or "").lower() for kw in keywords)


def is_negotiation_error(debug_str):
    """
    True only for genuine caps negotiation failures at pipeline startup.
    'streaming stopped, reason not-negotiated' from gst_base_src_loop is a
    mid-stream V4L2 buffer starvation (EAGAIN/ENOBUFS), NOT a caps problem.
    We exclude that source location so it falls through to plain restart logic.
    """
    d = (debug_str or "").lower()
    # Mid-stream buffer starvation — looks like negotiation but isn't
    if "gst_base_src_loop" in d or "gstbasesrc.c" in d:
        return False
    if "gstv4l2bufferpool" in d or "poll error" in d:
        return False
    keywords = ["not-negotiated", "not negotiated", "no common caps",
                "could not negotiate"]
    return any(kw in d for kw in keywords)


# ══════════════════════════════════════════════════════════════════════════════
#  CameraStream
# ══════════════════════════════════════════════════════════════════════════════

class CameraStream:
    """
    Manages one camera's GStreamer pipeline.
    Builds format-correct pipeline based on native_format:
      MJPG  → image/jpeg caps + jpegdec
      other → video/x-raw,format=... caps
    """

    MAX_ERRORS = 5

    def __init__(self, camera_id, device, width, height, fps,
                 bitrate, host, port, encoder, io_mode=0,
                 native_format="YUYV", max_w=None, max_h=None):
        self.camera_id     = camera_id
        self.device        = device
        self.host          = host
        self.port          = port
        self.encoder       = encoder
        self.bitrate       = bitrate
        self.io_mode       = io_mode
        self.native_format = native_format

        self.width         = width
        self.height        = height
        self.fps           = fps
        self._profile_idx  = 0
        # Cap fallback profiles at user-requested resolution, not probed resolution
        self._max_w        = max_w if max_w is not None else width
        self._max_h        = max_h if max_h is not None else height
        self._profiles     = self._build_profiles(width, height, fps)

        self.pipeline           = None
        self.running            = False
        self.status             = "idle"
        self._error_count       = 0
        self._usb_errors        = 0
        self._last_error_was_bp = False
        self._gave_up_logged    = False
        self._lock              = threading.Lock()

    def _build_profiles(self, w, h, fps):
        """
        Build resolution fallback ladder.
        The first entry is always the probed/negotiated starting resolution.
        All fallback entries are strictly <= the USER-REQUESTED resolution
        (stored as self._max_w / self._max_h), so a camera probed at 1280x720
        but requested at 320x240 will never fall back to 640x480.
        """
        ladder = [(w, h, fps)]
        cap_w  = getattr(self, "_max_w", w)
        cap_h  = getattr(self, "_max_h", h)
        for p in FALLBACK_PROFILES:
            if p != (w, h, fps) and p[0] <= cap_w and p[1] <= cap_h:
                ladder.append(p)
        seen, unique = set(), []
        for p in ladder:
            if p not in seen:
                seen.add(p)
                unique.append(p)
        return unique

    def try_lower_profile(self):
        nxt = self._profile_idx + 1
        if nxt >= len(self._profiles):
            return False
        self._profile_idx = nxt
        self.width, self.height, self.fps = self._profiles[nxt]
        print(f"[cam{self.camera_id}] ↓  Profile {nxt}: "
              f"{self.width}x{self.height}@{self.fps}fps")
        return True

    # ── Pipeline builder ──────────────────────────────────────────────────────

    def _source_segment(self):
        """
        Return the v4l2src + caps string appropriate for this camera's format.
        MJPG cameras need image/jpeg caps (not video/x-raw).
        Raw cameras need video/x-raw with explicit format.
        """
        if self.native_format == "MJPG":
            # MJPEG: negotiate as compressed JPEG, then decode to raw
            return (
                f"v4l2src device={self.device} io-mode={self.io_mode} ! "
                f"image/jpeg,width={self.width},height={self.height},"
                f"framerate={self.fps}/1 ! "
                f"jpegdec ! "
                f"videoconvert"
            )
        else:
            # Raw format (YUYV, YUY2, NV12, etc.)
            fmt = self.native_format if self.native_format else "YUY2"
            return (
                f"v4l2src device={self.device} io-mode={self.io_mode} ! "
                f"video/x-raw,format={fmt},"
                f"width={self.width},height={self.height},"
                f"framerate={self.fps}/1 ! "
                f"videoconvert"
            )

    def build(self):
        src   = self._source_segment()
        enc   = self.encoder['pipeline']
        tail  = (
            f"h264parse ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.host} port={self.port} sync=false"
        )
        pipeline_str = f"{src} ! {enc} ! {tail}"
        print(f"[cam{self.camera_id}] Pipeline "
              f"(fmt={self.native_format} io={self.io_mode} "
              f"{self.width}x{self.height}@{self.fps}):\n"
              f"  {pipeline_str}")
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            print(f"[cam{self.camera_id}] Parse error: {e}")
            return False

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_message)
        return True

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        with self._lock:
            if self.pipeline is None:
                return False
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                print(f"[cam{self.camera_id}] PLAYING failed")
                self.status = "error"
                return False
            self.running            = True
            self.status             = "streaming"
            self._error_count       = 0
            self._last_error_was_bp = False
        print(f"[cam{self.camera_id}] ✓ Streaming {self.device} → "
              f"{self.host}:{self.port}  "
              f"[{self.native_format} io={self.io_mode} "
              f"{self.width}x{self.height}@{self.fps}fps]")
        return True

    def stop(self):
        with self._lock:
            self._do_stop()
            self.status = "stopped"
        print(f"[cam{self.camera_id}] Stopped")

    def _do_stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.running = False

    def is_device_present(self):
        return os.path.exists(self.device)

    # ── Bus messages ──────────────────────────────────────────────────────────

    def _on_message(self, bus, message):
        t = message.type

        if t == Gst.MessageType.EOS:
            print(f"[cam{self.camera_id}] EOS")
            with self._lock:
                self._do_stop()
                self.status = "error"

        elif t == Gst.MessageType.ERROR:
            err, debug    = message.parse_error()
            debug_str     = debug or ""
            bp_err        = is_buffer_pool_error(debug_str)
            neg_err       = is_negotiation_error(debug_str)

            print(f"[cam{self.camera_id}] Error: {err}")
            if bp_err:
                print(f"[cam{self.camera_id}] ↳ Buffer pool failure "
                      f"(io={self.io_mode}, fmt={self.native_format})")
            elif neg_err:
                print(f"[cam{self.camera_id}] ↳ Format negotiation failure "
                      f"(fmt={self.native_format}) — will try lower profile")
            else:
                print(f"[cam{self.camera_id}] Debug: {debug_str[:300]}")

            with self._lock:
                self._error_count      += 1
                # Treat negotiation errors same as buffer pool — lower profile
                self._last_error_was_bp = bp_err or neg_err
                if bp_err or neg_err:
                    self._usb_errors += 1
                self._do_stop()
                self.status = "error"

        elif t == Gst.MessageType.WARNING:
            warn, _ = message.parse_warning()
            print(f"[cam{self.camera_id}] Warning: {warn}")

        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old, new, _ = message.parse_state_changed()
                print(f"[cam{self.camera_id}] State: "
                      f"{old.value_nick} → {new.value_nick}")
                if new == Gst.State.PLAYING:
                    with self._lock:
                        self.status = "streaming"


# ══════════════════════════════════════════════════════════════════════════════
#  MultiCameraTransmitter
# ══════════════════════════════════════════════════════════════════════════════

class MultiCameraTransmitter:

    MONITOR_INTERVAL = 2
    RESTART_DELAY    = 3
    STAGGER_DELAY    = 1.5

    def __init__(self, cameras, host, base_port, width, height, fps, bitrate):
        Gst.init(None)
        self.cameras   = cameras
        self.host      = host
        self.base_port = base_port
        self.width     = width
        self.height    = height
        self.fps       = fps
        self.bitrate   = bitrate
        self.encoder   = None
        self.streams   = []
        self.loop      = None
        self._running  = False

    def setup(self):
        self.encoder = detect_encoder()
        if not self.encoder:
            print("ERROR: No H.264 encoder found!")
            print_encoder_hints()
            sys.exit(1)

        report_usb_topology(self.cameras)
        print(f"Probing {len(self.cameras)} camera(s)...\n")

        for idx, device in enumerate(self.cameras):
            port = self.base_port + idx

            if not os.path.exists(device):
                print(f"[cam{idx}] {device} not present — monitoring for reconnect\n")
                stream = CameraStream(
                    idx, device, self.width, self.height, self.fps,
                    self.bitrate, self.host, port, self.encoder
                )
                stream.status = "error"
                self.streams.append(stream)
                continue

            io_mode, native_fmt, w, h, fps = probe_camera(
                device, self.width, self.height, self.fps
            )
            stream = CameraStream(
                idx, device, w, h, fps,
                self.bitrate, self.host, port, self.encoder,
                io_mode=io_mode, native_format=native_fmt,
                max_w=self.width, max_h=self.height   # cap fallbacks at user-requested size
            )
            self.streams.append(stream)

            if not (stream.build() and stream.start()):
                stream.status = "error"
            print()

            if idx < len(self.cameras) - 1:
                print(f"[cam{idx}] Settling {self.STAGGER_DELAY}s "
                      f"before next camera...\n")
                time.sleep(self.STAGGER_DELAY)

    def start(self):
        self.setup()
        self._running = True

        active = sum(1 for s in self.streams if s.status == "streaming")
        print(f"\n{'='*62}")
        print(f"  {active}/{len(self.streams)} camera(s) streaming")
        print(f"  Receiver : {self.host}")
        print(f"  Ports    : {[self.base_port + i for i in range(len(self.streams))]}")
        print(f"  Monitor  : every {self.MONITOR_INTERVAL}s")
        print(f"{'='*62}\n")

        threading.Thread(
            target=self._monitor_loop, daemon=True, name="cam-monitor"
        ).start()

        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.stop()

    def stop(self):
        print("Stopping all streams...")
        self._running = False
        for s in self.streams:
            if s.status not in ("stopped", "idle"):
                s.stop()
        if self.loop and self.loop.is_running():
            self.loop.quit()
        print("All streams stopped.")

    def _monitor_loop(self):
        print("[monitor] Started — watching for disconnects and errors")
        while self._running:
            time.sleep(self.MONITOR_INTERVAL)
            if not self._running:
                break

            # ── Phase 1: detect disconnects (fast, no delay) ──────────────────
            for s in self.streams:
                present = s.is_device_present()
                if s.status == "streaming" and not present:
                    print(f"[monitor] ⚠  CAM {s.camera_id} DISCONNECTED ({s.device})")
                    with s._lock:
                        s._do_stop()
                        s.status             = "error"
                        s._error_count       = 0
                        s._last_error_was_bp = False

            # ── Phase 2: collect cameras that need a restart ──────────────────
            pending = [
                s for s in self.streams
                if s.status == "error"
                and s._error_count < CameraStream.MAX_ERRORS
                and s.is_device_present()
            ]

            # ── Phase 3: restart one at a time with stagger ───────────────────
            for s in pending:
                if not self._running:
                    break
                self._restart_one(s)
                if len(pending) > 1:
                    # Give the USB subsystem time to settle between restarts
                    time.sleep(self.STAGGER_DELAY)

            # ── Phase 4: log give-ups ─────────────────────────────────────────
            for s in self.streams:
                if s.status == "error" and s._error_count >= CameraStream.MAX_ERRORS:
                    if not s._gave_up_logged:
                        print(f"[monitor] ✗  CAM {s.camera_id} hit max errors "
                              f"({CameraStream.MAX_ERRORS}). Re-plug to reset.")
                        s._gave_up_logged = True

        print("[monitor] Stopped")

    def _restart_one(self, s):
        """Handle restart logic for a single errored stream."""

        # Buffer pool / real caps error — try lower profile
        if s._last_error_was_bp:
            if not s.try_lower_profile():
                print(f"[monitor] ✗  CAM {s.camera_id} all profiles exhausted.")
                s._error_count = CameraStream.MAX_ERRORS
                return
            print(f"[monitor] ↓  CAM {s.camera_id} lower profile — "
                  f"restarting in {self.RESTART_DELAY}s...")
            time.sleep(self.RESTART_DELAY)
            if not self._running:
                return
            s._last_error_was_bp = False
            if s.build() and s.start():
                print(f"[monitor] ✓  CAM {s.camera_id} restarted at "
                      f"{s.width}x{s.height}@{s.fps}fps")
            else:
                print(f"[monitor] ✗  CAM {s.camera_id} restart failed "
                      f"(error #{s._error_count})")
            return

        # Plain transient error — straight restart
        print(f"[monitor] ↺  CAM {s.camera_id} restarting "
              f"(attempt {s._error_count}/{CameraStream.MAX_ERRORS}) "
              f"in {self.RESTART_DELAY}s...")
        time.sleep(self.RESTART_DELAY)
        if not self._running:
            return
        s._gave_up_logged = False
        if s.build() and s.start():
            print(f"[monitor] ✓  CAM {s.camera_id} reconnected")
        else:
            print(f"[monitor] ✗  CAM {s.camera_id} failed "
                  f"(error #{s._error_count})")


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def signal_handler(sig, frame):
    print("\nReceived signal, stopping...")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(
        description="GStreamer Multi-Camera Transmitter — RPi4",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("-c", "--cameras", nargs="+", default=["/dev/video0"],
                        metavar="DEVICE",
                        help="Camera devices: -c /dev/video0 /dev/video2 /dev/video4")
    parser.add_argument("-i", "--host", required=True,
                        help="Receiver IP address")
    parser.add_argument("-p", "--base-port", type=int, default=5000)
    parser.add_argument("-w", "--width",   type=int, default=640)
    parser.add_argument("-H", "--height",  type=int, default=480)
    parser.add_argument("-f", "--fps",     type=int, default=30)
    parser.add_argument("-b", "--bitrate", type=int, default=2000000)
    parser.add_argument("--stagger", type=float, default=1.5,
                        help="Seconds between pipeline starts")
    parser.add_argument("--skip-probe", action="store_true",
                        help="Skip probing, use mmap+YUYV for all cameras")

    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal_handler)

    MultiCameraTransmitter.STAGGER_DELAY = args.stagger

    if args.skip_probe:
        global probe_camera
        def probe_camera(device, w, h, fps):
            print(f"[probe] {device} — skipped, using io=0 YUYV")
            return 0, "YUYV", w, h, fps

    tx = MultiCameraTransmitter(
        cameras=args.cameras,
        host=args.host,
        base_port=args.base_port,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
    )
    tx.start()


if __name__ == "__main__":
    main()
