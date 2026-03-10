#!/usr/bin/env python3
"""
PING ROVER — Rover-side heartbeat, command receiver & single-camera streamer.

Runs on the rover (RPi). Responsibilities:
  1. Listens for heartbeat pings from the base station and replies immediately
     so the base can measure round-trip latency.
  2. Listens for WASD drive commands from the base, acknowledges each with the
     command name and receive-latency.
  3. Streams a single camera feed to the base using the full camera_transmitter
     pipeline (encoder detection, format probing, io-mode probing, resolution
     fallback, disconnect/reconnect monitoring).
  4. Continuously estimates wireless signal strength (RSSI) via iwconfig/iw.
  5. On transmission failure (no heartbeat for TIMEOUT seconds), writes a
     crash report .txt with timestamps, last-known RSSI, ping history, and
     event log.

Protocol (UDP, same port for heartbeat + commands):
  Heartbeat request :  [0xAA 0xBB SEQ_H SEQ_L 0x01 TS8_bytes CRC8]  (14 bytes)
  Heartbeat reply   :  [0xAA 0xBB SEQ_H SEQ_L 0x02 TS8_bytes CRC8]  (14 bytes)
  Command           :  [0xAA 0xBB SEQ_H SEQ_L 0x10 CMD CRC8]        (7 bytes)
  Command ACK       :  [0xAA 0xBB SEQ_H SEQ_L 0x11 CMD TS8_bytes CRC8] (15 bytes)

CMD byte:  0x57='W'  0x41='A'  0x53='S'  0x44='D'
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import argparse
import sys
import signal
import socket
import struct
import threading
import os
import time
import subprocess
import re
import datetime


# ══════════════════════════════════════════════════════════════════════════════
#  Protocol constants
# ══════════════════════════════════════════════════════════════════════════════

SOF1 = 0xAA
SOF2 = 0xBB
MSG_HEARTBEAT_REQ  = 0x01
MSG_HEARTBEAT_RPL  = 0x02
MSG_COMMAND        = 0x10
MSG_COMMAND_ACK    = 0x11

CMD_W = 0x57
CMD_A = 0x41
CMD_S = 0x53
CMD_D = 0x44
CMD_NAMES = {CMD_W: "W", CMD_A: "A", CMD_S: "S", CMD_D: "D"}

HEARTBEAT_TIMEOUT = 10  # seconds without heartbeat → write crash report


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ══════════════════════════════════════════════════════════════════════════════
#  RSSI / signal strength
# ══════════════════════════════════════════════════════════════════════════════

def get_rssi(interface="wlan0"):
    """Read wireless RSSI in dBm via iw or iwconfig. Returns int or None."""
    # Try iw first (modern)
    try:
        result = subprocess.run(
            ["iw", "dev", interface, "link"],
            capture_output=True, text=True, timeout=3
        )
        m = re.search(r"signal:\s*(-?\d+)\s*dBm", result.stdout)
        if m:
            return int(m.group(1))
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    # Fallback to iwconfig
    try:
        result = subprocess.run(
            ["iwconfig", interface],
            capture_output=True, text=True, timeout=3
        )
        m = re.search(r"Signal level[=:]?\s*(-?\d+)\s*dBm", result.stdout)
        if m:
            return int(m.group(1))
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return None


def rssi_quality(rssi):
    """Human-readable signal quality from dBm."""
    if rssi is None:
        return "UNKNOWN"
    if rssi >= -50:
        return "EXCELLENT"
    if rssi >= -60:
        return "GOOD"
    if rssi >= -70:
        return "FAIR"
    if rssi >= -80:
        return "WEAK"
    return "CRITICAL"


# ══════════════════════════════════════════════════════════════════════════════
#  Camera streaming (reuses camera_transmitter.py engine)
# ══════════════════════════════════════════════════════════════════════════════

ENCODER_CANDIDATES = [
    {"name": "v4l2h264enc", "element": "v4l2h264enc",
     "pipeline": 'v4l2h264enc extra-controls="controls,repeat_sequence_header=1,'
                 'h264_i_frame_period={keyint},video_bitrate={bitrate}" '
                 '! video/x-h264,profile=baseline',
     "note": "RPi4 HW (v4l2h264enc)"},
    {"name": "nvh264enc", "element": "nvh264enc",
     "pipeline": "nvh264enc preset=low-latency-hq rc-mode=cbr bitrate={bitrate_kbps} "
                 "gop-size={keyint} ! video/x-h264,profile=baseline",
     "note": "NVIDIA HW (nvh264enc)"},
    {"name": "x264enc", "element": "x264enc",
     "pipeline": "x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
                 "key-int-max={keyint} ! video/x-h264,profile=baseline",
     "note": "Software (x264enc)"},
    {"name": "openh264enc", "element": "openh264enc",
     "pipeline": "openh264enc bitrate={bitrate} ! video/x-h264,profile=baseline",
     "note": "Software (openh264enc)"},
]

FALLBACK_PROFILES = [
    (640, 480, 30), (640, 480, 15),
    (320, 240, 30), (320, 240, 15),
]

IO_MODE_CANDIDATES = [(0, "mmap/auto"), (4, "dmabuf-import"), (2, "userptr")]


def detect_encoder():
    for c in ENCODER_CANDIDATES:
        if Gst.ElementFactory.find(c["element"]):
            print(f"[encoder] Using: {c['note']}")
            return c
    return None


def probe_native_format(device):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", device, "--list-formats"],
            capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    formats_found = re.findall(r"'([A-Z0-9]{2,4})'", output)
    print(f"[probe] {device}  native formats: {formats_found}")
    if "MJPG" in formats_found:
        return "MJPG"
    for fmt in ("YUYV", "YUY2", "NV12", "BGR3", "RGB3"):
        if fmt in formats_found:
            return fmt
    return formats_found[0] if formats_found else None


def probe_v4l2_caps(device):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", device, "--list-formats-ext"],
            capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    caps, current_size = [], None
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
    if native_format == "MJPG":
        src_caps, sink_part = "image/jpeg", "! jpegdec ! fakesink"
    else:
        fmt = native_format or "YUY2"
        src_caps, sink_part = f"video/x-raw,format={fmt}", "! fakesink"
    for io_mode, label in IO_MODE_CANDIDATES:
        test_pipe = (f"v4l2src device={device} io-mode={io_mode} num-buffers=5 ! "
                     f"{src_caps} {sink_part}")
        try:
            result = subprocess.run(
                ["gst-launch-1.0", "-q", test_pipe],
                capture_output=True, text=True, timeout=8)
            combined = (result.stdout + result.stderr).lower()
            bad = any(k in combined for k in ["error", "failed", "not-negotiated", "buffer pool"])
            if result.returncode == 0 and not bad:
                print(f"[probe] {device}  io-mode={io_mode} ({label})  OK")
                return io_mode
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
    return 0


def probe_camera(device, req_w, req_h, req_fps):
    print(f"[probe] Probing {device}...")
    native_fmt = probe_native_format(device)
    if not native_fmt:
        native_fmt = "YUYV"
        print(f"[probe] {device}  format unknown — assuming {native_fmt}")
    else:
        print(f"[probe] {device}  selected format: {native_fmt}")
    io_mode = probe_best_io_mode(device, native_fmt)
    caps = probe_v4l2_caps(device)
    if caps:
        exact = (req_w, req_h, req_fps)
        if exact in caps:
            w, h, fps = exact
        else:
            candidates = [c for c in caps if c[0] <= req_w and c[1] <= req_h]
            w, h, fps = candidates[0] if candidates else caps[0]
        print(f"[probe] {device}  resolution: {w}x{h}@{fps}fps")
    else:
        w, h, fps = req_w, req_h, req_fps
        print(f"[probe] {device}  resolution: {w}x{h}@{fps}fps (v4l2-ctl unavailable)")
    return io_mode, native_fmt, w, h, fps


def is_buffer_pool_error(debug_str):
    keywords = ["buffer pool activation failed", "failed to allocate required memory",
                "gst_v4l2src_decide_allocation", "cannot allocate", "enomem"]
    return any(kw in (debug_str or "").lower() for kw in keywords)


def is_negotiation_error(debug_str):
    d = (debug_str or "").lower()
    if "gst_base_src_loop" in d or "gstbasesrc.c" in d:
        return False
    if "gstv4l2bufferpool" in d or "poll error" in d:
        return False
    keywords = ["not-negotiated", "not negotiated", "no common caps", "could not negotiate"]
    return any(kw in d for kw in keywords)


class CameraStream:
    MAX_ERRORS = 5

    def __init__(self, device, width, height, fps, bitrate, host, port,
                 encoder, io_mode=0, native_format="YUYV"):
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
        self._max_w        = width
        self._max_h        = height
        self._profiles     = self._build_profiles(width, height, fps)
        self.pipeline      = None
        self.running       = False
        self.status        = "idle"
        self._error_count       = 0
        self._last_error_was_bp = False
        self._gave_up_logged    = False
        self._lock              = threading.Lock()

    def _build_profiles(self, w, h, fps):
        ladder = [(w, h, fps)]
        for p in FALLBACK_PROFILES:
            if p != (w, h, fps) and p[0] <= self._max_w and p[1] <= self._max_h:
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
        print(f"[cam] Profile {nxt}: {self.width}x{self.height}@{self.fps}fps")
        return True

    def _source_segment(self):
        if self.native_format == "MJPG":
            return (f"v4l2src device={self.device} io-mode={self.io_mode} ! "
                    f"image/jpeg,width={self.width},height={self.height},"
                    f"framerate={self.fps}/1 ! jpegdec ! videoconvert")
        fmt = self.native_format or "YUY2"
        return (f"v4l2src device={self.device} io-mode={self.io_mode} ! "
                f"video/x-raw,format={fmt},width={self.width},height={self.height},"
                f"framerate={self.fps}/1 ! videoconvert")

    def build(self):
        src    = self._source_segment()
        keyint = max(self.fps, 15)
        enc    = self.encoder['pipeline'].format(
            bitrate=self.bitrate, bitrate_kbps=self.bitrate // 1000, keyint=keyint)
        tail   = (f"h264parse ! rtph264pay config-interval=-1 pt=96 mtu=1200 ! "
                  f"udpsink host={self.host} port={self.port} sync=false")
        pipeline_str = f"{src} ! {enc} ! {tail}"
        print(f"[cam] Pipeline ({self.native_format} io={self.io_mode} "
              f"{self.width}x{self.height}@{self.fps}):\n  {pipeline_str}")
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            print(f"[cam] Parse error: {e}")
            return False
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_message)
        return True

    def start(self):
        with self._lock:
            if not self.pipeline:
                return False
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.status = "error"
                return False
            self.running       = True
            self.status        = "streaming"
            self._error_count  = 0
            self._last_error_was_bp = False
        print(f"[cam] Streaming {self.device} -> {self.host}:{self.port}")
        return True

    def stop(self):
        with self._lock:
            self._do_stop()
            self.status = "stopped"

    def _do_stop(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.running = False

    def is_device_present(self):
        return os.path.exists(self.device)

    def _on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            with self._lock:
                self._do_stop()
                self.status = "error"
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            debug_str  = debug or ""
            bp_err  = is_buffer_pool_error(debug_str)
            neg_err = is_negotiation_error(debug_str)
            print(f"[cam] Error: {err}")
            with self._lock:
                self._error_count += 1
                self._last_error_was_bp = bp_err or neg_err
                self._do_stop()
                self.status = "error"
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                _, new, _ = message.parse_state_changed()
                if new == Gst.State.PLAYING:
                    with self._lock:
                        self.status = "streaming"


# ══════════════════════════════════════════════════════════════════════════════
#  Ping Rover — main coordinator
# ══════════════════════════════════════════════════════════════════════════════

class PingRover:
    MONITOR_INTERVAL = 2
    RESTART_DELAY    = 3
    RSSI_INTERVAL    = 2

    def __init__(self, base_ip, ctrl_port, cam_port, camera_device,
                 width, height, fps, bitrate, wlan_iface):
        Gst.init(None)
        self.base_ip       = base_ip
        self.ctrl_port     = ctrl_port
        self.cam_port      = cam_port
        self.camera_device = camera_device
        self.width         = width
        self.height        = height
        self.fps           = fps
        self.bitrate       = bitrate
        self.wlan_iface    = wlan_iface

        self.stream        = None
        self.encoder       = None
        self._running      = False
        self.loop          = None

        # State
        self.rssi          = None
        self.rssi_quality  = "UNKNOWN"
        self.last_heartbeat_ts = 0.0
        self.heartbeat_count   = 0
        self.command_count     = 0
        self.event_log         = []   # list of (timestamp_str, message)
        self._ping_history     = []   # last 60 RTT values from base's perspective

    def _log(self, msg):
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.event_log.append((ts, msg))
        print(f"[{ts}] {msg}")

    # ── UDP listener ──────────────────────────────────────────────────────────
    def _ctrl_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.ctrl_port))
        sock.settimeout(1.0)
        self._log(f"Listening for commands on UDP :{self.ctrl_port}")

        while self._running:
            try:
                data, addr = sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < 7 or data[0] != SOF1 or data[1] != SOF2:
                continue
            if crc8(data[:-1]) != data[-1]:
                continue

            seq = (data[2] << 8) | data[3]
            msg_type = data[4]

            if msg_type == MSG_HEARTBEAT_REQ and len(data) == 14:
                # Echo the timestamp back
                ts_bytes = data[5:13]  # 8 bytes of base timestamp
                reply_body = bytes([SOF1, SOF2, (seq >> 8) & 0xFF, seq & 0xFF,
                                    MSG_HEARTBEAT_RPL]) + ts_bytes
                reply = reply_body + bytes([crc8(reply_body)])
                sock.sendto(reply, addr)
                self.last_heartbeat_ts = time.monotonic()
                self.heartbeat_count += 1

            elif msg_type == MSG_COMMAND and len(data) == 7:
                cmd = data[5]
                recv_ts = time.monotonic()
                cmd_name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
                self._log(f"CMD received: {cmd_name}")
                self.command_count += 1
                # Send ACK with rover timestamp
                ts_bytes = struct.pack('!d', recv_ts)
                ack_body = bytes([SOF1, SOF2, (seq >> 8) & 0xFF, seq & 0xFF,
                                  MSG_COMMAND_ACK, cmd]) + ts_bytes
                ack = ack_body + bytes([crc8(ack_body)])
                sock.sendto(ack, addr)

        sock.close()

    # ── RSSI monitor ──────────────────────────────────────────────────────────
    def _rssi_loop(self):
        while self._running:
            self.rssi = get_rssi(self.wlan_iface)
            self.rssi_quality = rssi_quality(self.rssi)
            time.sleep(self.RSSI_INTERVAL)

    # ── Heartbeat watchdog ────────────────────────────────────────────────────
    def _watchdog_loop(self):
        """If no heartbeat for HEARTBEAT_TIMEOUT seconds, write crash report."""
        reported = False
        while self._running:
            time.sleep(1)
            if self.last_heartbeat_ts == 0:
                continue  # haven't received first heartbeat yet
            elapsed = time.monotonic() - self.last_heartbeat_ts
            if elapsed > HEARTBEAT_TIMEOUT and not reported:
                self._log(f"HEARTBEAT LOST — no ping for {elapsed:.1f}s")
                self._write_crash_report()
                reported = True
            elif elapsed <= HEARTBEAT_TIMEOUT:
                reported = False

    # ── Camera monitor ────────────────────────────────────────────────────────
    def _camera_monitor(self):
        """Restart camera stream on errors, with profile fallback."""
        while self._running:
            time.sleep(self.MONITOR_INTERVAL)
            if not self._running or not self.stream:
                continue

            s = self.stream
            if s.status == "streaming" and not s.is_device_present():
                self._log("Camera DISCONNECTED")
                with s._lock:
                    s._do_stop()
                    s.status = "error"
                    s._error_count = 0
                    s._last_error_was_bp = False

            if (s.status == "error"
                    and s._error_count < CameraStream.MAX_ERRORS
                    and s.is_device_present()):
                self._restart_stream(s)

            if s.status == "error" and s._error_count >= CameraStream.MAX_ERRORS:
                if not s._gave_up_logged:
                    self._log(f"Camera max errors ({CameraStream.MAX_ERRORS}) — re-plug to reset")
                    s._gave_up_logged = True

    def _restart_stream(self, s):
        if s._last_error_was_bp:
            if not s.try_lower_profile():
                self._log("Camera: all profiles exhausted")
                s._error_count = CameraStream.MAX_ERRORS
                return
            self._log(f"Camera: lower profile — restarting in {self.RESTART_DELAY}s")
            time.sleep(self.RESTART_DELAY)
            if not self._running:
                return
            s._last_error_was_bp = False
            if s.build() and s.start():
                self._log(f"Camera restarted at {s.width}x{s.height}@{s.fps}fps")
            else:
                self._log("Camera restart failed")
            return

        self._log(f"Camera: restarting (attempt {s._error_count}/{CameraStream.MAX_ERRORS})")
        time.sleep(self.RESTART_DELAY)
        if not self._running:
            return
        s._gave_up_logged = False
        if s.build() and s.start():
            self._log("Camera reconnected")
        else:
            self._log("Camera restart failed")

    # ── Crash report ──────────────────────────────────────────────────────────
    def _write_crash_report(self):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(script_dir, f"crash_report_{ts}.txt")
        try:
            with open(path, "w") as f:
                f.write(f"PING ROVER — CRASH REPORT\n")
                f.write(f"Generated: {datetime.datetime.now().isoformat()}\n")
                f.write(f"{'='*60}\n\n")
                f.write(f"Base station IP  : {self.base_ip}\n")
                f.write(f"Control port     : {self.ctrl_port}\n")
                f.write(f"Camera port      : {self.cam_port}\n")
                f.write(f"Camera device    : {self.camera_device}\n")
                f.write(f"WLAN interface   : {self.wlan_iface}\n\n")
                f.write(f"Last RSSI        : {self.rssi} dBm\n")
                f.write(f"Signal quality   : {self.rssi_quality}\n")
                f.write(f"Heartbeats recv  : {self.heartbeat_count}\n")
                f.write(f"Commands recv    : {self.command_count}\n")
                cam_st = self.stream.status if self.stream else "N/A"
                f.write(f"Camera status    : {cam_st}\n\n")
                f.write(f"{'='*60}\n")
                f.write(f"EVENT LOG (last 200 entries):\n")
                f.write(f"{'='*60}\n")
                for ts_str, msg in self.event_log[-200:]:
                    f.write(f"  [{ts_str}]  {msg}\n")
            self._log(f"Crash report written: {path}")
        except OSError as e:
            self._log(f"Failed to write crash report: {e}")

    # ── Start / Stop ──────────────────────────────────────────────────────────
    def start(self):
        self._running = True
        self._log("PING ROVER starting")

        # Encoder
        self.encoder = detect_encoder()
        if not self.encoder:
            self._log("ERROR: No H.264 encoder found!")
            sys.exit(1)

        # Camera stream
        if os.path.exists(self.camera_device):
            io_mode, native_fmt, w, h, fps = probe_camera(
                self.camera_device, self.width, self.height, self.fps)
            self.stream = CameraStream(
                self.camera_device, w, h, fps, self.bitrate,
                self.base_ip, self.cam_port, self.encoder,
                io_mode=io_mode, native_format=native_fmt)
            if not (self.stream.build() and self.stream.start()):
                self.stream.status = "error"
                self._log("Camera initial start failed — will retry")
        else:
            self._log(f"{self.camera_device} not present — will monitor for reconnect")
            self.stream = CameraStream(
                self.camera_device, self.width, self.height, self.fps,
                self.bitrate, self.base_ip, self.cam_port, self.encoder)
            self.stream.status = "error"

        # Background threads
        for target in (self._ctrl_listener, self._rssi_loop,
                       self._watchdog_loop, self._camera_monitor):
            threading.Thread(target=target, daemon=True).start()

        # Status printer
        threading.Thread(target=self._status_printer, daemon=True).start()

        self._log("All subsystems running")
        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        self._log("Shutting down")
        self._running = False
        if self.stream and self.stream.status not in ("stopped", "idle"):
            self.stream.stop()
        if self.loop and self.loop.is_running():
            self.loop.quit()

    def _status_printer(self):
        while self._running:
            cam_st = self.stream.status if self.stream else "N/A"
            rssi_str = f"{self.rssi} dBm" if self.rssi is not None else "N/A"
            hb_age = (time.monotonic() - self.last_heartbeat_ts
                      if self.last_heartbeat_ts > 0 else -1)
            hb_str = f"{hb_age:.1f}s ago" if hb_age >= 0 else "waiting"
            print(f"\r[status] cam={cam_st}  rssi={rssi_str} ({self.rssi_quality})  "
                  f"hb={hb_str}  cmds={self.command_count}      ", end="", flush=True)
            time.sleep(2)


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(
        description="PING ROVER — Heartbeat, command receiver & camera streamer",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="""
Example:
  python3 ping_rover.py -i 192.168.1.100 -c /dev/video0
  python3 ping_rover.py -i 192.168.1.100 -c /dev/video0 -b 500000 --wlan wlan1
        """)
    parser.add_argument("-i", "--base-ip", required=True,
                        help="Base station IP address")
    parser.add_argument("--ctrl-port", type=int, default=6000,
                        help="UDP port for heartbeat / commands")
    parser.add_argument("--cam-port", type=int, default=5000,
                        help="UDP port for camera stream")
    parser.add_argument("-c", "--camera", default="/dev/video0",
                        help="Camera device")
    parser.add_argument("-w", "--width", type=int, default=640)
    parser.add_argument("-H", "--height", type=int, default=480)
    parser.add_argument("-f", "--fps", type=int, default=30)
    parser.add_argument("-b", "--bitrate", type=int, default=800000,
                        help="H.264 bitrate in bits/s")
    parser.add_argument("--wlan", default="wlan0",
                        help="Wireless interface for RSSI reading")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    rover = PingRover(
        base_ip=args.base_ip,
        ctrl_port=args.ctrl_port,
        cam_port=args.cam_port,
        camera_device=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bitrate=args.bitrate,
        wlan_iface=args.wlan,
    )
    rover.start()


if __name__ == "__main__":
    main()
