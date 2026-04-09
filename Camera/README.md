# Camera System

Multi-camera streaming and monitoring toolkit built around GStreamer.

Supports:
- USB camera transmit over RTP/UDP (H.264)
- Multi-feed GTK receiver GUI
- RTSP feed display (test receiver)
- One-way latency telemetry side-channel
- Runtime camera control from receiver to transmitter
- Snapshot + QR/ArUco scan utilities

## Directory Files

- `camera_transmitter.py`
	Main UDP transmitter.
	Probes camera format, io-mode, and supported resolutions.
	Streams each camera on a sequential UDP port.
	Sends per-frame latency telemetry to `port + 1000`.
	Accepts runtime control packets on UDP `--control-port`.

- `camera_receiver.py`
	Main UDP receiver GUI.
	Multi-tile layout with maximize, tabs, pop-out windows, per-feed FPS and latency.
	Has settings popover that sends runtime config updates to transmitter.

- `test_transmitter.py`
	Test/development variant of transmitter (same CLI and behavior pattern as `camera_transmitter.py`).

- `test_reciever.py`
	Test/development receiver variant.
	Adds RTSP feed support and RTSP latency telemetry port mapping.
	Note: filename intentionally uses `reciever` spelling in repository.

- `ip_camera.py`
	Simple OpenCV RTSP viewer for quick camera validation.

- `scan.py`
	Decodes QR and ArUco markers from an image path and shows result in a Tk window.

- `run_transmitter_camera.sh`
	Example launcher for transmitter with three USB cameras.

- `run_receiver_camera.sh`
	Example launcher for receiver with eight UDP ports.

- `image.png`
	GUI placeholder/logo image used by receiver tiles.

## Quick Start

Run from this directory:

```bash
cd ~/Documents/ARC_2026/Camera
```

### 1) Start Transmitter (USB cameras -> UDP)

Single camera:

```bash
python3 camera_transmitter.py -c /dev/video0 -i 192.168.1.50 -p 5000
```

Multiple cameras:

```bash
python3 camera_transmitter.py -c /dev/video0 /dev/video2 /dev/video4 -i 192.168.1.50 -p 5000
```

Low-bandwidth practical command:

```bash
python3 camera_transmitter.py -c /dev/video0 /dev/video2 /dev/video4 -i 192.168.1.50 -p 5000 -w 320 -H 240
```

Transmitter with explicit control port:

```bash
python3 camera_transmitter.py -c /dev/video0 /dev/video2 -i 192.168.1.50 -p 5000 --control-port 7000
```

### 2) Start Receiver GUI (UDP)

Single feed:

```bash
python3 camera_receiver.py -p 5000 -i 192.168.1.100
```

Multiple feeds:

```bash
python3 camera_receiver.py -p 5000 5001 5002 5003 5004 5005 5006 5007 -i 192.168.1.100 -w 1920 -H 1080
```

Receiver with explicit control port (must match transmitter):

```bash
python3 camera_receiver.py -p 5000 5001 5002 5003 -i 192.168.1.100 --control-port 7000
```

### 3) RTSP in GUI (test receiver)

Use RTSP feed in the GUI:

```bash
python3 test_reciever.py --rtsp rtsp://admin:Interplanetar123@192.168.1.141:554/live/0/SUB
```

Use RTSP feed plus latency telemetry side-channel port (same scheme as UDP telemetry):

```bash
python3 test_reciever.py --rtsp rtsp://admin:Interplanetar123@192.168.1.141:554/live/0/SUB --rtsp-latency-ports 9100
```

### 4) Validate RTSP Quickly (OpenCV)

```bash
python3 ip_camera.py
```

## Complete CLI Arguments

### camera_transmitter.py

```bash
python3 camera_transmitter.py [options]
```

Arguments:
- `-c`, `--cameras DEVICE [DEVICE ...]`
	Optional camera device list. Example: `/dev/video0 /dev/video2`.
- `-i`, `--host HOST`
	Receiver IP address. Required.
- `-p`, `--base-port PORT`
	Base UDP port for camera streams. Default: `5000`.
- `-w`, `--width WIDTH`
	Requested capture width. Default: `640`.
- `-H`, `--height HEIGHT`
	Requested capture height. Default: `480`.
- `-f`, `--fps FPS`
	Requested FPS. Default: `25`.
- `-b`, `--bitrate BPS`
	Encoder bitrate in bits/s. Default: `1000000`.
- `--stagger SECONDS`
	Delay between camera pipeline starts. Default: `1.5`.
- `--skip-probe`
	Skip camera probing and use fallback assumptions.
- `--control-port PORT`
	UDP control channel port for runtime settings. Default: `7000`.

Example with all common options:

```bash
python3 camera_transmitter.py \
	-c /dev/video0 /dev/video2 /dev/video4 \
	-i 192.168.1.50 \
	-p 5000 \
	-w 640 -H 480 -f 25 -b 1000000 \
	--stagger 1.5 \
	--control-port 7000
```

### camera_receiver.py

```bash
python3 camera_receiver.py [options]
```

Arguments:
- `-p`, `--ports PORT [PORT ...]`
	UDP port list to receive. Default: `5000..5007`.
- `-w`, `--width WIDTH`
	Decode/display width per feed. Default: `1920`.
- `-H`, `--height HEIGHT`
	Decode/display height per feed. Default: `1080`.
- `-i`, `--host HOST`
	Transmitter IP used by control and latency paths. Default: `127.0.0.1`.
- `--control-port PORT`
	UDP control port used to send runtime camera settings. Default: `7000`.

Example:

```bash
python3 camera_receiver.py \
	-p 5000 5001 5002 5003 \
	-i 192.168.1.100 \
	-w 1920 -H 1080 \
	--control-port 7000
```

### test_reciever.py (RTSP-enabled receiver)

```bash
python3 test_reciever.py [options]
```

Arguments:
- Includes all `camera_receiver.py` arguments above.
- `--rtsp URL [URL ...]`
	Optional RTSP feed list.
- `--rtsp-latency-ports PORT [PORT ...]`
	Optional telemetry UDP ports for RTSP feeds.

Example:

```bash
python3 test_reciever.py \
	-p 5000 5001 5002 \
	-i 127.0.0.1 \
	--control-port 7000 \
	--rtsp rtsp://admin:Interplanetar123@192.168.1.141:554/live/0/SUB \
	--rtsp-latency-ports 9100
```

Control-port rule:
- `--control-port` on receiver and transmitter must be the same value.

## Latency Telemetry Scheme

- Receiver listens on telemetry port = video port + `1000` for UDP feeds.
- For RTSP feeds in `test_reciever.py`, telemetry ports are passed explicitly via `--rtsp-latency-ports`.
- Telemetry packet format is JSON over UDP:

```json
{"camera_id": 0, "pts": 1234567890, "sender_ts_ns": 1712345678901234567}
```

- For RTSP, fallback payload without `pts` is also accepted:

```json
{"sender_ts_ns": 1712345678901234567}
```

## Utility Commands

Run bundled launchers:

```bash
bash run_transmitter_camera.sh
bash run_receiver_camera.sh
```

Decode QR/ArUco from a captured image:

```bash
python3 scan.py /absolute/path/to/image.jpg
```

## Notes

- `camera_receiver.py` is the stable UDP receiver.
- `test_reciever.py` is the RTSP-enabled experimental receiver.
- `test_transmitter.py` mirrors transmitter behavior for development/testing.
- If latency is not provided on the telemetry socket, GUI shows `---ms`.
- Accurate one-way latency requires sender and receiver clocks to be synchronized (NTP/PTP).
