# Camera System — GStreamer Video Streaming

Multi-camera video transmission system using H.264 encoding over UDP.

## Files

- **`camera_transmitter.py`** — Runs on Xavier. Captures from USB cameras and streams H.264 video over UDP.
- **`camera_receiver.py`** — Runs on Base. Receives and displays multiple camera feeds in a GUI with FPS/ping monitoring.

## Quick Start

### On Jetson Xavier (Transmitter)

Single camera:
```bash
python3 camera_transmitter.py -c /dev/video0 -i 192.168.1.100 -p 5000
```

Multiple cameras:
```bash
python3 camera_transmitter.py -c /dev/video0 /dev/video2 /dev/video4 -i 192.168.1.100 -p 5000
```

Custom resolution:
```bash
python3 camera_transmitter.py -c /dev/video0 -i 192.168.1.100 -p 5000 -w 1280 -H 720 -f 30
```

Practical Command:
```bash
python3 camera_transmitter.py -c /dev/video0 /dev/video2 /dev/video4 -i 192.168.1.100 -p 5000 -w 320 -H 240
```

### On Base station (Receiver)

Single camera:
```bash
python3 camera_receiver.py -p 5000 -i 192.168.1.50
```

Multiple cameras (4 feeds):
```bash
python3 camera_receiver.py -p 5000 5001 5002 5003 -i 192.168.1.50
```

Practical Command:
```bash
python3 camera_receiver.py -p 5000 5001 5002 5003 5004 5005 5006 5007 -i 192.168.1.50 -w 1920 -H 1080
```

## Notes

- Transmitter auto-detects camera capabilities (MJPEG/raw, resolution, USB mode)
- Receiver displays feeds in 4×4 grid — click any tile to maximize, ESC to return
- Each camera uses sequential ports (base_port, base_port+1, base_port+2, ...)
- Transmitter monitors for disconnects and auto-restarts streams
