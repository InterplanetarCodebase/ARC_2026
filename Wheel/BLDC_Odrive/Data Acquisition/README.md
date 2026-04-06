# Data Acquisition

## Folder Structure

```
Data Acquisition/
├── bldc_data_acquisition_gui.py       # BLDC motor telemetry GUI
├── pose_data_acquisition_gui.py       # Wheel + ZED pose data GUI
├── bldc_*.jsonl                       # Recorded telemetry sessions
└── __pycache__/                       # Python cache
```

## File Tasks

| File | Task |
|------|------|
| `bldc_data_acquisition_gui.py` | PyQt5 GUI for ODrive motor telemetry: live current (Iq/Id/Ibus), temperature, errors. Real-time plotting with OpenGL 3D pose visualization. Record/replay JSONL sessions. |
| `pose_data_acquisition_gui.py` | PyQt5 GUI for synchronized wheel + ZED pose collection. 4 plots (ZED pos/ori, wheel pos/ori). JSONL recording and replay with speed controls. |
| `bldc_*.jsonl` | Recorded session files. Each line is a JSON telemetry or pose record with timestamp and state. |

## Usage

```bash
# BLDC telemetry (connect to rover WebSocket server first)
python3 bldc_data_acquisition_gui.py

# Wheel + ZED pose calibration
python3 pose_data_acquisition_gui.py
```
