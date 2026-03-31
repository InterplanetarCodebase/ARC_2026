# Wheel System

Multi-motor drive control with dual hardware options.

## Architecture Overview

```
Base Station (UDP)
    ↓
Jetson Xavier (wheel_receiver.py)
    ↓
Arduino Nano (4x BTS7960 drivers)
    ↓
Motors (DC or BLDC)
```

## Subsystems

### DC_MOTOR
Traditional DC motor control via Arduino Nano with 4x BTS7960 H-bridge drivers.

**Control Interfaces:**
- **FlySky**: RC controller PWM input (CH1: steering, CH2: forward/back)
- **Serial**: UDP commands from base station (port 5761) with CRC validation

**Drive Type:** Differential drive (left = x - z, right = x + z)

**Files:**
- `wheel_receiver.py` — Jetson Xavier: listens UDP, validates CRC, forwards to Arduino, includes watchdog
- `flysky_wheel_nano.ino` — Arduino: RC PWM input direct control
- `serial_wheel_nano.ino` — Arduino: Serial protocol receiver

### BLDC_Odrive
BLDC hub motor control via Maker base Xdrive (ODrive 3.6-based) motor controller.

**Hardware:**
- Motor: BLDC hub motors (hoverboard-style)
- Controller: 54V rated, 36V DC input, 75A per axis
- Configuration: odrive==0.5.1.post0

**Files:**
- `odrive_calibration.py` — Automated setup and calibration
- `odrive_gui_test.py` — Interactive testing with real-time telemetry
