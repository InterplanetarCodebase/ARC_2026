# BLDC Motor Control with ODrive 3.6

This directory contains configuration and control scripts for the ODrive 3.6 motor controller driving BLDC motors on the rover's wheel system. ODrive is an open-source motor controller capable of sensored and sensorless commutation, position control, and velocity control with real-time feedback.

## Hardware Overview

- **Motor Controller**: Maker base Xdrive (based on ODrive 3.6, 54V rated)
- **Input Voltage**: 36 VDC (battery)
- **Max Current**: 75 A continuous per axis
- **Motor Configuration**: BLDC motor with Hall-effect encoder
- **Encoder**: 90 CPR (counts per revolution) Hall-effect sensor
- **Control Modes**: Velocity control (primary)

## Files Description

### `odrive_calibration.py`
Automated calibration and setup script for the ODrive controller. This script:
- Erases existing configuration
- Applies motor and encoder settings
- Performs full calibration sequence (~40s per axis)
- Configures Hall-effect encoder
- Starts closed-loop velocity control

**Features:**
- CLI argument support: `--axis 0 1` (specify which axes to configure)
- Device selection: `--device odrv0` or `dev0`
- Comprehensive error decoding with helpful troubleshooting hints
- Automatic reboot handling between steps

**Motor Configuration Applied:**
- Pole pairs: 15
- Hall encoder: 90 CPR
- Control mode: Velocity control
- Torque constant: 0.5169
- Velocity limit: 10 turns/s

### `odrive_gui_test.py`
Interactive GUI control application with real-time telemetry and plotting. Provides:
- Live velocity and position readouts
- Matplotlib real-time plotting (10-second history window)
- Keyboard control for manual motor testing
- Dark navy-blue / orange themed dashboard

**Keyboard Controls:**
- **W**: Forward (positive velocity)
- **S**: Reverse (negative velocity)
- **Q**: Quit and idle motors
- **Other keys**: Stop (zero velocity)

**Features:**
- Velocity input range: -5 to +5 turns/s (values clamped)
- Position calibration: First reading stored as offset, display starts at 0
- Refresh rate: 20 Hz
- Simulation mode available (when `SIMULATE = True`)

### `config.txt`
Manual configuration reference file containing all ODrive settings in command format. This is a reference guide showing the exact configuration applied by `odrive_calibration.py`. Can be used for manual configuration via `odrivetool` if needed.

## Installation & Setup

### Prerequisites
Install the ODrive USB drivers and Python library (specific older firmware version required):
```bash
pip3 install odrive==0.5.1.post0
```

Note: Version 0.5.1.post0 is the correct version for this hardware configuration and driver compatibility.

### First-Time Motor Setup

1. **Connect Hardware:**
   - Connect 56V power supply to ODrive
   - Connect BLDC motor phases (U, V, W) to axis port
   - Connect Hall encoder (3 pins) to Hall sensor input
   - Connect USB to computer for configuration

2. **Run Calibration Script:**
   
   Calibration process guided by: https://www.youtube.com/watch?v=9UxTPxgvOAA
   
   ```bash
   # Configure both axes (axes 0 and 1)
   python3 odrive_calibration.py --axis 0 1
   
   # Configure single axis
   python3 odrive_calibration.py --axis 0
   ```

3. **Expected Output:**
   - Step 1: Configuration erased
   - Step 2: Motor/encoder settings applied
   - Step 3: Full calibration (~40s) — wait without moving the motor
   - Step 4: Closed-loop control enabled
   - Summary shows active axes ready for control

### Motor Control Runtime

After calibration, motors are ready for velocity commands:
```python
odrv0.axis0.controller.input_vel = 2.5  # 2.5 turns/s
odrv0.axis1.controller.input_vel = -1.0  # -1 turn/s (reverse)
```

### GUI Testing

Launch the control GUI:
```bash
python3 odrive_gui_test.py
```

Use keyboard controls to test motor response in real-time with live plotting.

## Configuration Parameters

### Motor Config
| Parameter | Value | Purpose |
|-----------|-------|---------|
| pole_pairs | 15 | Number of magnetic poles / 2 |
| torque_constant | 0.5169 | Nm/A motor constant |
| resistance_calib_max_voltage | 4 V | Max voltage during resistance calibration |
| requested_current_range | 25 A | Peak current during calibration |
| current_control_bandwidth | 100 Hz | Current loop bandwidth |

### Encoder Config
| Parameter | Value | Purpose |
|-----------|-------|---------|
| mode | ENCODER_MODE_HALL | Hall-effect sensor mode |
| cpr | 90 | Counts per revolution |
| calib_scan_distance | 150 | Encoder scan distance during calibration |
| bandwidth | 100 Hz | Encoder feedback bandwidth |

### Controller Config
| Parameter | Value | Purpose |
|-----------|-------|---------|
| control_mode | CONTROL_MODE_VELOCITY_CONTROL | Velocity control loop |
| vel_limit | 10 turns/s | Maximum velocity limit |
| pos_gain | 1.0 | Position loop gain |
| vel_gain | computed | Velocity loop gain |
| vel_integrator_gain | computed | Velocity integral gain |

## Troubleshooting

### Calibration Fails at Motor Configuration
**Error**: `MOTOR_DISARMED` or motor phase coil test fails

**Fixes:**
- Check motor phase wiring (U, V, W) to ODrive motor connections
- Verify 56V power supply is connected and providing voltage
- Ensure motor power connector is fully seated
- Check for blown motor phase or shorted coils (resistance between phases should be 0.5–5 Ω)

### Calibration Fails at Encoder
**Error**: `ENCODER_FAILED` or Hall sensor reading issues

**Fixes:**
- Verify Hall sensor wiring to ODrive (3 wires: 5V, GND, signal)
- Check encoder cable for cuts or corrosion
- Confirm encoder CPR is correct (90 for this setup)
- Hall sensor must have clear, unobstructed rotation

### Closed-Loop Control Won't Start
**Error**: Remains in IDLE state after calibration

**Fixes:**
- Verify both motor and encoder are healthy (no pre-existing errors)
- Check that calibration completed successfully
- Ensure motor can rotate freely without mechanical resistance
- Try reducing `vel_limit` to lower values (e.g., 5 turns/s) if power supply is marginal

### USB Connection Lost
**Error**: "Could not connect to ODrive"

**Fixes:**
- Reconnect USB cable
- Check for multiple ODrive processes: `lsof | grep odrive` and kill if needed
- Ensure ODrive has sufficient power (56V minimum for this config)
- Try restarting the calibration script

### Motor Spins Backward
**Solution**: Reverse phase wiring or set negative velocity commands
```python
odrv0.axis0.controller.input_vel = -2.5  # Reverse direction
```

## Performance Notes

- Calibration time: ~50s per axis (includes erase, config, and calibration)
- Velocity response: ~100 ms settling with current settings
- Maximum tested velocity: 10 turns/s (can be increased via `vel_limit` config)
- Steady-state ripple: < 2% with Hall encoder feedback

## References

- [ODrive Documentation](https://docs.odriverobotics.com/)
- [Hall-Effect Encoder Setup](https://docs.odriverobotics.com/en/latest/guides/encoders.html)
- ODrive GitHub: https://github.com/odriverobotics/ODrive