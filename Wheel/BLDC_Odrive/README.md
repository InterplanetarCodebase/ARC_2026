# BLDC Motor Control with ODrive 3.6

**What it does:** Core scripts for configuring and controlling BLDC motors via ODrive 3.6 motor controller with Hall-effect encoder feedback.

## Files

- **`odrive_calibration.py`** — Erases config, applies motor/encoder settings, runs full calibration (~40s per axis), enables closed-loop velocity control.
- **`odrive_gui_test.py`** — Interactive keyboard control (W/S/Q) with real-time velocity/position plotting.
- **`ws_to_diffdrive.py`** — WebSocket server (port 8765) on Jetson Xavier. Receives differential drive packets from GUI, commands ODrive, sends telemetry JSON back.
- **`wheel_odom_node.py`** — ROS2 node publishing `wheel_odom`, `wheel_pose`, and `odom -> base_link` TF.
- **`urdf/rover_visual.urdf`** — Lightweight rover visual model for RViz (appearance-focused).
- **`run_robot_model.sh`** — Starts `robot_state_publisher` with the visual URDF.
- **`config.txt`** — Reference configuration for manual setup via `odrivetool`.

## Setup

```bash
pip3 install odrive==0.5.1.post0
```

## Commands

**1. Calibrate both axes (axis 0 and 1):**
```bash
python3 odrive_calibration.py --axis 0 1
```

**2. Calibrate single axis (axis 0 only):**
```bash
python3 odrive_calibration.py --axis 0
```

**3. Run GUI control with real-time plotting:**
```bash
python3 odrive_gui_test.py
```

Use keyboard: **W** (forward) / **S** (reverse) / **Q** (quit & stop).

## RViz Rover Visualization (URDF)

This is for visual appeal and pose feedback in RViz, not precision geometry.

1. Run odometry TF publisher:
```bash
python3 wheel_odom_node.py
```

2. Run robot model publisher:
```bash
bash run_robot_model.sh
```

3. Start RViz:
```bash
rviz2
```

4. In RViz:
- Set `Fixed Frame` to `odom`.
- Add `RobotModel` display.
- Add `Odometry` display on topic `/wheel_odom` (optional arrow/trail).

The URDF model will move using your existing `odom -> base_link` transform.

## Motor Config

- Pole pairs: 15 | Hall encoder: 90 CPR | Velocity limit: 10 turns/s
- Right side motors (FR, RR) negated for differential drive alignment