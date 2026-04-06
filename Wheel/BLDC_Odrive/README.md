# BLDC Motor Control with ODrive 3.6

**What it does:** Core scripts for configuring and controlling BLDC motors via ODrive 3.6 motor controller with Hall-effect encoder feedback.

## Files

- **`odrive_calibration.py`** — Erases config, applies motor/encoder settings, runs full calibration (~40s per axis), enables closed-loop velocity control.
- **`odrive_gui_test.py`** — Interactive keyboard control (W/S/Q) with real-time velocity/position plotting.
- **`ws_to_diffdrive.py`** — WebSocket server + ROS2 odometry publisher. Main production script: receives drive packets, commands ODrive, publishes `/wheel_odom`, `/wheel_pose`, `/tf`.
- **`ws_to_diffdrive_ros.py`** — Copy of `ws_to_diffdrive.py` (ROS integrated).
- **`ws_to_diffdrive_rosless.py`** — WebSocket-only variant (no ROS, telemetry only).
- **`wheel_odom_node.py`** — Legacy ROS2 node (superseded by integrated `ws_to_diffdrive.py`).
- **`esp_serial_to_ws.py`** — Serial-to-WebSocket bridge for Arduino command packets.
- **`urdf/rover_visual.urdf`** — Lightweight rover visual model for RViz.
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

**4. Run drive server + ROS odometry (main production):**
```bash
python3 ws_to_diffdrive.py --max_vel 4.0 --yaw_scale 0.5 --odom_rate_hz 100.0
```

Listens on `ws://0.0.0.0:8765` for drive commands and publishes ROS topics simultaneously.

## RViz Rover Visualization (URDF)

This is for visual appeal and pose feedback in RViz, not precision geometry.

1. Run integrated drive + odometry server:
```bash
python3 ws_to_diffdrive.py --max_vel 4.0 --yaw_scale 0.5 --odom_rate_hz 100.0
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

## File Details

### `odrive_calibration.py`
**Runs on:** Device connected to ODrives

- Calibrates **all 4 ODrives at once** (one script run, sequential execution)
- Serial numbers **hardcoded** in script
- **Control mode:** Velocity control (default)
- **Velocity limit:** 10 turns/s ⚠️ **PROHIBITED to exceed**
- **CRITICAL:** `axis.motor.config.current_lim = 20.0` **MUST** be set
  - If set to 10.0 A: wheels cannot overcome obstacles, get trapped
  - 20.0 A provides sufficient torque to push through resistance

### `wheel_odom_node.py`
**Runs on:** Base station

- Differential drive odometry + pose publisher
- **Critical tuning:** Z-axis rotation scale factor = **0.5**
  - Without this: rotation values appear 2× larger than ground truth
  - Empirically calibrated against ZED ground truth
- **Right wheels (FR, RR):** Physically inverted → values negated in pose/odometry calculation
- Publishes: `/wheel_odom`, `/wheel_pose`, `odom → base_link` TF

### `ws_to_diffdrive_ros.py`
**Runs on:** JETSON Xavier

- Main controller script: WebSocket ↔ ODrive ↔ ROS2
- Receives drive commands from base station WebSocket
- Commands ODrive motors on Jetson
- **Publishes ROS topics** (`/wheel_odom`, `/wheel_pose`, `/tf`) visible on base station
- **Critical parameter:** `VELOCITY_RAMP_RATE = 20.0` turns/s²
  - Ensures rapid deceleration when telemetry stops
  - Improves responsiveness: higher ramp rate = faster stop
- Returns motor telemetry (current, temp, errors) via WebSocket

### `ws_to_diffdrive_rosless.py`
**Runs on:** JETSON Xavier (alternative)

- Identical to `ws_to_diffdrive_ros.py` but **ROS-free**
- Use when base station and Jetson have incompatible ROS versions
- WebSocket only (no ROS topics published)

## Motor Config

- Pole pairs: 15 | Hall encoder: 90 CPR | Velocity limit: 10 turns/s
- Right side motors (FR, RR) negated for differential drive alignment
- **Current limit:** 20.0 A (hardcoded in calibration, essential for obstacle traverse)
- **Velocity ramp rate:** 20.0 turns/s² (fast deceleration for safety)