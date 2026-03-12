# Team Interplanetar — Mars Rover Control Stack
### BUET | Anatolian Rover Challenge & European Rover Challenge

---

## Overview

This repository contains the complete control software and firmware for the Team Interplanetar BUET rover. The stack uses **ROS2 Humble** with **CycloneDDS** for the communication backbone, providing low-latency, reliable teleoperation over WiFi with a clean path toward autonomous navigation.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        BASE STATION PC                          │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  gui_node.py  (pygame + rclpy)                           │   │
│  │                                                          │   │
│  │  Logitech Extreme 3D Pro joystick input                  │   │
│  │  Live telemetry display                                  │   │
│  │                                                          │   │
│  │  Publishes:                                              │   │
│  │    /cmd_vel   geometry_msgs/Twist  @ 20 Hz               │   │
│  │    /arm_cmd   ArmCommand           @ 20 Hz               │   │
│  └──────────────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────────────┘
                            │ CycloneDDS over WiFi
                            │ (UDP multicast / unicast)
┌───────────────────────────▼─────────────────────────────────────┐
│                       JETSON XAVIER (Rover)                     │
│                                                                 │
│  ┌─────────────────────────┐  ┌──────────────────────────────┐  │
│  │  wheel_bridge_node.py   │  │  arm_bridge_node.py          │  │
│  │                         │  │                              │  │
│  │  Sub: /cmd_vel          │  │  Sub: /arm_cmd               │  │
│  │  Watchdog: 2s → STOP    │  │  Watchdog: 2s → ESTOP        │  │
│  │  Heartbeat to Arduino   │  │  Heartbeat to ESP32 @ 5Hz    │  │
│  └──────────┬──────────────┘  └──────────────┬───────────────┘  │
│             │ USB Serial 115200               │ USB Serial 921600│
└─────────────┼─────────────────────────────────┼─────────────────┘
              │                                 │
    ┌─────────▼──────────┐           ┌──────────▼──────────┐
    │   Arduino Nano     │           │      ESP32           │
    │  serial_wheel_nano │           │  arm_controller.ino  │
    │  4x BTS7960        │           │  3x BTS7960          │
    │  Differential drive│           │  1x L298N            │
    │  4 wheel motors    │           │  Wrist servo         │
    └────────────────────┘           └─────────────────────┘
```

### Topic Reference

| Topic | Type | QoS | Publisher | Subscriber |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Best Effort, depth 1 | `gui_node` | `wheel_bridge_node` |
| `/arm_cmd` | `ArmCommand` | Reliable, depth 1 | `gui_node` | `arm_bridge_node` |

### QoS Rationale

- **`/cmd_vel` — Best Effort:** Drive commands are time-critical. A stale retransmitted command arriving late is more dangerous than a dropped one. The watchdog handles dropout.
- **`/arm_cmd` — Reliable:** Motor state changes are infrequent and must arrive. A missed STOP is unacceptable for arm safety.

### Serial Packet Formats

**Wheel (Arduino Nano):**
```
[0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]  — 8 bytes
x_i8, z_i8: throttle-scaled signed int8 from GUI
```

**Arm (ESP32):**
```
[0xAA][0xBB][SEQ_H][SEQ_L][CMD][VAL][CRC8]  — 7 bytes
ACK: [0xAC][SEQ_H][SEQ_L][STATUS]           — 4 bytes
```

### Control Mapping (Logitech Extreme 3D Pro)

| Input | Function |
|---|---|
| HAT East / West | Base CW / CCW |
| HAT North / South | Shoulder Up / Down |
| BTN 6 / BTN 4 | Elbow Up / Down |
| BTN 5 / BTN 3 | Wrist Servo Up / Down |
| BTN 8 / BTN 7 | Roller CW / CCW |
| BTN 10 / BTN 9 | Gripper Open / Close |
| Y Axis | Wheel Forward / Reverse |
| Z Twist | Wheel Turn |
| Throttle Slider | Drive Speed Scale |

---

## Repository Structure

```
.
├── README.md
├── firmware/
│   ├── arm_esp32/
│   │   └── arm_controller.ino           # ESP32 arm firmware
│   └── wheel_nano/
│       └── serial_wheel_nano.ino        # Arduino Nano wheel firmware
├── scripts/
│   ├── setup_cyclone_base.sh            # Env setup for base station
│   └── setup_cyclone_rover.sh           # Env setup for Jetson Xavier
└── ros2_ws/
    └── src/
        └── interplanetar_rover/
            ├── interplanetar_rover/
            │   ├── __init__.py
            │   ├── gui_node.py           # Base station: pygame teleop GUI
            │   ├── wheel_bridge_node.py  # Xavier: /cmd_vel → Arduino serial
            │   └── arm_bridge_node.py    # Xavier: /arm_cmd → ESP32 serial
            ├── msg/
            │   └── ArmCommand.msg        # Custom ROS2 message
            ├── launch/
            │   ├── base.launch.py        # Run on base station
            │   └── rover.launch.py       # Run on Jetson Xavier
            ├── config/
            │   └── cyclonedds.xml        # DDS tuning for low latency
            ├── resource/
            │   └── interplanetar_rover
            ├── CMakeLists.txt
            ├── package.xml
            └── setup.py
```

---

## Setup Instructions

### Prerequisites

Both machines must be on the **same WiFi network** and running **ROS2 Humble**.

#### Install ROS2 Humble (Ubuntu 22.04)
```bash
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

#### Install CycloneDDS
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

#### Install Python dependencies
```bash
pip3 install pyserial pygame
```

---

### Building the Package

Run on **both** the base station and the Jetson Xavier:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> `--symlink-install` means you can edit Python files without rebuilding.

---

### Environment Setup

#### Base Station
```bash
source scripts/setup_cyclone_base.sh
```

#### Jetson Xavier
```bash
source scripts/setup_cyclone_rover.sh
```

> Add these to `~/.bashrc` on both machines for convenience.

---

### Running the System

#### 1. Find your serial ports (on Xavier)
```bash
ls /dev/ttyUSB*
dmesg | tail -20   # after plugging each USB to identify which is which
```

#### 2. Grant serial port permissions
```bash
sudo usermod -aG dialout $USER
# Log out and back in, or run:
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

#### 3. Start rover side (on Jetson Xavier)
```bash
ros2 launch interplanetar_rover rover.launch.py \
  wheel_serial_port:=/dev/ttyUSB0 \
  arm_serial_port:=/dev/ttyUSB1
```

#### 4. Start base station (on base station PC)
```bash
ros2 launch interplanetar_rover base.launch.py
```

---

### Flashing Firmware

#### Arduino Nano (Wheel Controller)
- Open `firmware/wheel_nano/serial_wheel_nano.ino` in Arduino IDE
- Board: **Arduino Nano**
- Processor: **ATmega328P (Old Bootloader)** if needed
- Upload

#### ESP32 (Arm Controller)
- Open `firmware/arm_esp32/arm_controller.ino` in Arduino IDE
- Install **ESP32Servo** library via Library Manager
- Board: **ESP32 Dev Module**
- Upload Speed: **921600**
- Upload

---

### Debugging

```bash
# Check topics are live
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /arm_cmd

# Check publish rate
ros2 topic hz /cmd_vel    # expect ~20 Hz
ros2 topic hz /arm_cmd    # expect ~20 Hz

# Check nodes are discovered across WiFi
ros2 node list

# Record a session
ros2 bag record /cmd_vel /arm_cmd
ros2 bag play <bag_directory>
```

#### If nodes can't discover each other across WiFi
Add a static peer in `config/cyclonedds.xml`:
```xml
<Discovery>
  <Peers>
    <Peer address="192.168.10.177"/>  <!-- rover IP -->
  </Peers>
</Discovery>
```
And ensure `ROS_DOMAIN_ID` matches on both machines (set to 42 in the setup scripts).

---

## Safety

- Both bridge nodes have a **2-second software watchdog**: no message → wheels stop / arm ESTOPs.
- ESP32 has an independent **1-second hardware watchdog**.
- Arduino has an independent **1.5-second hardware watchdog**.
- On node shutdown, both bridges send stop/ESTOP before closing serial.

---

## Team

**Team Interplanetar — BUET**
Competing at Anatolian Rover Challenge (ARC) and European Rover Challenge (ERC).
