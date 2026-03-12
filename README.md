# Team Interplanetar — Mars Rover Control Stack
### BUET | Anatolian Rover Challenge & European Rover Challenge

---

## Overview

Complete control software and firmware for the Team Interplanetar BUET rover. Uses **ROS2 Humble** with **CycloneDDS** for low-latency teleoperation over WiFi. Build type is **ament_python** — no C++ compilation required.

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
│  │    /cmd_vel   geometry_msgs/Twist       @ 20 Hz          │   │
│  │    /arm_cmd   std_msgs/Int16MultiArray  @ 20 Hz          │   │
│  └──────────────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────────────┘
                            │ CycloneDDS over WiFi (UDP multicast)
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

---

## Topic Reference

| Topic | Type | QoS | Publisher | Subscriber |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Best Effort, depth 1 | `gui_node` | `wheel_bridge_node` |
| `/arm_cmd` | `std_msgs/Int16MultiArray` | Reliable, depth 1 | `gui_node` | `arm_bridge_node` |

### /arm_cmd Layout

`Int16MultiArray` with 7 fixed slots:

| Index | Field | Values |
|---|---|---|
| 0 | motor1_cmd (Base) | 0x11=FWD, 0x12=REV, 0x13=STOP |
| 1 | motor2_cmd (Shoulder) | 0x21=FWD, 0x22=REV, 0x23=STOP |
| 2 | motor3_cmd (Elbow) | 0x31=FWD, 0x32=REV, 0x33=STOP |
| 3 | motor4a_cmd (Roller) | 0x41=FWD, 0x42=REV, 0x43=STOP |
| 4 | motor4b_cmd (Gripper) | 0x51=FWD, 0x52=REV, 0x53=STOP |
| 5 | servo_angle | 0–180 degrees |
| 6 | motor_speed | 0–255 (PWM) |

### QoS Rationale

- **`/cmd_vel` — Best Effort:** Drive commands are time-critical. A stale retransmitted command arriving late is more dangerous than a dropped one. The watchdog handles dropout.
- **`/arm_cmd` — Reliable:** Motor state changes must arrive. A missed STOP is unacceptable for arm safety.

### Serial Packet Formats

**Wheel (Arduino Nano):**
```
[0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]  — 8 bytes
x_i8, z_i8: throttle-scaled signed int8, applied on GUI side
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
interplanetar_rover/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── interplanetar_rover
├── interplanetar_rover/          # Python nodes
│   ├── __init__.py
│   ├── gui_node.py               # Base station: pygame teleop GUI
│   ├── wheel_bridge_node.py      # Xavier: /cmd_vel → Arduino serial
│   └── arm_bridge_node.py        # Xavier: /arm_cmd → ESP32 serial
├── launch/
│   ├── base.launch.py            # Run on base station
│   └── rover.launch.py           # Run on Jetson Xavier
├── config/
│   ├── cyclonedds_base.xml       # DDS config — base station
│   └── cyclonedds_rover.xml      # DDS config — Jetson Xavier
├── scripts/
│   ├── setup_cyclone_base.sh     # Env setup for base station
│   └── setup_cyclone_rover.sh    # Env setup for Jetson Xavier
└── firmware/
    ├── arm_esp32/
    │   └── arm_controller.ino    # ESP32 arm firmware
    └── wheel_nano/
        └── serial_wheel_nano.ino # Arduino Nano wheel firmware
```

---

## Prerequisites

Both machines must be on the **same WiFi network** and running **ROS2 Humble** (Ubuntu 22.04).

### Install ROS2 Humble
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

### Install CycloneDDS
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### Install Python dependencies
```bash
pip3 install pyserial pygame
```

---

## CycloneDDS Setup

Each machine has its own config file with its own IP address. Edit the IP before first use.

**`config/cyclonedds_base.xml`** — set to base station IP:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>192.168.1.100</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <LateAckMode>false</LateAckMode>
      <ResponsivenessTimeout>50ms</ResponsivenessTimeout>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
```

**`config/cyclonedds_rover.xml`** — set to Xavier IP:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>192.168.1.20</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <LateAckMode>false</LateAckMode>
      <ResponsivenessTimeout>50ms</ResponsivenessTimeout>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
```

The setup scripts point each machine at the correct config file automatically. The only difference between the two XMLs is `NetworkInterfaceAddress`.

---

## Environment Setup

The setup scripts source ROS2, set the DDS implementation, domain ID, and config path. They use `BASH_SOURCE` so they work from any directory.

### Base Station
```bash
source /path/to/interplanetar_rover/scripts/setup_cyclone_base.sh
```

### Jetson Xavier
```bash
source /path/to/interplanetar_rover/scripts/setup_cyclone_rover.sh
```

### Add to ~/.bashrc (recommended)
Run once on each machine so the environment is always ready:
```bash
# On base station
echo "source /path/to/interplanetar_rover/scripts/setup_cyclone_base.sh" >> ~/.bashrc

# On Xavier
echo "source /path/to/interplanetar_rover/scripts/setup_cyclone_rover.sh" >> ~/.bashrc
```

Expected output after sourcing:
```
[BASE] ROS2 ready — domain=42
[BASE] DDS config: file:///path/to/interplanetar_rover/config/cyclonedds_base.xml
```

---

## Building the Package

Run on **both** machines after sourcing the setup script:

```bash
cd /path/to/ros2_ws
colcon build --packages-select interplanetar_rover --symlink-install
```

`--symlink-install` means Python node edits take effect immediately without rebuilding.

Then re-source:
```bash
source scripts/setup_cyclone_base.sh   # or rover
```

---

## Running the System

### 1. Find serial ports (Xavier only)
```bash
ls /dev/ttyUSB*
dmesg | tail -20   # plug each USB one at a time to confirm which is which
```

### 2. Grant serial permissions (Xavier only, once)
```bash
sudo usermod -aG dialout $USER
# Log out and back in, or run immediately:
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

### 3. Start rover side (Jetson Xavier)
```bash
ros2 launch interplanetar_rover rover.launch.py \
  wheel_port:=/dev/ttyUSB0 \
  arm_port:=/dev/ttyUSB1
```

Default ports if not specified: `wheel_port=/dev/ttyUSB0`, `arm_port=/dev/ttyUSB1`.

### 4. Start base station
```bash
ros2 launch interplanetar_rover base.launch.py
```

---

## Flashing Firmware

### Arduino Nano (Wheel Controller)
- Open `firmware/wheel_nano/serial_wheel_nano.ino` in Arduino IDE
- Board: **Arduino Nano**
- Processor: **ATmega328P (Old Bootloader)** if upload fails
- Baud: **115200**
- Upload

### ESP32 (Arm Controller)
- Open `firmware/arm_esp32/arm_controller.ino` in Arduino IDE
- Install **ESP32Servo** library via Library Manager
- Board: **ESP32 Dev Module**
- Upload Speed: **921600**
- Upload

---

## Debugging

```bash
# Verify topics are live
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /arm_cmd

# Check publish rate
ros2 topic hz /cmd_vel    # expect ~20 Hz
ros2 topic hz /arm_cmd    # expect ~20 Hz

# Check both machines see each other's nodes
ros2 node list

# Record and replay a session
ros2 bag record /cmd_vel /arm_cmd
ros2 bag play <bag_directory>

# Verify CycloneDDS config is loaded
ros2 doctor --report | grep -i cyclone
echo $CYCLONEDDS_URI
echo $ROS_DOMAIN_ID
```

### Nodes can't discover each other
1. Confirm `ROS_DOMAIN_ID=42` on both machines: `echo $ROS_DOMAIN_ID`
2. Confirm `CYCLONEDDS_URI` points to the correct file and the file exists
3. Confirm both machines can reach each other: `ping <other_machine_ip>`
4. If the radio blocks multicast, add static peers to both XML configs:
```xml
<Discovery>
  <Peers>
    <Peer address="192.168.1.100"/>  <!-- base station -->
    <Peer address="192.168.1.20"/>   <!-- Xavier       -->
  </Peers>
</Discovery>
```

---

## Safety

- Both bridge nodes have a **2-second software watchdog**: no message → wheels stop / arm ESTOPs.
- `arm_bridge_node` sends `CMD_HEARTBEAT` to ESP32 every 200 ms while link is alive.
- ESP32 has an independent **1-second hardware watchdog**.
- Arduino Nano has an independent **1.5-second hardware watchdog**.
- On node shutdown, both bridges send stop/ESTOP before closing serial.

---

## Team

**Team Interplanetar — BUET**
Competing at Anatolian Rover Challenge (ARC) and European Rover Challenge (ERC).
