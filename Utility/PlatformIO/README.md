```markdown
# ESP32_Flash

A PlatformIO-based environment for compiling and flashing ESP32 firmware from the terminal,
set up on both a development machine and a Jetson Orin Nano.

---

## Installation

### 1. System Dependencies
```bash
# Linux / Jetson Orin Nano
sudo apt-get update
sudo apt-get install python3 python3-pip git curl
```

### 2. PlatformIO
```bash
pip install platformio
```

### 3. pyserial
```bash
pip install pyserial
```

### 4. Project Setup
```bash
mkdir ~/ESP32_Flash && cd ~/ESP32_Flash
pio project init --board esp32dev
pio pkg install --library "madhephaestus/ESP32Servo"
```

### 5. Port Permissions
```bash
sudo usermod -aG dialout $USER
# Log out and back in, or temporarily:
sudo chmod 666 /dev/ttyUSB0
```

---

## platformio.ini
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    madhephaestus/ESP32Servo
```

---

## Flashing Code

Place your `.ino` or `.cpp` code in the `src/` directory, then:

```bash
# Compile only
pio run

# Compile + Flash
pio run --target upload --upload-port /dev/ttyUSB0
```

---
    
## Serial Monitor

A custom Python script (`monitor.py`) is used instead of PlatformIO's built-in monitor,
allowing interactive serial input and output at runtime with a configurable baud rate.

### Usage
```bash
# Full command line — no prompts
python3 monitor.py /dev/ttyUSB0 115200

# Port only — prompts for baud rate
python3 monitor.py /dev/ttyUSB0

# No args — prompts for both port and baud rate
python3 monitor.py
```

- Baud rate: **115200** (921600 was found to be unstable on the CH340 USB-UART chip)
- Supports sending commands via keyboard input
- Ctrl+C to disconnect

> **Note:** Stop the monitor before flashing, otherwise the port will be busy.

---

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | ESP32 Dev Module |
| USB-UART Chip | CH340 |
| Deployment Host | Jetson Orin Nano |
| Baud Rate | 115200 |
| Flash Tool | PlatformIO CLI |

---

## Port Reference

| OS | Typical Port |
|---|---|
| Linux / Jetson | `/dev/ttyUSB0` or `/dev/ttyACM0` |
| macOS | `/dev/cu.usbserial-XXXX` |
| Windows | `COM3` or similar |

Check your port with:
```bash
dmesg | grep tty
```

---

## Project Structure

```
ESP32_Flash/
├── src/
│   └── main.cpp        # Main firmware code
├── monitor.py          # Python serial monitor with input support
├── platformio.ini      # PlatformIO board and library config
└── README.md
```

Need to keep the ino file inside src/ folder of ESP_Flash
