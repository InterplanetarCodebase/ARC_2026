# Arm Control System — UDP to Serial Bridge

Robotic arm control system with base station → Jetson → ESP32 command pipeline.

## Files

- **`arm_controller.ino`** — Arduino/ESP32 firmware. Receives serial commands and controls arm servos/motors.
- **`arm_receiver.py`** — Runs on Jetson Xavier. Receives UDP commands from base station and forwards to ESP32 via serial. Includes watchdog for safety.

## Quick Start

### 1. Flash ESP32 (Arduino IDE)

```bash
# Open arm_controller.ino in Arduino IDE
# Select board: ESP32 Dev Module (or your specific board)
# Select port: /dev/ttyUSB0 (or appropriate port)
# Click Upload
```

### 2. Run on Jetson Xavier

```bash
python3 arm_receiver.py
```

**Default configuration:**
- Listens on UDP port `5760`
- Connects to ESP32 on `/dev/ttyUSB0` @ 921600 baud
- Watchdog timeout: 2 seconds (sends ESTOP if no commands received)
- Heartbeat interval: 0.2 seconds

### 3. Send Commands from Base Station

Commands use binary protocol with CRC8 validation:
```
[SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
```

## Protocol Features

- **Sequence numbers** — Prevents replay attacks and duplicate commands
- **CRC8 validation** — Ensures data integrity
- **ACK relay** — ESP32 acknowledgments forwarded back to base station
- **Watchdog** — Auto-ESTOP if base station link lost (>2s timeout)
- **Heartbeat** — Keeps ESP32 watchdog alive

## Modifying Configuration

Edit these constants in `arm_receiver.py`:
```python
UDP_LISTEN_PORT = 5760
SERIAL_PORT     = "/dev/ttyUSB0"
SERIAL_BAUD     = 921600
WATCHDOG_TIMEOUT = 2.0    # seconds
```
