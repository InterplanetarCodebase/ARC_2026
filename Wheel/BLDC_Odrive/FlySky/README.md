# PWM to ODrive — Differential Drive Robot

A direct bridge between a FlySky RC receiver and two ODrive motor controllers,
running on a Jetson Xavier. No ROS, no WebSocket — just RC input mapped straight
to wheel velocity commands.

---

## System Overview

The robot has four wheels driven by two ODrive v3.x controllers:

- **odrv0** controls the left side — Front Left (FL) and Rear Left (RL)
- **odrv1** controls the right side — Front Right (FR) and Rear Right (RR)

The right side motors are physically mounted in reverse, so their velocity
commands are negated in software to compensate.

An Arduino Nano reads two PWM channels from the FlySky RC receiver and
streams the pulse widths over USB Serial to the Jetson at 20Hz.
The Jetson reads that serial stream and translates it into wheel velocity
commands sent directly to the ODrives over USB.

---

## Hardware Connections

```
FlySky Receiver
    CH1 (steering)  →  Arduino Nano D10
    CH2 (throttle)  →  Arduino Nano D11

Arduino Nano  →  Jetson Xavier (USB Serial, /dev/ttyUSB0)

Jetson Xavier  →  odrv0 (USB, serial: 336D33573235)  →  FL + RL motors
               →  odrv1 (USB, serial: 335A33633235)  →  FR + RR motors
```

---

## Arduino Nano — PWM Reader

The Arduino uses pin-change interrupts on D10 and D11 to measure the pulse
width of both RC channels simultaneously without polling. On every rising edge
it records a timestamp; on every falling edge it computes the pulse width.

Pulse widths are validated to be within 750–2250µs (standard RC range with
margin) before being stored. If a channel goes silent for more than 200ms,
the Arduino reports 0 for that channel to signal loss.

Every 50ms the Arduino prints a line over Serial:

```
CH1:1500 CH2:1523
```

---

## Jetson Script — pwm_to_odrive.py

### PWM to Velocity Mapping

Standard RC PWM ranges from 1000µs (full reverse/left) to 2000µs (full
forward/right), with 1500µs as neutral.

Each channel is mapped to a normalized value between -1.0 and +1.0.
A deadband of ±60µs around neutral is applied to absorb FlySky receiver
noise — without this, the wheels would creep slightly at rest.

- **CH2** maps to `x` (forward/back throttle). It is negated because the
  FlySky transmitter outputs high PWM when the stick is pulled back.
- **CH1** maps to `z` (steering/yaw).

### Differential Drive Mix

```
left  = x - z
right = x + z
```

Both values are clamped to [-1.0, +1.0] then scaled by the configured
maximum velocity (default 4.0 turns/second).

### ODrive Control

Each ODrive axis is configured in velocity control mode with a velocity
ramp rate of 20 turns/s² to smooth acceleration. All four axes are put into
closed-loop control on startup.

### Failsafe

If no valid serial data arrives for 0.5 seconds, or if both RC channels
simultaneously report signal loss (value 0), the motors are commanded to
zero velocity. They resume as soon as valid data returns.

### ODrive Reconnect

If an ODrive is not found at startup (e.g. not powered yet), the script
waits and retries every 5 seconds indefinitely — it will connect
automatically once the ODrive powers on.

If an ODrive disconnects mid-run, the script detects the communication
failure, identifies which one dropped by pinging both, and reconnects and
re-arms only the affected one. The other ODrive is left undisturbed.

### CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Serial port of the Arduino Nano |
| `--max_vel` | `4.0` | Maximum wheel velocity in turns/second (1.0–10.0) |
| `--invert_ch1` | off | Invert steering direction |
| `--invert_ch2` | off | Invert throttle direction |

---

## Systemd Service — pwm-to-odrive.service

The script runs as a systemd service so it starts automatically on boot
without needing to SSH in.

- Runs as user `interplanetar` with group `dialout` (required for serial port access)
- Restarts automatically after 5 seconds if the script crashes
- Sends `SIGINT` on stop so the `finally` block runs, stopping and idling all motors cleanly
- Logs to journald

### Useful Commands

```bash
# View live logs
journalctl -u pwm-to-odrive -f

# Check status
sudo systemctl status pwm-to-odrive

# Stop the service (motors will idle)
sudo systemctl stop pwm-to-odrive

# Restart after a config change
sudo systemctl restart pwm-to-odrive

# Disable autostart
sudo systemctl disable pwm-to-odrive
```

---

## Tuning Notes

### Deadband

If wheels creep at rest, increase `PWM_DEADBAND` in `pwm_to_odrive.py`.
If the robot feels slow to respond when you first move the stick, decrease it.

```python
PWM_DEADBAND = 60   # us — good starting point for FlySky noise
```

After editing, restart the service:

```bash
sudo systemctl restart pwm-to-odrive
```

### Direction

If forward/back or left/right are inverted, use the CLI flags:

```bash
# In /etc/systemd/system/pwm-to-odrive.service, edit ExecStart:
ExecStart=/usr/bin/python3 /home/interplanetar/pwm_to_odrive.py --max_vel 4.0 --invert_ch2
```

Then reload:

```bash
sudo systemctl daemon-reload
sudo systemctl restart pwm-to-odrive
```

### Max Velocity

Increase `--max_vel` in the service `ExecStart` line for a faster robot.
Valid range is 1.0–10.0 turns/second.

---

## Motor Layout Reference

```
        FRONT
   FL (odrv0.axis0)   FR (odrv1.axis0, negated)
   RL (odrv0.axis1)   RR (odrv1.axis1, negated)
        REAR
```