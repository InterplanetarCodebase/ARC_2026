#!/usr/bin/env python3
"""
ws_to_diffdrive.py  —  Runs on Jetson Xavier
=============================================
WebSocket server that receives drive packets from Arduino Nano
and drives ODrive motors with differential drive.

Packet format:
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]

Differential drive:
  left  = x - z
  right = x + z

Polarity (matched from experimental odrive_test.py):
  odrv0.axis0 = FL (+left_vel)
  odrv0.axis1 = RL (+left_vel)
  odrv1.axis0 = FR (-right_vel)   ← right side physically inverted
  odrv1.axis1 = RR (-right_vel)

Also publishes ROS2 topics for RViz from the same process:
    - nav_msgs/Odometry on "wheel_odom"
    - geometry_msgs/PoseStamped on "wheel_pose"
    - TF odom -> base_link
"""
from __future__ import annotations
import asyncio
import argparse
import json
import websockets
import struct
import logging
import math
import time
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)

try:
    import rclpy
    from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped, Twist, Vector3
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster

    ROS_OK = True
except Exception:
    rclpy = None
    Point = Pose = PoseStamped = Quaternion = TransformStamped = Twist = Vector3 = None
    Odometry = None
    TransformBroadcaster = None
    ROS_OK = False

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [WS→DIFFDRIVE] %(levelname)s %(message)s'
)
log = logging.getLogger(__name__)

# ─── CONFIG ────────────────────────────────────────────────────────
WS_HOST      = "0.0.0.0"
WS_PORT      = 8765
PACKET_LEN   = 8

ODRV0_SERIAL = "336D33573235"
ODRV1_SERIAL = "335A33633235"

VEL_MAX      = 4.0    # max turns/s sent to ODrive — tune this
CMD_TIMEOUT_S = 0.35   # safety timeout: stop if no fresh command arrives
WATCHDOG_PERIOD_S = 0.05

WHEEL_DIAMETER_M = 0.20
TRACK_WIDTH_M = 0.762
RIGHT_WHEEL_SIGN = -1.0
FORWARD_SIGN = -1.0
ODOM_TOPIC = "wheel_odom"
POSE_TOPIC = "wheel_pose"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"


def parse_args():
    parser = argparse.ArgumentParser(
        description="WebSocket differential-drive bridge for dual ODrive controllers."
    )
    parser.add_argument(
        "--max_vel",
        type=float,
        default=VEL_MAX,
        help="Maximum wheel velocity in turns/s (range: 1.0 to 10.0).",
    )
    parser.add_argument(
        "--yaw_scale",
        type=float,
        default=0.5,
        help="Yaw calibration scale for wheel odometry (example: 0.5).",
    )
    parser.add_argument(
        "--odom_rate_hz",
        type=float,
        default=100.0,
        help="ROS odom/pose publish rate in Hz.",
    )
    args = parser.parse_args()

    if not (1.0 <= args.max_vel <= 10.0):
        parser.error("--max_vel must be between 1.0 and 10.0")
    if args.yaw_scale <= 0.0:
        parser.error("--yaw_scale must be > 0.0")
    if args.odom_rate_hz <= 0.0:
        parser.error("--odom_rate_hz must be > 0.0")

    return args


def normalize_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


def yaw_to_quaternion(yaw: float):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

# ─── PROTOCOL ──────────────────────────────────────────────────────
SOF1 = 0xAA
SOF2 = 0xBB

# ─── CRC8 (poly 0x07) ──────────────────────────────────────────────
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

# ─── DIFF DRIVE DECODE ─────────────────────────────────────────────
def decode_packet(pkt: bytes):
    if len(pkt) != PACKET_LEN:
        log.warning(f"Bad packet length: {len(pkt)}")
        return None
    if pkt[0] != SOF1 or pkt[1] != SOF2:
        log.warning("Bad SOF bytes")
        return None
    if crc8(pkt[:7]) != pkt[7]:
        log.warning("CRC mismatch")
        return None

    seq  = (pkt[2] << 8) | pkt[3]
    x_i8 = struct.unpack('b', bytes([pkt[4]]))[0]
    z_i8 = struct.unpack('b', bytes([pkt[5]]))[0]

    left  = max(-127, min(127, x_i8 - z_i8))
    right = max(-127, min(127, x_i8 + z_i8))

    return seq, x_i8, z_i8, left, right


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def _safe_int(value):
    try:
        return int(value)
    except Exception:
        return None


def _safe_call(reader):
    try:
        return reader()
    except Exception:
        return None


def read_wheel_turns(odrv0, odrv1):
    fl = _safe_float(_safe_call(lambda: odrv0.axis0.encoder.pos_estimate))
    rl = _safe_float(_safe_call(lambda: odrv0.axis1.encoder.pos_estimate))
    fr = _safe_float(_safe_call(lambda: odrv1.axis0.encoder.pos_estimate))
    rr = _safe_float(_safe_call(lambda: odrv1.axis1.encoder.pos_estimate))
    if any(v is None for v in (fl, rl, fr, rr)):
        return None

    left_turns = ((fl + rl) / 2.0) * FORWARD_SIGN
    right_turns = ((fr + rr) / 2.0) * RIGHT_WHEEL_SIGN * FORWARD_SIGN
    return left_turns, right_turns


class RosOdomPublisher:
    def __init__(self, yaw_scale: float):
        self.yaw_scale = yaw_scale
        self.node = rclpy.create_node("ws_to_diffdrive_odom_pub")
        self.odom_pub = self.node.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, POSE_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.wheel_circumference_m = math.pi * WHEEL_DIAMETER_M
        self.prev_left = None
        self.prev_right = None
        self.last_time = time.monotonic()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.node.get_logger().info(
            f"Publishing ROS odom: odom_topic={ODOM_TOPIC}, pose_topic={POSE_TOPIC}, yaw_scale={self.yaw_scale:.3f}"
        )

    def update_from_turns(self, left_turns: float, right_turns: float):
        now = time.monotonic()
        if self.prev_left is None or self.prev_right is None:
            self.prev_left = left_turns
            self.prev_right = right_turns
            self.last_time = now
            return

        dt = now - self.last_time
        if dt <= 0.0:
            return

        d_left = (left_turns - self.prev_left) * self.wheel_circumference_m
        d_right = (right_turns - self.prev_right) * self.wheel_circumference_m
        self.prev_left = left_turns
        self.prev_right = right_turns
        self.last_time = now

        d_center = (d_left + d_right) / 2.0
        d_yaw_raw = (d_right - d_left) / TRACK_WIDTH_M
        d_yaw = d_yaw_raw * self.yaw_scale

        yaw_mid = self.yaw + (d_yaw * 0.5)
        self.x += d_center * math.cos(yaw_mid)
        self.y += d_center * math.sin(yaw_mid)
        self.yaw = normalize_angle(self.yaw + d_yaw)

        vx = d_center / dt
        v_yaw = d_yaw / dt
        stamp = self.node.get_clock().now().to_msg()
        self.publish_odom(stamp, vx, v_yaw)
        self.publish_pose(stamp)
        self.publish_tf(stamp)

    def publish_odom(self, stamp, vx: float, v_yaw: float):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = ODOM_FRAME
        msg.child_frame_id = BASE_FRAME
        msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=yaw_to_quaternion(self.yaw),
        )
        msg.twist.twist = Twist(
            linear=Vector3(x=vx, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=v_yaw),
        )
        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36
        self.odom_pub.publish(msg)

    def publish_pose(self, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = ODOM_FRAME
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quaternion(self.yaw)
        self.pose_pub.publish(msg)

    def publish_tf(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = ODOM_FRAME
        tf_msg.child_frame_id = BASE_FRAME
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = yaw_to_quaternion(self.yaw)
        self.tf_broadcaster.sendTransform(tf_msg)

    def close(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass


async def ros_odom_loop(odrv0, odrv1, odom_pub: RosOdomPublisher, rate_hz: float):
    period = 1.0 / rate_hz
    while True:
        turns = read_wheel_turns(odrv0, odrv1)
        if turns is not None:
            odom_pub.update_from_turns(turns[0], turns[1])
        await asyncio.sleep(period)


def build_telemetry(odrv0, odrv1, seq: int, command: dict | None = None) -> dict:
    def axis_payload(axis, name: str) -> dict:
        return {
            "name": name,
            "pos_estimate": _safe_float(_safe_call(lambda: axis.encoder.pos_estimate)),
            "iq_measured": _safe_float(_safe_call(lambda: axis.motor.current_control.Iq_measured)),
            "id_measured": _safe_float(_safe_call(lambda: axis.motor.current_control.Id_measured)),
            "ibus": _safe_float(_safe_call(lambda: axis.motor.current_control.Ibus)),
            "fet_thermistor_temp": _safe_float(_safe_call(lambda: axis.fet_thermistor.temperature)),
            "axis_error": _safe_int(_safe_call(lambda: axis.error)),
            "motor_error": _safe_int(_safe_call(lambda: axis.motor.error)),
            "encoder_error": _safe_int(_safe_call(lambda: axis.encoder.error)),
            "controller_error": _safe_int(_safe_call(lambda: axis.controller.error)),
        }

    return {
        "type": "odrive_telemetry",
        "seq": seq,
        "t_monotonic": time.monotonic(),
        "command": command or {
            "seq": seq,
            "x_i8": None,
            "z_i8": None,
            "left_i8": None,
            "right_i8": None,
            "left_vel_tps": None,
            "right_vel_tps": None,
        },
        "odrv0": {
            "vbus_voltage": _safe_float(getattr(odrv0, "vbus_voltage", None)),
            "axes": {
                "axis0": axis_payload(odrv0.axis0, "FL"),
                "axis1": axis_payload(odrv0.axis1, "RL"),
            },
        },
        "odrv1": {
            "vbus_voltage": _safe_float(getattr(odrv1, "vbus_voltage", None)),
            "axes": {
                "axis0": axis_payload(odrv1.axis0, "FR"),
                "axis1": axis_payload(odrv1.axis1, "RR"),
            },
        },
    }

# ─── ODRIVE SETUP ──────────────────────────────────────────────────
def setup_axis(ax, name=""):
    log.info(f"[{name}] → IDLE")
    ax.requested_state = AXIS_STATE_IDLE
    time.sleep(0.3)

    try:
        ax.error = 0
        ax.motor.error = 0
        ax.encoder.error = 0
        ax.controller.error = 0
    except Exception:
        pass

    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode    = INPUT_MODE_VEL_RAMP
    ax.controller.config.vel_ramp_rate = 20.0
    ax.controller.input_vel = 0.0

    log.info(f"[{name}] → CLOSED LOOP")
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.6)

    log.info(f"[{name}] state={ax.current_state}  error={ax.error}")


def connect_odrives():
    log.info("Connecting odrv0...")
    odrv0 = odrive.find_any(serial_number=ODRV0_SERIAL)
    time.sleep(1)   # prevent fibre race

    log.info("Connecting odrv1...")
    odrv1 = odrive.find_any(serial_number=ODRV1_SERIAL)

    log.info(f"odrv0 serial: {odrv0.serial_number}")
    log.info(f"odrv1 serial: {odrv1.serial_number}")

    setup_axis(odrv0.axis0, "odrv0.axis0 FL")
    setup_axis(odrv0.axis1, "odrv0.axis1 RL")
    setup_axis(odrv1.axis0, "odrv1.axis0 FR")
    setup_axis(odrv1.axis1, "odrv1.axis1 RR")

    return odrv0, odrv1

# ─── MOTOR HELPERS ─────────────────────────────────────────────────
def drive(odrv0, odrv1, left_vel: float, right_vel: float):
    """
    Apply velocities with experimental polarity from odrive_test.py.
    Right side motors are physically inverted → negate right_vel.
    """
    odrv0.axis0.controller.input_vel =  left_vel   # FL
    odrv0.axis1.controller.input_vel =  left_vel   # RL
    odrv1.axis0.controller.input_vel = -right_vel  # FR
    odrv1.axis1.controller.input_vel = -right_vel  # RR


def stop(odrv0, odrv1):
    """
    SAFETY CRITICAL: Stop all motors with defensive error handling.
    Each motor is stopped individually to ensure partial success even if some fail.
    """
    motors_stopped = []
    motors_failed = []
    
    # Try to stop each motor individually with error handling
    motor_configs = [
        (odrv0, 'axis0', 'FL', lambda: setattr(odrv0.axis0.controller, 'input_vel', 0.0)),
        (odrv0, 'axis1', 'RL', lambda: setattr(odrv0.axis1.controller, 'input_vel', 0.0)),
        (odrv1, 'axis0', 'FR', lambda: setattr(odrv1.axis0.controller, 'input_vel', 0.0)),
        (odrv1, 'axis1', 'RR', lambda: setattr(odrv1.axis1.controller, 'input_vel', 0.0)),
    ]
    
    for odrv, axis_name, motor_label, stop_func in motor_configs:
        try:
            stop_func()
            motors_stopped.append(motor_label)
        except Exception as e:
            motors_failed.append(motor_label)
            log.error(f"SAFETY: Failed to stop motor {motor_label}: {e}")
    
    # Log results
    if motors_stopped:
        log.info(f"SAFETY: Motors stopped → {', '.join(motors_stopped)}")
    if motors_failed:
        log.error(f"SAFETY: FAILED to stop motors → {', '.join(motors_failed)}")
    
    # Return success status
    return len(motors_failed) == 0


def idle_all(odrv0, odrv1):
    for odrv in [odrv0, odrv1]:
        try:
            odrv.axis0.requested_state = AXIS_STATE_IDLE
            odrv.axis1.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

# ─── WEBSOCKET HANDLER ─────────────────────────────────────────────
last_seq: dict[str, int] = {}
connected_clients: dict[str, object] = {}
command_clients: set[str] = set()
last_command_ts: float = 0.0
failsafe_active: bool = False


async def broadcast_telemetry(payload: str):
    stale_clients = []
    for client_id, ws in list(connected_clients.items()):
        try:
            await ws.send(payload)
        except Exception:
            stale_clients.append(client_id)

    for client_id in stale_clients:
        connected_clients.pop(client_id, None)
        last_seq.pop(client_id, None)
        command_clients.discard(client_id)
        log.info(f"Client removed during broadcast: {client_id}")


async def command_watchdog(odrv0, odrv1):
    """
    SAFETY CRITICAL: Watchdog timer that stops motors if commands stop arriving.
    Runs every WATCHDOG_PERIOD_S and checks if last command is older than CMD_TIMEOUT_S.
    """
    global failsafe_active
    watchdog_iteration = 0
    last_heartbeat_log = time.monotonic()
    
    log.info(f"WATCHDOG: Started (checking every {WATCHDOG_PERIOD_S}s, timeout={CMD_TIMEOUT_S}s)")
    
    while True:
        await asyncio.sleep(WATCHDOG_PERIOD_S)
        watchdog_iteration += 1
        
        # Heartbeat log every 10 seconds to confirm watchdog is alive
        now = time.monotonic()
        if now - last_heartbeat_log >= 10.0:
            log.info(f"WATCHDOG: Heartbeat #{watchdog_iteration} (active clients: {len(command_clients)})")
            last_heartbeat_log = now

        # Only enforce timeout when at least one client has issued commands.
        if not command_clients:
            continue

        age = now - last_command_ts
        if age > CMD_TIMEOUT_S:
            success = stop(odrv0, odrv1)
            if not failsafe_active:
                log.warning(
                    f"WATCHDOG TRIGGERED: No command for {age:.3f}s (> {CMD_TIMEOUT_S:.3f}s). "
                    f"Motors emergency stop {'SUCCEEDED' if success else 'PARTIAL/FAILED'}!"
                )
            failsafe_active = True

def make_handler(odrv0, odrv1):
    async def handle_client(websocket):
        global last_command_ts, failsafe_active
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        log.info(f"Client connected: {client_id}")
        last_seq[client_id] = -1
        connected_clients[client_id] = websocket

        try:
            async for message in websocket:
                if not isinstance(message, bytes):
                    log.warning(f"Non-binary from {client_id} — ignored")
                    continue

                result = decode_packet(message)
                if result is None:
                    continue

                seq, x_i8, z_i8, left, right = result

                # Deduplicate
                if seq == last_seq[client_id]:
                    log.debug(f"Duplicate seq {seq} — dropped")
                    continue
                last_seq[client_id] = seq

                # Scale [-127, 127] → [-VEL_MAX, +VEL_MAX]
                left_vel  = (left  / 127.0) * VEL_MAX
                right_vel = (right / 127.0) * VEL_MAX

                command_clients.add(client_id)
                drive(odrv0, odrv1, left_vel, right_vel)
                last_command_ts = time.monotonic()
                if failsafe_active:
                    log.info("Command stream restored. Leaving failsafe state.")
                    failsafe_active = False

                command_info = {
                    "seq": seq,
                    "x_i8": x_i8,
                    "z_i8": z_i8,
                    "left_i8": left,
                    "right_i8": right,
                    "left_vel_tps": left_vel,
                    "right_vel_tps": right_vel,
                }

                telemetry = build_telemetry(odrv0, odrv1, seq, command=command_info)
                await broadcast_telemetry(json.dumps(telemetry))

                log.info(
                    f"seq={seq:5d} | "
                    f"x={x_i8:+4d} z={z_i8:+4d} | "
                    f"left={left_vel:+6.2f}  right={right_vel:+6.2f} turns/s"
                )

        except websockets.exceptions.ConnectionClosed as e:
            log.warning(f"CONNECTION LOST: Client {client_id} disconnected ({e.code}: {e.reason})")
        except Exception as e:
            log.error(f"ERROR in client handler {client_id}: {type(e).__name__}: {e}")
        finally:
            # Clean up client tracking
            connected_clients.pop(client_id, None)
            last_seq.pop(client_id, None)
            was_command_client = client_id in command_clients
            command_clients.discard(client_id)
            
            # SAFETY: Stop motors if last command client is gone
            if was_command_client and not command_clients:
                log.warning(f"SAFETY: Last command client disconnected ({client_id}). Stopping motors...")
                success = stop(odrv0, odrv1)
                log.warning(f"SAFETY: Emergency stop {'SUCCEEDED' if success else 'PARTIAL/FAILED'}")
            elif was_command_client:
                log.info(f"Command client disconnected: {client_id} ({len(command_clients)} still active)")
            else:
                log.info(f"Telemetry-only client disconnected: {client_id}")

    return handle_client

# ─── MAIN ──────────────────────────────────────────────────────────
async def main(args):
    log.info("=" * 70)
    log.info("SAFETY CONFIGURATION:")
    log.info(f"  Watchdog timeout: {CMD_TIMEOUT_S}s (motors stop if no command)")
    log.info(f"  Watchdog check period: {WATCHDOG_PERIOD_S}s")
    log.info(f"  Max velocity: {args.max_vel} turns/s")
    log.info("=" * 70)
    
    odrv0, odrv1 = connect_odrives()
    
    # Start watchdog task
    watchdog_task = asyncio.create_task(command_watchdog(odrv0, odrv1))
    log.info("WATCHDOG: Task started")

    ros_odom_task = None
    ros_odom_pub = None
    if ROS_OK:
        if not rclpy.ok():
            rclpy.init(args=None)
        ros_odom_pub = RosOdomPublisher(yaw_scale=args.yaw_scale)
        ros_odom_task = asyncio.create_task(
            ros_odom_loop(odrv0, odrv1, ros_odom_pub, args.odom_rate_hz)
        )
    else:
        log.warning("ROS2 packages not found. Drive control works, but wheel_odom/wheel_pose will not be published.")

    log.info(f"WebSocket server starting on {WS_HOST}:{WS_PORT}")
    async with websockets.serve(make_handler(odrv0, odrv1), WS_HOST, WS_PORT):
        log.info("Ready — waiting for drive packets")
        try:
            await asyncio.Future()
        except (asyncio.CancelledError, KeyboardInterrupt):  # ← catch KeyboardInterrupt here too
            pass
        finally:
            log.warning("SHUTDOWN: Stopping all motors...")
            stop(odrv0, odrv1)
            watchdog_task.cancel()
            try:
                await watchdog_task
            except asyncio.CancelledError:
                pass

            if ros_odom_task is not None:
                ros_odom_task.cancel()
                try:
                    await ros_odom_task
                except asyncio.CancelledError:
                    pass

            if ros_odom_pub is not None:
                ros_odom_pub.close()
                if rclpy.ok():
                    rclpy.shutdown()

            log.info("Shutting down — stopping and idling motors")
            stop(odrv0, odrv1)
            time.sleep(0.5)       # ← increased from 0.2 to let motors settle
            idle_all(odrv0, odrv1)
            time.sleep(0.3)       # ← give ODrive USB time to flush before libusb teardown


if __name__ == "__main__":
    cli_args = parse_args()
    VEL_MAX = cli_args.max_vel
    log.info(
        f"Using max velocity: {VEL_MAX:.2f} turns/s | yaw_scale={cli_args.yaw_scale:.3f} | odom_rate_hz={cli_args.odom_rate_hz:.1f}"
    )

    try:
        asyncio.run(main(cli_args))
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt caught at top level — exited cleanly.")