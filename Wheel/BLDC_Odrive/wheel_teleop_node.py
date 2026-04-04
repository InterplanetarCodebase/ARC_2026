#!/usr/bin/env python3
"""
wheel_teleop_node.py
====================
ROS2 teleop bridge for ARC wheels.

Subscribes to geometry_msgs/TwistStamped (cmd_vel), converts to differential wheel
commands, packs them into the Arduino-style 8-byte packet expected by
ws_to_diffdrive.py, and sends over websocket.

Packet format:
  [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

try:
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import WebSocketException

    WEBSOCKETS_OK = True
except Exception:
    ws_connect = None
    WebSocketException = Exception
    WEBSOCKETS_OK = False


SOF1 = 0xAA
SOF2 = 0xBB
EOF_MARK = 0xFF


def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def to_i8(value: float) -> int:
    return int(round(clamp(value, -127.0, 127.0)))


class WheelTeleopNode(Node):
    def __init__(self):
        super().__init__("wheel_teleop_node")

        self.declare_parameter("rover_ip", "192.168.10.176")
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("send_rate_hz", 30.0)
        self.declare_parameter("cmd_timeout_s", 0.30)
        self.declare_parameter("ws_reconnect_s", 1.0)

        self.declare_parameter("wheel_diameter_m", 0.20)
        self.declare_parameter("track_width_m", 0.762)
        self.declare_parameter("vel_max_turns_s", 4.0)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_rps", 1.0)

        self.rover_ip = str(self.get_parameter("rover_ip").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.ws_reconnect_s = float(self.get_parameter("ws_reconnect_s").value)

        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.vel_max_turns_s = float(self.get_parameter("vel_max_turns_s").value)
        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_angular_rps = float(self.get_parameter("max_angular_rps").value)

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.ws_url = f"ws://{self.rover_ip}:{self.ws_port}"

        self.ws = None
        self.connected = False
        self.next_reconnect_at = 0.0

        self.seq = 0
        self.last_cmd_time = 0.0
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_sent_x = None
        self.last_sent_z = None

        self.create_subscription(TwistStamped, self.cmd_vel_topic, self.cmd_vel_cb, 20)
        self.timer = self.create_timer(1.0 / self.send_rate_hz, self.timer_cb)

        if not WEBSOCKETS_OK:
            self.get_logger().error("websockets package missing. Install with: pip install websockets")

        self.get_logger().info(
            "wheel_teleop_node started: "
            f"cmd_vel={self.cmd_vel_topic}, ws={self.ws_url}, send_rate={self.send_rate_hz:.1f}Hz"
        )

    def cmd_vel_cb(self, msg: TwistStamped):
        self.last_linear = clamp(float(msg.twist.linear.x), -self.max_linear_mps, self.max_linear_mps)
        self.last_angular = clamp(float(msg.twist.angular.z), -self.max_angular_rps, self.max_angular_rps)
        self.last_cmd_time = time.monotonic()

    def try_connect(self):
        if not WEBSOCKETS_OK:
            return

        now = time.monotonic()
        if now < self.next_reconnect_at:
            return

        try:
            self.ws = ws_connect(self.ws_url, open_timeout=0.25, close_timeout=0.25)
            self.connected = True
            self.get_logger().info(f"Connected to drive websocket: {self.ws_url}")
        except Exception as exc:
            self.connected = False
            self.next_reconnect_at = now + self.ws_reconnect_s
            self.get_logger().warn(f"Websocket connect failed: {exc}")

    def disconnect(self, reason: str = ""):
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass
        self.ws = None
        self.connected = False
        self.next_reconnect_at = time.monotonic() + self.ws_reconnect_s
        if reason:
            self.get_logger().warn(reason)

    def cmd_to_packet_axes(self, linear_mps: float, angular_rps: float) -> tuple[int, int]:
        left_mps = linear_mps - (angular_rps * self.track_width_m * 0.5)
        right_mps = linear_mps + (angular_rps * self.track_width_m * 0.5)

        left_turns_s = left_mps / self.wheel_circumference_m
        right_turns_s = right_mps / self.wheel_circumference_m

        left_i8 = to_i8((left_turns_s / self.vel_max_turns_s) * 127.0)
        right_i8 = to_i8((right_turns_s / self.vel_max_turns_s) * 127.0)

        x_i8 = to_i8((left_i8 + right_i8) * 0.5)
        z_i8 = to_i8((right_i8 - left_i8) * 0.5)
        return x_i8, z_i8

    def build_packet(self, x_i8: int, z_i8: int) -> bytes:
        seq_h = (self.seq >> 8) & 0xFF
        seq_l = self.seq & 0xFF

        payload = bytes([
            SOF1,
            SOF2,
            seq_h,
            seq_l,
            x_i8 & 0xFF,
            z_i8 & 0xFF,
            EOF_MARK,
        ])
        checksum = crc8(payload)

        self.seq = (self.seq + 1) & 0xFFFF
        return payload + bytes([checksum])

    def send_packet(self, x_i8: int, z_i8: int):
        if self.ws is None or not self.connected:
            return

        packet = self.build_packet(x_i8, z_i8)
        try:
            self.ws.send(packet)
        except (OSError, WebSocketException, RuntimeError) as exc:
            self.disconnect(f"Websocket send failed: {exc}")

    def timer_cb(self):
        if self.ws is None or not self.connected:
            self.try_connect()

        now = time.monotonic()
        cmd_age = now - self.last_cmd_time

        if cmd_age > self.cmd_timeout_s:
            x_i8, z_i8 = 0, 0
        else:
            x_i8, z_i8 = self.cmd_to_packet_axes(self.last_linear, self.last_angular)

        self.send_packet(x_i8, z_i8)

        if x_i8 != self.last_sent_x or z_i8 != self.last_sent_z:
            self.get_logger().info(
                f"tx seq={self.seq:5d} x={x_i8:+4d} z={z_i8:+4d} "
                f"(lin={self.last_linear:+.2f}m/s ang={self.last_angular:+.2f}rad/s)"
            )
            self.last_sent_x = x_i8
            self.last_sent_z = z_i8

    def destroy_node(self):
        # Send zero packets before closing to force a safe stop.
        for _ in range(3):
            self.send_packet(0, 0)
            time.sleep(0.02)
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
