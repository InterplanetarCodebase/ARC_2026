#!/usr/bin/env python3
"""
wheel_odom_node.py
==================
Host-side ROS2 node that reads ODrive telemetry over websocket and publishes
wheel odometry for RViz.

Telemetry source:
    ws://<rover_ip>:8765 from ws_to_diffdrive.py on Jetson

Robot assumptions:
    - 4-wheel differential drive (FL, RL, FR, RR)
    - wheel diameter = 0.20 m
    - track width (left-right) = 0.762 m
    - right side encoder sign is inverted by default

Published:
    - nav_msgs/Odometry on topic "wheel_odom"
    - TF odom -> base_link (optional, enabled by default)

Run:
    python3 wheel_odom_node.py
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

try:
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import WebSocketException

    WEBSOCKETS_OK = True
except Exception:
    ws_connect = None
    WebSocketException = Exception
    WEBSOCKETS_OK = False


# All zeros → RViz will not render the covariance ellipse,
# leaving only the clean arrow at a sensible scale.
POSE_COVARIANCE  = [0.0] * 36
TWIST_COVARIANCE = [0.0] * 36


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


def to_float(value) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_node")

        self.declare_parameter("rover_ip", "192.168.10.176")
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("wheel_diameter_m", 0.20)
        self.declare_parameter("track_width_m", 0.762)
        self.declare_parameter("right_wheel_sign", -1.0)
        self.declare_parameter("forward_sign", -1.0)
        self.declare_parameter("yaw_scale", 0.5)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("ws_reconnect_s", 1.0)
        self.declare_parameter("odom_topic", "wheel_odom")
        self.declare_parameter("pose_topic", "wheel_pose")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.rover_ip = self.get_parameter("rover_ip").value
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.wheel_diameter_m = float(self.get_parameter("wheel_diameter_m").value)
        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.right_wheel_sign = float(self.get_parameter("right_wheel_sign").value)
        self.forward_sign = float(self.get_parameter("forward_sign").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.ws_reconnect_s = float(self.get_parameter("ws_reconnect_s").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m
        self.ws_url = f"ws://{self.rover_ip}:{self.ws_port}"

        self.ws = None
        self.connected = False
        self.next_reconnect_at = 0.0
        self.last_telemetry_at = 0.0

        self.latest_left_turns = None
        self.latest_right_turns = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None
        self.last_time = time.monotonic()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        if not WEBSOCKETS_OK:
            self.get_logger().error("websockets package missing. Install with: pip install websockets")

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(
            f"wheel_odom_node started. odom={self.odom_topic}, pose={self.pose_topic}, ws={self.ws_url}, rate={self.publish_rate_hz:.1f} Hz, yaw_scale={self.yaw_scale:.3f}"
        )

    def try_connect(self):
        if not WEBSOCKETS_OK:
            return

        now = time.monotonic()
        if now < self.next_reconnect_at:
            return

        try:
            self.ws = ws_connect(self.ws_url, open_timeout=0.25, close_timeout=0.25)
            self.connected = True
            self.get_logger().info(f"Connected to telemetry websocket: {self.ws_url}")
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

    def extract_turns(self, data: dict) -> tuple[float, float] | None:
        try:
            axes0 = data["odrv0"]["axes"]
            axes1 = data["odrv1"]["axes"]

            fl = to_float(axes0["axis0"].get("pos_estimate"))
            rl = to_float(axes0["axis1"].get("pos_estimate"))
            fr = to_float(axes1["axis0"].get("pos_estimate"))
            rr = to_float(axes1["axis1"].get("pos_estimate"))

            if any(v is None for v in (fl, rl, fr, rr)):
                return None

            left_turns = ((fl + rl) / 2.0) * self.forward_sign
            right_turns = ((fr + rr) / 2.0) * self.right_wheel_sign * self.forward_sign
            return left_turns, right_turns
        except Exception:
            return None

    def poll_websocket(self, max_messages: int = 10):
        if self.ws is None or not self.connected:
            return

        for _ in range(max_messages):
            try:
                msg = self.ws.recv(timeout=0.0)
            except TimeoutError:
                break
            except (OSError, WebSocketException, RuntimeError) as exc:
                self.disconnect(f"Websocket receive failed: {exc}")
                break

            if msg is None:
                break

            try:
                if isinstance(msg, bytes):
                    msg = msg.decode("utf-8", errors="ignore")
                if not isinstance(msg, str):
                    continue

                data = json.loads(msg)
                if not isinstance(data, dict):
                    continue
                if data.get("type") != "odrive_telemetry":
                    continue

                turns = self.extract_turns(data)
                if turns is None:
                    continue

                self.latest_left_turns, self.latest_right_turns = turns
                self.last_telemetry_at = time.monotonic()
            except Exception:
                continue

    def timer_cb(self):
        if self.ws is None or not self.connected:
            self.try_connect()

        self.poll_websocket()

        if self.latest_left_turns is None or self.latest_right_turns is None:
            return

        now = time.monotonic()
        left_turns = self.latest_left_turns
        right_turns = self.latest_right_turns

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

        # Differential drive odometry math:
        # d_center = (d_left + d_right) / 2
        # d_yaw = (d_right - d_left) / track_width
        d_center = (d_left + d_right) / 2.0
        d_yaw_raw = (d_right - d_left) / self.track_width_m
        d_yaw = d_yaw_raw * self.yaw_scale

        yaw_mid = self.yaw + (d_yaw * 0.5)
        self.x += d_center * math.cos(yaw_mid)
        self.y += d_center * math.sin(yaw_mid)
        self.yaw = normalize_angle(self.yaw + d_yaw)

        vx = d_center / dt
        v_yaw = d_yaw / dt

        stamp = self.get_clock().now().to_msg()
        self.publish_odom(stamp, vx, v_yaw)
        self.publish_pose(stamp)
        if self.publish_tf and self.tf_broadcaster is not None:
            self.publish_tf_transform(stamp)

    def publish_odom(self, stamp, vx: float, v_yaw: float):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose = Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=yaw_to_quaternion(self.yaw),
        )
        msg.twist.twist = Twist(
            linear=Vector3(x=vx, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=v_yaw),
        )

        msg.pose.covariance  = [float(v) for v in POSE_COVARIANCE]
        msg.twist.covariance = [float(v) for v in TWIST_COVARIANCE]
        self.odom_pub.publish(msg)

    def publish_pose(self, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(self.yaw)
        self.pose_pub.publish(pose_msg)

    def publish_tf_transform(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = yaw_to_quaternion(self.yaw)

        self.tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        self.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = WheelOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()