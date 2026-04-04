#!/usr/bin/env python3
"""
collect_wheel_zed_raw.py
========================
Collect synchronized raw data for wheel odometry calibration against ZED.

What this records per row:
- Latest drive command from websocket telemetry (x_i8, z_i8, left/right command)
- Latest wheel odometry (nav_msgs/Odometry)
- Latest ZED odometry (nav_msgs/Odometry)
- Raw wheel encoder estimates from telemetry (FL, RL, FR, RR)

Usage example:
  python3 collect_wheel_zed_raw.py \
    --ros-args \
    -p rover_ip:=192.168.10.176 \
    -p wheel_odom_topic:=/wheel_odom \
    -p zed_odom_topic:=/zed/zed_node/odom \
    -p log_rate_hz:=100.0 \
    -p output_csv:=/tmp/wheel_zed_raw.csv
"""

from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

try:
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import WebSocketException

    WEBSOCKETS_OK = True
except Exception:
    ws_connect = None
    WebSocketException = Exception
    WEBSOCKETS_OK = False


def yaw_from_quaternion(q) -> float:
    # Planar yaw from quaternion (ROS uses x,y,z,w order).
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def to_float(value):
    try:
        return float(value)
    except Exception:
        return None


class WheelZedRawCollector(Node):
    def __init__(self):
        super().__init__("wheel_zed_raw_collector")

        self.declare_parameter("rover_ip", "192.168.10.176")
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("ws_reconnect_s", 1.0)
        self.declare_parameter("wheel_odom_topic", "/wheel_odom")
        self.declare_parameter("zed_odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("log_rate_hz", 100.0)
        self.declare_parameter("output_csv", "")

        self.rover_ip = self.get_parameter("rover_ip").value
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.ws_reconnect_s = float(self.get_parameter("ws_reconnect_s").value)
        self.wheel_odom_topic = self.get_parameter("wheel_odom_topic").value
        self.zed_odom_topic = self.get_parameter("zed_odom_topic").value
        self.log_rate_hz = float(self.get_parameter("log_rate_hz").value)

        out = self.get_parameter("output_csv").value
        self.output_csv = out.strip() if isinstance(out, str) else ""
        if not self.output_csv:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_csv = f"wheel_zed_raw_{stamp}.csv"

        self.ws_url = f"ws://{self.rover_ip}:{self.ws_port}"
        self.ws = None
        self.ws_connected = False
        self.next_ws_retry = 0.0

        self.latest_wheel_odom = None
        self.latest_zed_odom = None
        self.latest_telem = {
            "seq": None,
            "t_monotonic": None,
            "cmd_seq": None,
            "cmd_x_i8": None,
            "cmd_z_i8": None,
            "cmd_left_i8": None,
            "cmd_right_i8": None,
            "cmd_left_vel_tps": None,
            "cmd_right_vel_tps": None,
            "fl_pos": None,
            "rl_pos": None,
            "fr_pos": None,
            "rr_pos": None,
            "left_turns_avg": None,
            "right_turns_avg": None,
        }

        self.wheel_sub = self.create_subscription(Odometry, self.wheel_odom_topic, self.on_wheel_odom, 50)
        self.zed_sub = self.create_subscription(Odometry, self.zed_odom_topic, self.on_zed_odom, 50)

        self.csv_file = None
        self.csv_writer = None
        self.rows_written = 0
        self._open_csv()

        if not WEBSOCKETS_OK:
            self.get_logger().error("websockets package missing. Install with: pip install websockets")

        period = 1.0 / max(1.0, self.log_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"collector started: wheel={self.wheel_odom_topic}, zed={self.zed_odom_topic}, ws={self.ws_url}, rate={self.log_rate_hz:.1f} Hz, csv={self.output_csv}"
        )

    def _open_csv(self):
        out_dir = os.path.dirname(self.output_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        self.csv_file = open(self.output_csv, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_columns())
        self.csv_writer.writeheader()
        self.csv_file.flush()

    @staticmethod
    def csv_columns():
        return [
            "log_wall_time",
            "log_ros_time",
            "telem_seq",
            "telem_t_monotonic",
            "cmd_seq",
            "cmd_x_i8",
            "cmd_z_i8",
            "cmd_left_i8",
            "cmd_right_i8",
            "cmd_left_vel_tps",
            "cmd_right_vel_tps",
            "wheel_fl_pos",
            "wheel_rl_pos",
            "wheel_fr_pos",
            "wheel_rr_pos",
            "wheel_left_turns_avg",
            "wheel_right_turns_avg",
            "wheel_stamp",
            "wheel_x",
            "wheel_y",
            "wheel_z",
            "wheel_yaw",
            "wheel_vx",
            "wheel_vy",
            "wheel_vz",
            "wheel_wx",
            "wheel_wy",
            "wheel_wz",
            "zed_stamp",
            "zed_x",
            "zed_y",
            "zed_z",
            "zed_yaw",
            "zed_vx",
            "zed_vy",
            "zed_vz",
            "zed_wx",
            "zed_wy",
            "zed_wz",
        ]

    def on_wheel_odom(self, msg: Odometry):
        self.latest_wheel_odom = msg

    def on_zed_odom(self, msg: Odometry):
        self.latest_zed_odom = msg

    def _try_ws_connect(self):
        if not WEBSOCKETS_OK:
            return

        now = time.monotonic()
        if now < self.next_ws_retry:
            return

        try:
            self.ws = ws_connect(self.ws_url, open_timeout=0.25, close_timeout=0.25)
            self.ws_connected = True
            self.get_logger().info(f"Connected telemetry websocket: {self.ws_url}")
        except Exception as exc:
            self.ws_connected = False
            self.next_ws_retry = now + self.ws_reconnect_s
            self.get_logger().warn(f"Telemetry websocket connect failed: {exc}")

    def _close_ws(self, reason: str = ""):
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass
        self.ws = None
        self.ws_connected = False
        self.next_ws_retry = time.monotonic() + self.ws_reconnect_s
        if reason:
            self.get_logger().warn(reason)

    def _poll_ws(self, max_messages: int = 20):
        if self.ws is None or not self.ws_connected:
            return

        import json

        for _ in range(max_messages):
            try:
                msg = self.ws.recv(timeout=0.0)
            except TimeoutError:
                break
            except (OSError, WebSocketException, RuntimeError) as exc:
                self._close_ws(f"Telemetry websocket receive failed: {exc}")
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

                cmd = data.get("command") if isinstance(data.get("command"), dict) else {}
                odrv0 = data.get("odrv0") if isinstance(data.get("odrv0"), dict) else {}
                odrv1 = data.get("odrv1") if isinstance(data.get("odrv1"), dict) else {}
                axes0 = odrv0.get("axes") if isinstance(odrv0.get("axes"), dict) else {}
                axes1 = odrv1.get("axes") if isinstance(odrv1.get("axes"), dict) else {}
                a0 = axes0.get("axis0") if isinstance(axes0.get("axis0"), dict) else {}
                a1 = axes0.get("axis1") if isinstance(axes0.get("axis1"), dict) else {}
                b0 = axes1.get("axis0") if isinstance(axes1.get("axis0"), dict) else {}
                b1 = axes1.get("axis1") if isinstance(axes1.get("axis1"), dict) else {}

                fl = to_float(a0.get("pos_estimate"))
                rl = to_float(a1.get("pos_estimate"))
                fr = to_float(b0.get("pos_estimate"))
                rr = to_float(b1.get("pos_estimate"))

                left_avg = None
                right_avg = None
                if fl is not None and rl is not None:
                    left_avg = (fl + rl) / 2.0
                if fr is not None and rr is not None:
                    right_avg = (fr + rr) / 2.0

                self.latest_telem = {
                    "seq": data.get("seq"),
                    "t_monotonic": to_float(data.get("t_monotonic")),
                    "cmd_seq": cmd.get("seq"),
                    "cmd_x_i8": cmd.get("x_i8"),
                    "cmd_z_i8": cmd.get("z_i8"),
                    "cmd_left_i8": cmd.get("left_i8"),
                    "cmd_right_i8": cmd.get("right_i8"),
                    "cmd_left_vel_tps": to_float(cmd.get("left_vel_tps")),
                    "cmd_right_vel_tps": to_float(cmd.get("right_vel_tps")),
                    "fl_pos": fl,
                    "rl_pos": rl,
                    "fr_pos": fr,
                    "rr_pos": rr,
                    "left_turns_avg": left_avg,
                    "right_turns_avg": right_avg,
                }
            except Exception:
                continue

    @staticmethod
    def _odom_parts(msg: Odometry | None):
        if msg is None:
            return {
                "stamp": None,
                "x": None,
                "y": None,
                "z": None,
                "yaw": None,
                "vx": None,
                "vy": None,
                "vz": None,
                "wx": None,
                "wy": None,
                "wz": None,
            }

        pose = msg.pose.pose
        twist = msg.twist.twist
        return {
            "stamp": f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw_from_quaternion(pose.orientation),
            "vx": twist.linear.x,
            "vy": twist.linear.y,
            "vz": twist.linear.z,
            "wx": twist.angular.x,
            "wy": twist.angular.y,
            "wz": twist.angular.z,
        }

    def _write_row(self):
        if self.csv_writer is None or self.csv_file is None:
            return

        wheel = self._odom_parts(self.latest_wheel_odom)
        zed = self._odom_parts(self.latest_zed_odom)

        now_ros = self.get_clock().now().nanoseconds / 1e9
        row = {
            "log_wall_time": f"{time.time():.6f}",
            "log_ros_time": f"{now_ros:.9f}",
            "telem_seq": self.latest_telem["seq"],
            "telem_t_monotonic": self.latest_telem["t_monotonic"],
            "cmd_seq": self.latest_telem["cmd_seq"],
            "cmd_x_i8": self.latest_telem["cmd_x_i8"],
            "cmd_z_i8": self.latest_telem["cmd_z_i8"],
            "cmd_left_i8": self.latest_telem["cmd_left_i8"],
            "cmd_right_i8": self.latest_telem["cmd_right_i8"],
            "cmd_left_vel_tps": self.latest_telem["cmd_left_vel_tps"],
            "cmd_right_vel_tps": self.latest_telem["cmd_right_vel_tps"],
            "wheel_fl_pos": self.latest_telem["fl_pos"],
            "wheel_rl_pos": self.latest_telem["rl_pos"],
            "wheel_fr_pos": self.latest_telem["fr_pos"],
            "wheel_rr_pos": self.latest_telem["rr_pos"],
            "wheel_left_turns_avg": self.latest_telem["left_turns_avg"],
            "wheel_right_turns_avg": self.latest_telem["right_turns_avg"],
            "wheel_stamp": wheel["stamp"],
            "wheel_x": wheel["x"],
            "wheel_y": wheel["y"],
            "wheel_z": wheel["z"],
            "wheel_yaw": wheel["yaw"],
            "wheel_vx": wheel["vx"],
            "wheel_vy": wheel["vy"],
            "wheel_vz": wheel["vz"],
            "wheel_wx": wheel["wx"],
            "wheel_wy": wheel["wy"],
            "wheel_wz": wheel["wz"],
            "zed_stamp": zed["stamp"],
            "zed_x": zed["x"],
            "zed_y": zed["y"],
            "zed_z": zed["z"],
            "zed_yaw": zed["yaw"],
            "zed_vx": zed["vx"],
            "zed_vy": zed["vy"],
            "zed_vz": zed["vz"],
            "zed_wx": zed["wx"],
            "zed_wy": zed["wy"],
            "zed_wz": zed["wz"],
        }

        self.csv_writer.writerow(row)
        self.rows_written += 1

        if self.rows_written % 50 == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f"rows={self.rows_written} cmd=({self.latest_telem['cmd_x_i8']},{self.latest_telem['cmd_z_i8']}) wheel_xy=({wheel['x']},{wheel['y']}) zed_xy=({zed['x']},{zed['y']})"
            )

    def on_timer(self):
        if self.ws is None or not self.ws_connected:
            self._try_ws_connect()

        self._poll_ws()
        self._write_row()

    def destroy_node(self):
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
                self.get_logger().info(f"Saved {self.rows_written} rows to {self.output_csv}")
        finally:
            self._close_ws()
            super().destroy_node()


def main():
    rclpy.init()
    node = WheelZedRawCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
