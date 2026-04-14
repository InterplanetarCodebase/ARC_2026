#!/usr/bin/env python3
"""
wasd_teleop.py
==============
Keyboard teleop node for ROS 2 Humble.

Publishes geometry_msgs/msg/TwistStamped on rover_controller/cmd_vel.

Controls:
    w: forward (constant speed while pressed)
    s: reverse (constant speed while pressed)
    a: left turn (constant angular speed while pressed)
    d: right turn (constant angular speed while pressed)
  x or space: immediate stop
  q: quit
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


DEFAULT_TOPIC = "rover_controller/cmd_vel"
DEFAULT_RATE_HZ = 20.0
DEFAULT_LINEAR_SPEED = 0.60
DEFAULT_ANGULAR_SPEED = 1.20
DEFAULT_KEY_HOLD_TIMEOUT_S = 0.20


def parse_args():
    parser = argparse.ArgumentParser(
        description="WASD teleop publisher for geometry_msgs/TwistStamped"
    )
    parser.add_argument("--topic", type=str, default=DEFAULT_TOPIC, help="TwistStamped topic")
    parser.add_argument("--rate_hz", type=float, default=DEFAULT_RATE_HZ, help="Publish rate")
    parser.add_argument(
        "--linear_speed",
        type=float,
        default=DEFAULT_LINEAR_SPEED,
        help="Constant |linear.x| speed while key is held (m/s)",
    )
    parser.add_argument(
        "--angular_speed",
        type=float,
        default=DEFAULT_ANGULAR_SPEED,
        help="Constant |angular.z| speed while key is held (rad/s)",
    )
    parser.add_argument(
        "--key_hold_timeout_s",
        type=float,
        default=DEFAULT_KEY_HOLD_TIMEOUT_S,
        help="Deadman timeout used to infer key release in terminal mode (s)",
    )

    args, ros_args = parser.parse_known_args()

    if args.rate_hz <= 0.0:
        parser.error("--rate_hz must be > 0")
    if args.linear_speed <= 0.0:
        parser.error("--linear_speed must be > 0")
    if args.angular_speed <= 0.0:
        parser.error("--angular_speed must be > 0")
    if args.key_hold_timeout_s <= 0.0:
        parser.error("--key_hold_timeout_s must be > 0")

    return args, ros_args


class WasdTeleopNode(Node):
    def __init__(
        self,
        topic: str,
        rate_hz: float,
        linear_speed: float,
        angular_speed: float,
        key_hold_timeout_s: float,
    ):
        super().__init__("wasd_teleop_node")
        self.pub = self.create_publisher(TwistStamped, topic, 20)

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.key_hold_timeout_s = key_hold_timeout_s

        self.vx = 0.0
        self.wz = 0.0
        self.last_logged_cmd = (None, None)
        self.quit_requested = False
        self.key_last_seen = {"w": -1e9, "a": -1e9, "s": -1e9, "d": -1e9}

        self.lock = threading.Lock()
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_cmd)

        self.get_logger().info(
            "WASD teleop publishing to %s at %.1f Hz" % (topic, rate_hz)
        )
        self.get_logger().info(
            "Controls: hold w/s/a/d to move, release to stop, x|space stop, q quit"
        )
        self.get_logger().info(
            "Terminal deadman timeout: %.2fs" % self.key_hold_timeout_s
        )

    def _recompute_cmd_locked(self, now: float):
        forward = (now - self.key_last_seen["w"]) <= self.key_hold_timeout_s
        reverse = (now - self.key_last_seen["s"]) <= self.key_hold_timeout_s
        left = (now - self.key_last_seen["a"]) <= self.key_hold_timeout_s
        right = (now - self.key_last_seen["d"]) <= self.key_hold_timeout_s

        if forward and not reverse:
            self.vx = -self.linear_speed
        elif reverse and not forward:
            self.vx = self.linear_speed
        else:
            self.vx = 0.0

        if left and not right:
            self.wz = -self.angular_speed
        elif right and not left:
            self.wz = self.angular_speed
        else:
            self.wz = 0.0

    def _log_cmd_if_changed_locked(self):
        cmd = (self.vx, self.wz)
        if cmd != self.last_logged_cmd:
            self.get_logger().info(
                "cmd -> linear.x=%.2f m/s, angular.z=%.2f rad/s"
                % (self.vx, self.wz)
            )
            self.last_logged_cmd = cmd

    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            self._recompute_cmd_locked(time.monotonic())
            self._log_cmd_if_changed_locked()
            msg.twist.linear.x = self.vx
            msg.twist.angular.z = self.wz
        self.pub.publish(msg)

    def _clear_motion_locked(self):
        self.key_last_seen["w"] = -1e9
        self.key_last_seen["a"] = -1e9
        self.key_last_seen["s"] = -1e9
        self.key_last_seen["d"] = -1e9
        self.vx = 0.0
        self.wz = 0.0

    def handle_key_event(self, key: str):
        now = time.monotonic()
        with self.lock:
            if key in ("w", "a", "s", "d"):
                self.key_last_seen[key] = now
                self._recompute_cmd_locked(now)
            elif key in ("x", " "):
                self._clear_motion_locked()
            elif key == "q":
                self._clear_motion_locked()
                self.quit_requested = True

            self._log_cmd_if_changed_locked()

class KeyboardReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.is_tty = sys.stdin.isatty()
        self.old_settings = termios.tcgetattr(self.fd) if self.is_tty else None

    def __enter__(self):
        if self.is_tty:
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.is_tty and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key_nonblocking(self, timeout_s: float = 0.05):
        if not self.is_tty:
            time.sleep(timeout_s)
            return None
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if ready:
            return sys.stdin.read(1)
        return None


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)

    node = WasdTeleopNode(
        topic=args.topic,
        rate_hz=args.rate_hz,
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        key_hold_timeout_s=args.key_hold_timeout_s,
    )

    print("\nWASD teleop ready. Hold w/a/s/d to move. Release key to stop. Press q to quit.\n")

    try:
        with KeyboardReader() as kb:
            if not kb.is_tty:
                print("stdin is not a TTY: keyboard input disabled, but ROS topic will still be published.")
            while rclpy.ok() and not node.quit_requested:
                rclpy.spin_once(node, timeout_sec=0.0)
                key = kb.read_key_nonblocking(timeout_s=0.05)
                if key is not None:
                    node.handle_key_event(key.lower())
    except KeyboardInterrupt:
        pass
    finally:
        with node.lock:
            node._clear_motion_locked()
        node.publish_cmd()
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)

        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
