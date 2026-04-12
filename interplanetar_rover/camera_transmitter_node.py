#!/usr/bin/env python3
"""
camera_transmitter_node.py

ROS2 wrapper that launches the packaged camera transmitter app
as a managed subprocess.
"""

import subprocess
import sys

import rclpy
from rclpy.node import Node


def _parse_csv(text, cast=str):
    out = []
    raw = str(text).strip()
    if not raw:
        return out
    for item in raw.split(','):
        token = item.strip()
        if not token:
            continue
        try:
            out.append(cast(token))
        except Exception:
            continue
    return out


def _to_int(text, default):
    try:
        return int(str(text).strip())
    except Exception:
        return int(default)


def _to_float(text, default):
    try:
        return float(str(text).strip())
    except Exception:
        return float(default)


def _to_bool(text, default=False):
    raw = str(text).strip().lower()
    if raw in ('1', 'true', 'yes', 'on'):
        return True
    if raw in ('0', 'false', 'no', 'off'):
        return False
    return bool(default)


class CameraTransmitterNode(Node):
    def __init__(self):
        super().__init__('camera_transmitter_node')

        self.declare_parameter('python_executable', sys.executable)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('cameras_csv', '')
        self.declare_parameter('base_port', '5000')
        self.declare_parameter('width', '640')
        self.declare_parameter('height', '480')
        self.declare_parameter('fps', '25')
        self.declare_parameter('bitrate', '1000000')
        self.declare_parameter('stagger', '1.5')
        self.declare_parameter('skip_probe', 'false')
        self.declare_parameter('control_port', '7000')
        self.declare_parameter('restart_on_exit', 'false')

        self._proc = None
        self._launch_command = self._build_command()
        self.get_logger().info(f'Starting transmitter subprocess: {" ".join(self._launch_command)}')
        try:
            self._proc = subprocess.Popen(self._launch_command)
        except Exception as exc:
            self.get_logger().fatal(f'Failed to start camera transmitter: {exc}')
            raise SystemExit(1)

        self._restart_on_exit = _to_bool(
            self.get_parameter('restart_on_exit').value,
            default=False,
        )
        self.create_timer(1.0, self._monitor_subprocess)

    def _build_command(self):
        py_exec = str(self.get_parameter('python_executable').value).strip() or sys.executable
        host = str(self.get_parameter('host').value).strip() or '127.0.0.1'

        base_port = _to_int(self.get_parameter('base_port').value, 5000)
        width = _to_int(self.get_parameter('width').value, 640)
        height = _to_int(self.get_parameter('height').value, 480)
        fps = _to_int(self.get_parameter('fps').value, 25)
        bitrate = _to_int(self.get_parameter('bitrate').value, 1000000)
        stagger = _to_float(self.get_parameter('stagger').value, 1.5)
        control_port = _to_int(self.get_parameter('control_port').value, 7000)
        skip_probe = _to_bool(self.get_parameter('skip_probe').value, default=False)
        cameras = _parse_csv(self.get_parameter('cameras_csv').value, cast=str)

        cmd = [
            py_exec,
            '-m',
            'interplanetar_rover.camera_transmitter_app',
            '-i', host,
            '-p', str(base_port),
            '-w', str(width),
            '-H', str(height),
            '-f', str(fps),
            '-b', str(bitrate),
            '--stagger', str(stagger),
            '--control-port', str(control_port),
        ]

        if cameras:
            cmd.append('-c')
            cmd.extend(cameras)

        if skip_probe:
            cmd.append('--skip-probe')

        return cmd

    def _monitor_subprocess(self):
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is None:
            return

        self.get_logger().warn(f'camera_transmitter.py exited with code {code}')
        self._proc = None

        if self._restart_on_exit:
            self.get_logger().info('Restarting camera transmitter subprocess')
            try:
                self._proc = subprocess.Popen(self._launch_command)
            except Exception as exc:
                self.get_logger().error(f'Failed to restart camera transmitter: {exc}')

    def destroy_node(self):
        if self._proc is not None and self._proc.poll() is None:
            self.get_logger().info('Stopping camera transmitter subprocess')
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTransmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
