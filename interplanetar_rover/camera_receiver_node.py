#!/usr/bin/env python3
"""
camera_receiver_node.py

ROS2 wrapper that launches the packaged camera receiver app
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


def _to_bool(text, default=False):
    raw = str(text).strip().lower()
    if raw in ('1', 'true', 'yes', 'on'):
        return True
    if raw in ('0', 'false', 'no', 'off'):
        return False
    return bool(default)


class CameraReceiverNode(Node):
    def __init__(self):
        super().__init__('camera_receiver_node')

        self.declare_parameter('python_executable', sys.executable)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('ports_csv', '5000,5001,5002,5003,5004,5005,5006,5007')
        self.declare_parameter('width', '1920')
        self.declare_parameter('height', '1080')
        self.declare_parameter('control_port', '7000')
        self.declare_parameter('rtsp_csv', '')
        self.declare_parameter('rtsp_latency_ports_csv', '')
        self.declare_parameter('restart_on_exit', 'false')

        self._proc = None
        self._launch_command = self._build_command()
        self.get_logger().info(f'Starting receiver subprocess: {" ".join(self._launch_command)}')
        try:
            self._proc = subprocess.Popen(self._launch_command)
        except Exception as exc:
            self.get_logger().fatal(f'Failed to start camera receiver: {exc}')
            raise SystemExit(1)

        self._restart_on_exit = _to_bool(
            self.get_parameter('restart_on_exit').value,
            default=False,
        )
        self.create_timer(1.0, self._monitor_subprocess)

    def _build_command(self):
        py_exec = str(self.get_parameter('python_executable').value).strip() or sys.executable
        host = str(self.get_parameter('host').value).strip() or '127.0.0.1'

        width = _to_int(self.get_parameter('width').value, 1920)
        height = _to_int(self.get_parameter('height').value, 1080)
        control_port = _to_int(self.get_parameter('control_port').value, 7000)

        ports = _parse_csv(self.get_parameter('ports_csv').value, cast=int)
        if not ports:
            ports = [5000, 5001, 5002, 5003, 5004, 5005, 5006, 5007]

        rtsp_urls = _parse_csv(self.get_parameter('rtsp_csv').value, cast=str)
        rtsp_latency_ports = _parse_csv(self.get_parameter('rtsp_latency_ports_csv').value, cast=int)

        cmd = [
            py_exec,
            '-m',
            'interplanetar_rover.camera_receiver_app',
            '-i', host,
            '-w', str(width),
            '-H', str(height),
            '--control-port', str(control_port),
            '-p',
        ]
        cmd.extend([str(p) for p in ports])

        if rtsp_urls:
            cmd.append('--rtsp')
            cmd.extend(rtsp_urls)

        if rtsp_latency_ports:
            cmd.append('--rtsp-latency-ports')
            cmd.extend([str(p) for p in rtsp_latency_ports])

        return cmd

    def _monitor_subprocess(self):
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is None:
            return

        self.get_logger().warn(f'camera_receiver.py exited with code {code}')
        self._proc = None

        if self._restart_on_exit:
            self.get_logger().info('Restarting camera receiver subprocess')
            try:
                self._proc = subprocess.Popen(self._launch_command)
            except Exception as exc:
                self.get_logger().error(f'Failed to restart camera receiver: {exc}')

    def destroy_node(self):
        if self._proc is not None and self._proc.poll() is None:
            self.get_logger().info('Stopping camera receiver subprocess')
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
