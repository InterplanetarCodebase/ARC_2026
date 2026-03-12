#!/usr/bin/env python3
"""
wheel_bridge_node.py — Jetson Xavier
Subscribes /cmd_vel (Twist) → Arduino Nano USB serial
Packet: [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]
"""
import struct, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
import serial

SOF1, SOF2 = 0xAA, 0xBB

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc<<1)^0x07)&0xFF if crc&0x80 else (crc<<1)&0xFF
    return crc

class WheelBridgeNode(Node):
    def __init__(self):
        super().__init__('wheel_bridge_node')
        self.declare_parameter('serial_port',      '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',         115200)
        self.declare_parameter('watchdog_timeout',  2.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self._wd  = self.get_parameter('watchdog_timeout').value
        self._seq = 0
        self._lock = threading.Lock()
        self._last_msg = time.time()
        self._wd_tripped = False

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'Serial: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(str(e)); raise SystemExit(1)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Twist, '/cmd_vel', self._cb, qos)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info('WheelBridgeNode ready')

    def _next_seq(self):
        with self._lock:
            s = self._seq; self._seq = (self._seq+1)&0xFFFF
        return s

    def _send(self, fwd, turn):
        x = int(max(-127, min(127, fwd  * 127)))
        z = int(max(-127, min(127, turn * 127)))
        seq = self._next_seq()
        body = bytes([SOF1, SOF2, (seq>>8)&0xFF, seq&0xFF,
                      struct.pack('b',x)[0], struct.pack('b',z)[0], 0xFF])
        try:
            self.ser.write(body + bytes([crc8(body)]))
        except serial.SerialException as e:
            self.get_logger().error(str(e), throttle_duration_sec=2.0)

    def _cb(self, msg):
        self._last_msg = time.time()
        self._wd_tripped = False
        self._send(msg.linear.x, msg.angular.z)

    def _watchdog(self):
        if time.time() - self._last_msg > self._wd:
            if not self._wd_tripped:
                self.get_logger().warn('Watchdog: no /cmd_vel — stopping wheels')
                self._wd_tripped = True
            self._send(0.0, 0.0)

    def destroy_node(self):
        self.get_logger().info('Shutdown: stopping wheels')
        self._send(0.0, 0.0)
        time.sleep(0.05)
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WheelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()