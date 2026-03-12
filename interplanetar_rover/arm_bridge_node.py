#!/usr/bin/env python3
"""
arm_bridge_node.py — Jetson Xavier
Subscribes /arm_cmd (Int16MultiArray) → ESP32 USB serial
msg.data: [m1, m2, m3, m4a, m4b, servo_angle, motor_speed]
Sends 6 serial packets per message (one per motor/servo).
Packet: [0xAA][0xBB][SEQ_H][SEQ_L][CMD][VAL][CRC8]
"""
import time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int16MultiArray
import serial

SOF1, SOF2    = 0xAA, 0xBB
ACK_BYTE      = 0xAC
ACK_LEN       = 4
CMD_ESTOP     = 0xFF
CMD_HEARTBEAT = 0x00

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc<<1)^0x07)&0xFF if crc&0x80 else (crc<<1)&0xFF
    return crc

class ArmBridgeNode(Node):
    def __init__(self):
        super().__init__('arm_bridge_node')
        self.declare_parameter('serial_port',        '/dev/ttyUSB1')
        self.declare_parameter('baud_rate',           921600)
        self.declare_parameter('watchdog_timeout',    2.0)
        self.declare_parameter('heartbeat_interval',  0.2)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self._wd = self.get_parameter('watchdog_timeout').value
        hb   = self.get_parameter('heartbeat_interval').value

        self._seq  = 0
        self._lock = threading.Lock()
        self._last_msg   = time.time()
        self._wd_tripped = False
        self._ack_buf    = bytearray()

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'Serial: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().fatal(str(e)); raise SystemExit(1)

        threading.Thread(target=self._read_acks, daemon=True).start()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Int16MultiArray, '/arm_cmd', self._cb, qos)
        self.create_timer(hb, self._heartbeat)
        self.get_logger().info('ArmBridgeNode ready')

    def _next_seq(self):
        with self._lock:
            s = self._seq; self._seq = (self._seq+1)&0xFFFF
        return s

    def _send(self, cmd, val=0):
        seq  = self._next_seq()
        body = bytes([SOF1, SOF2, (seq>>8)&0xFF, seq&0xFF, cmd&0xFF, val&0xFF])
        try:
            self.ser.write(body + bytes([crc8(body)]))
        except serial.SerialException as e:
            self.get_logger().error(str(e), throttle_duration_sec=2.0)

    def _read_acks(self):
        while True:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if chunk:
                    self._ack_buf.extend(chunk)
                    while len(self._ack_buf) >= ACK_LEN:
                        if self._ack_buf[0] != ACK_BYTE:
                            self._ack_buf.pop(0); continue
                        ack = bytes(self._ack_buf[:ACK_LEN])
                        self._ack_buf = self._ack_buf[ACK_LEN:]
                        if ack[3] != 0x00:
                            self.get_logger().warn(
                                f'ACK seq={(ack[1]<<8)|ack[2]} status=0x{ack[3]:02X}')
            except Exception as e:
                self.get_logger().error(str(e), throttle_duration_sec=5.0)
                time.sleep(0.01)

    def _cb(self, msg):
        if len(msg.data) < 7:
            self.get_logger().warn(f'arm_cmd: expected 7 ints, got {len(msg.data)}')
            return
        self._last_msg   = time.time()
        self._wd_tripped = False
        m1,m2,m3,m4a,m4b,servo,speed = msg.data[:7]
        speed = max(0, min(255, speed))
        
        
        self.get_logger().info(
        f'Received /arm_cmd → m1:{m1}, m2:{m2}, m3:{m3}, m4a:{m4a}, m4b:{m4b}, servo:{servo}, speed:{speed}'
    	)
    	
        self._send(m1,  speed)
        self._send(m2,  speed)
        self._send(m3,  speed)
        self._send(m4a, speed)
        self._send(m4b, speed)
        self._send(0x60, max(0, min(180, servo)))  # CMD_SERVO_ANGLE

    def _heartbeat(self):
        if time.time() - self._last_msg > self._wd:
            if not self._wd_tripped:
                self.get_logger().warn('Watchdog: no /arm_cmd — ESTOP')
                self._wd_tripped = True
            self._send(CMD_ESTOP)
        else:
            self._send(CMD_HEARTBEAT)

    def destroy_node(self):
        self.get_logger().info('Shutdown: ESTOP to ESP32')
        self._send(CMD_ESTOP)
        time.sleep(0.05)
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
