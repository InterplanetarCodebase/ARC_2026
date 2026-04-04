#!/usr/bin/env python3
"""
WitMotion BWT901CL — Full ROS2 Publisher Node
Publishes: IMU, MagneticField, Temperature, PoseStamped, Odometry + TF
"""

import serial
import math

# ── pywitmotion parsers ──────────────────────────────────────────────────────
try:
    import pywitmotion as wit
except ImportError:
    raise SystemExit("❌  pywitmotion not found. Run: pip install pywitmotion")

# ── ROS2 ────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg       import Header
from sensor_msgs.msg    import Imu, MagneticField, Temperature
from geometry_msgs.msg  import PoseStamped, Vector3, TransformStamped
from nav_msgs.msg       import Odometry
from tf2_ros            import TransformBroadcaster


# ════════════════════════════════════════════════════════════════════════════
#  Helpers
# ════════════════════════════════════════════════════════════════════════════

DEG2RAD = math.pi / 180.0
G_TO_MS2 = 9.81          # 1 g  →  m/s²
UT_TO_T  = 1e-6          # µT   →  Tesla


def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """ZYX Euler (degrees) → quaternion (x, y, z, w)."""
    r = roll_deg  * DEG2RAD
    p = pitch_deg * DEG2RAD
    y = yaw_deg   * DEG2RAD

    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,   # qx
        cr * sp * cy + sr * cp * sy,   # qy
        cr * cp * sy - sr * sp * cy,   # qz
        cr * cp * cy + sr * sp * sy,   # qw
    )


def remove_gravity(ax_g, ay_g, az_g, roll_deg, pitch_deg):
    """Subtract gravity vector (rotated to sensor frame). Returns m/s²."""
    r = roll_deg  * DEG2RAD
    p = pitch_deg * DEG2RAD
    gx =  -math.sin(p)
    gy =   math.sin(r) * math.cos(p)
    gz =   math.cos(r) * math.cos(p)
    return (
        (ax_g - gx) * G_TO_MS2,
        (ay_g - gy) * G_TO_MS2,
        (az_g - gz) * G_TO_MS2,
    )


def rotate_to_world(ax, ay, az, roll_deg, pitch_deg, yaw_deg):
    """Rotate linear acceleration from sensor frame → world frame."""
    r = roll_deg  * DEG2RAD
    p = pitch_deg * DEG2RAD
    y = yaw_deg   * DEG2RAD

    wx = (math.cos(y)*math.cos(p)) * ax \
       + (math.cos(y)*math.sin(p)*math.sin(r) - math.sin(y)*math.cos(r)) * ay \
       + (math.cos(y)*math.sin(p)*math.cos(r) + math.sin(y)*math.sin(r)) * az

    wy = (math.sin(y)*math.cos(p)) * ax \
       + (math.sin(y)*math.sin(p)*math.sin(r) + math.cos(y)*math.cos(r)) * ay \
       + (math.sin(y)*math.sin(p)*math.cos(r) - math.cos(y)*math.sin(r)) * az

    wz = (-math.sin(p)) * ax \
       + (math.cos(p)*math.sin(r)) * ay \
       + (math.cos(p)*math.cos(r)) * az

    return wx, wy, wz


# ════════════════════════════════════════════════════════════════════════════
#  ROS2 Node
# ════════════════════════════════════════════════════════════════════════════

class BWT901CLNode(Node):

    # ── Tuning knobs ─────────────────────────────────────────────────────────
    ACCEL_DEADBAND   = 0.05   # m/s²  — zero out tiny accelerations at rest
    PRINT_EVERY_N    = 20     # print to terminal every N packets (~2 Hz @ 10 Hz sensor)

    def __init__(self):
        super().__init__('bwt901cl_imu_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('port',       '/dev/ttyUSB0')
        self.declare_parameter('baud',        115200)
        self.declare_parameter('imu_frame',  'imu_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        port            = self.get_parameter('port').value
        baud            = self.get_parameter('baud').value
        self.imu_frame  = self.get_parameter('imu_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ── QoS ──────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        # Standard sensor_msgs/Imu  (accel + gyro + orientation)
        self.imu_pub  = self.create_publisher(Imu,           'imu/data',        qos)
        # Magnetometer
        self.mag_pub  = self.create_publisher(MagneticField, 'imu/mag',         qos)
        # Temperature
        self.tmp_pub  = self.create_publisher(Temperature,   'imu/temperature', qos)
        # PoseStamped — orientation only, useful for quick RViz2 check
        self.pose_pub = self.create_publisher(PoseStamped,   'imu/pose',        qos)
        # Odometry — dead-reckoning position
        self.odom_pub = self.create_publisher(Odometry,      'imu/odom',        qos)

        # TF broadcaster
        self.tf_br = TransformBroadcaster(self)

        # ── Serial ───────────────────────────────────────────────────────────
        try:
            self.ser    = serial.Serial(port, baud, timeout=1)
            self.buffer = b''
            self.get_logger().info(f"✅  Serial opened  →  {port}  @  {baud} baud")
        except serial.SerialException as e:
            self.get_logger().fatal(f"❌  Cannot open serial port: {e}")
            raise SystemExit(1)

        # ── Sensor state ─────────────────────────────────────────────────────
        # Latest parsed values (raw)
        self.angle   = None   # (roll, pitch, yaw)  degrees
        self.accel   = None   # (ax, ay, az)         g
        self.gyro    = None   # (gx, gy, gz)         deg/s
        self.mag     = None   # (mx, my, mz)         µT
        self.quat    = None   # (q0, q1, q2, q3)     unit quaternion
        self.temp    = None   # float                 °C

        # Zeroing offsets (captured at startup)
        self.angle_offsets = None

        # Dead-reckoning state
        self.vx = self.vy = self.vz = 0.0
        self.px = self.py = self.pz = 0.0
        self.last_time = None

        # Terminal print counter
        self._print_cnt = 0

        # ── Timer: poll serial @ 100 Hz ──────────────────────────────────────
        self.timer = self.create_timer(0.01, self._read_and_publish)

        print("\n" + "="*60)
        print("  WitMotion BWT901CL  —  ROS2 Node  STARTED")
        print("="*60)
        print(f"  Port        : {port}  @  {baud}")
        print(f"  IMU frame   : {self.imu_frame}")
        print(f"  Odom frame  : {self.odom_frame}")
        print(f"  Base frame  : {self.base_frame}")
        print("-"*60)
        print("  Topics published:")
        print("    /imu/data          sensor_msgs/Imu")
        print("    /imu/mag           sensor_msgs/MagneticField")
        print("    /imu/temperature   sensor_msgs/Temperature")
        print("    /imu/pose          geometry_msgs/PoseStamped")
        print("    /imu/odom          nav_msgs/Odometry")
        print("    TF: odom → base_link")
        print("="*60)
        print("  ⏳  Waiting for first valid packet … keep sensor still!\n")

    # ── Serial read + parse ──────────────────────────────────────────────────

    def _read_and_publish(self):
        """Called at 100 Hz. Drain serial buffer, parse all complete packets."""
        try:
            self.buffer += self.ser.read(64)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            return

        packets      = self.buffer.split(b'\x55')
        self.buffer  = packets[-1]          # incomplete tail — keep for next call

        for pkt in packets[:-1]:
            self._parse_packet(pkt)

    def _parse_packet(self, pkt: bytes):
        """Try every pywitmotion parser on the raw packet."""
        # ── Angle ──────────────────────────────────────────────
        try:
            v = wit.get_angle(pkt)
            if v is not None:
                self.angle = v
        except Exception:
            pass

        # ── Acceleration (g) ───────────────────────────────────
        try:
            v = wit.get_acceleration(pkt)
            if v is not None:
                self.accel = v
        except Exception:
            pass

        # ── Gyroscope (deg/s) ──────────────────────────────────
        try:
            v = wit.get_gyro(pkt)
            if v is not None:
                self.gyro = v
        except Exception:
            pass

        # ── Magnetometer (µT) ──────────────────────────────────
        try:
            v = wit.get_magnetic(pkt)
            if v is not None:
                self.mag = v
        except Exception:
            pass

        # ── Quaternion ─────────────────────────────────────────
        try:
            v = wit.get_quaternion(pkt)
            if v is not None:
                self.quat = v
        except Exception:
            pass

        # ── Temperature (°C) ──────────────────────────────────
        try:
            v = wit.get_temperature(pkt)
            if v is not None:
                self.temp = v
        except Exception:
            pass

        # ── Publish once we have at minimum angle + accel ─────
        if self.angle is not None and self.accel is not None:
            self._handle_full_update()

    # ── Core publish logic ───────────────────────────────────────────────────

    def _handle_full_update(self):
        curr_roll, curr_pitch, curr_yaw = self.angle

        # ── First-packet zeroing ─────────────────────────────────────────────
        if self.angle_offsets is None:
            self.angle_offsets = (curr_roll, curr_pitch, curr_yaw)
            self.last_time     = self.get_clock().now()

            print("┌─────────────────────────────────────────────┐")
            print("│  ✅  ZERO POINT CAPTURED                    │")
            print(f"│  Roll : {curr_roll:8.3f}°                          │")
            print(f"│  Pitch: {curr_pitch:8.3f}°                          │")
            print(f"│  Yaw  : {curr_yaw:8.3f}°                          │")
            print("└─────────────────────────────────────────────┘\n")
            return

        # ── Relative angles ──────────────────────────────────────────────────
        rel_roll  = curr_roll  - self.angle_offsets[0]
        rel_pitch = curr_pitch - self.angle_offsets[1]
        rel_yaw   = curr_yaw   - self.angle_offsets[2]

        # ── dt ───────────────────────────────────────────────────────────────
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0 or dt > 1.0:        # skip bad dt (e.g. first frame)
            return

        stamp = now.to_msg()

        # ── Orientation quaternion ────────────────────────────────────────────
        # Prefer on-sensor quaternion (9-axis AHRS) if available
        if self.quat is not None:
            q0, q1, q2, q3 = self.quat   # wit convention: q0=w, q1=x, q2=y, q3=z
            qx, qy, qz, qw = q1, q2, q3, q0
        else:
            qx, qy, qz, qw = euler_to_quaternion(rel_roll, rel_pitch, rel_yaw)

        # ── Dead-reckoning ───────────────────────────────────────────────────
        ax_g, ay_g, az_g = self.accel
        ax, ay, az = remove_gravity(ax_g, ay_g, az_g, rel_roll, rel_pitch)

        # Apply deadband
        ax = 0.0 if abs(ax) < self.ACCEL_DEADBAND else ax
        ay = 0.0 if abs(ay) < self.ACCEL_DEADBAND else ay
        az = 0.0 if abs(az) < self.ACCEL_DEADBAND else az

        wx, wy, wz = rotate_to_world(ax, ay, az, rel_roll, rel_pitch, rel_yaw)

        self.vx += wx * dt;  self.px += self.vx * dt
        self.vy += wy * dt;  self.py += self.vy * dt
        self.vz += wz * dt;  self.pz += self.vz * dt

        # ════════════════════════════════════════════════════════════════════
        #  1. sensor_msgs/Imu  →  /imu/data
        # ════════════════════════════════════════════════════════════════════
        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = self.imu_frame

        # Orientation (from 9-axis AHRS)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance[0] = 1e-4   # diagonal: xx
        imu_msg.orientation_covariance[4] = 1e-4   #           yy
        imu_msg.orientation_covariance[8] = 1e-4   #           zz

        # Angular velocity (deg/s → rad/s)
        if self.gyro is not None:
            gx_deg, gy_deg, gz_deg = self.gyro
            imu_msg.angular_velocity.x = gx_deg * DEG2RAD
            imu_msg.angular_velocity.y = gy_deg * DEG2RAD
            imu_msg.angular_velocity.z = gz_deg * DEG2RAD
        imu_msg.angular_velocity_covariance[0] = 1e-5
        imu_msg.angular_velocity_covariance[4] = 1e-5
        imu_msg.angular_velocity_covariance[8] = 1e-5

        # Linear acceleration (g → m/s²) — RAW (gravity included)
        imu_msg.linear_acceleration.x = ax_g * G_TO_MS2
        imu_msg.linear_acceleration.y = ay_g * G_TO_MS2
        imu_msg.linear_acceleration.z = az_g * G_TO_MS2
        imu_msg.linear_acceleration_covariance[0] = 1e-4
        imu_msg.linear_acceleration_covariance[4] = 1e-4
        imu_msg.linear_acceleration_covariance[8] = 1e-4

        self.imu_pub.publish(imu_msg)

        # ════════════════════════════════════════════════════════════════════
        #  2. sensor_msgs/MagneticField  →  /imu/mag
        # ════════════════════════════════════════════════════════════════════
        if self.mag is not None:
            mx_ut, my_ut, mz_ut = self.mag
            mag_msg = MagneticField()
            mag_msg.header.stamp    = stamp
            mag_msg.header.frame_id = self.imu_frame
            mag_msg.magnetic_field.x = mx_ut * UT_TO_T
            mag_msg.magnetic_field.y = my_ut * UT_TO_T
            mag_msg.magnetic_field.z = mz_ut * UT_TO_T
            mag_msg.magnetic_field_covariance[0] = 1e-7
            mag_msg.magnetic_field_covariance[4] = 1e-7
            mag_msg.magnetic_field_covariance[8] = 1e-7
            self.mag_pub.publish(mag_msg)

        # ════════════════════════════════════════════════════════════════════
        #  3. sensor_msgs/Temperature  →  /imu/temperature
        # ════════════════════════════════════════════════════════════════════
        if self.temp is not None:
            tmp_msg = Temperature()
            tmp_msg.header.stamp    = stamp
            tmp_msg.header.frame_id = self.imu_frame
            tmp_msg.temperature     = float(self.temp)
            tmp_msg.variance        = 0.01
            self.tmp_pub.publish(tmp_msg)

        # ════════════════════════════════════════════════════════════════════
        #  4. geometry_msgs/PoseStamped  →  /imu/pose
        # ════════════════════════════════════════════════════════════════════
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = self.px
        pose_msg.pose.position.y = self.py
        pose_msg.pose.position.z = self.pz
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        # ════════════════════════════════════════════════════════════════════
        #  5. nav_msgs/Odometry  →  /imu/odom
        # ════════════════════════════════════════════════════════════════════
        odom_msg = Odometry()
        odom_msg.header.stamp    = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id  = self.base_frame

        odom_msg.pose.pose.position.x    = self.px
        odom_msg.pose.pose.position.y    = self.py
        odom_msg.pose.pose.position.z    = self.pz
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear  = Vector3(x=self.vx, y=self.vy, z=self.vz)
        if self.gyro is not None:
            gx_deg, gy_deg, gz_deg = self.gyro
            odom_msg.twist.twist.angular = Vector3(
                x = gx_deg * DEG2RAD,
                y = gy_deg * DEG2RAD,
                z = gz_deg * DEG2RAD,
            )
        self.odom_pub.publish(odom_msg)

        # ════════════════════════════════════════════════════════════════════
        #  6. TF  odom → base_link
        # ════════════════════════════════════════════════════════════════════
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = self.px
        tf.transform.translation.y = self.py
        tf.transform.translation.z = self.pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

        # ════════════════════════════════════════════════════════════════════
        #  Terminal output  (every N packets)
        # ════════════════════════════════════════════════════════════════════
        self._print_cnt += 1
        if self._print_cnt >= self.PRINT_EVERY_N:
            self._print_cnt = 0
            self._print_status(
                rel_roll, rel_pitch, rel_yaw,
                ax_g, ay_g, az_g,
                wx, wy, wz,
            )

    def _print_status(self, roll, pitch, yaw, ax_g, ay_g, az_g, wx, wy, wz):
        gx_str = gy_str = gz_str = "  N/A  "
        # Change "if self.gyro:" to "if self.gyro is not None:"
        if self.gyro is not None:
            gx_str = f"{self.gyro[0]:7.2f}"
            gy_str = f"{self.gyro[1]:7.2f}"
            gz_str = f"{self.gyro[2]:7.2f}"

        mx_str = my_str = mz_str = "  N/A  "
        # Change "if self.mag:" to "if self.mag is not None:"
        if self.mag is not None:
            mx_str = f"{self.mag[0]:7.1f}"
            my_str = f"{self.mag[1]:7.1f}"
            mz_str = f"{self.mag[2]:7.1f}"

        tmp_str = f"{self.temp:.1f} °C" if self.temp is not None else "N/A"

        qw_str = "N/A"
        # Change "if self.quat:" to "if self.quat is not None:"
        if self.quat is not None:
            qw_str = (f"w={self.quat[0]:.3f}  x={self.quat[1]:.3f}  "
                    f"y={self.quat[2]:.3f}  z={self.quat[3]:.3f}")
    
    # ... rest of the print function

        print(
            f"┌── BWT901CL ─────────────────────────────────────────────────────\n"
            f"│ ANGLE   R:{roll:7.2f}°   P:{pitch:7.2f}°   Y:{yaw:7.2f}°\n"
            f"│ ACCEL   X:{ax_g:7.3f}g  Y:{ay_g:7.3f}g  Z:{az_g:7.3f}g  "
            f"(world: {wx:6.3f} {wy:6.3f} {wz:6.3f} m/s²)\n"
            f"│ GYRO    X:{gx_str}°/s  Y:{gy_str}°/s  Z:{gz_str}°/s\n"
            f"│ MAG     X:{mx_str}µT  Y:{my_str}µT  Z:{mz_str}µT\n"
            f"│ QUAT    {qw_str}\n"
            f"│ TEMP    {tmp_str}\n"
            f"│ ODOM    pos({self.px:.3f}, {self.py:.3f}, {self.pz:.3f}) m   "
            f"vel({self.vx:.3f}, {self.vy:.3f}, {self.vz:.3f}) m/s\n"
            f"└────────────────────────────────────────────────────────────────"
        )

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("\n🔌  Serial port closed.")
        super().destroy_node()


# ════════════════════════════════════════════════════════════════════════════
#  Entry point
# ════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = BWT901CLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⛔  Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()