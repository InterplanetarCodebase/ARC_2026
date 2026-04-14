#!/usr/bin/env python3
"""
odrive_cmd_vel.py
=================
ROS 2 Humble node that subscribes to geometry_msgs/TwistStamped and commands
dual ODrive controllers in differential-drive mode.

Topic:
  rover_controller/cmd_vel (geometry_msgs/msg/TwistStamped)

Drive mapping:
  v_left_mps  = v_x - (w_z * track_width / 2)
  v_right_mps = v_x + (w_z * track_width / 2)
  turns/s     = m/s / wheel_circumference

Motor polarity (matched to existing ws_to_diffdrive_ros.py behavior):
  odrv0.axis0 = FL (+left)
  odrv0.axis1 = RL (+left)
  odrv1.axis0 = FR (-right)  # physically inverted
  odrv1.axis1 = RR (-right)  # physically inverted
"""

from __future__ import annotations

import argparse
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

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


logging.basicConfig(
	level=logging.INFO,
	format="%(asctime)s [ODRIVE_CMD_VEL] %(levelname)s %(message)s",
)
log = logging.getLogger(__name__)


ODRV0_SERIAL = "336D33573235"
ODRV1_SERIAL = "335A33633235"

DEFAULT_TOPIC = "rover_controller/cmd_vel"
DEFAULT_WHEEL_DIAMETER_M = 0.20
DEFAULT_TRACK_WIDTH_M = 0.762
DEFAULT_MAX_WHEEL_TPS = 3.0
DEFAULT_VEL_RAMP_RATE = 20.0


def parse_args():
	parser = argparse.ArgumentParser(
		description="ROS2 TwistStamped to dual-ODrive differential drive bridge."
	)
	parser.add_argument("--topic", type=str, default=DEFAULT_TOPIC, help="TwistStamped topic name")
	parser.add_argument(
		"--odrv0_serial",
		type=str,
		default=ODRV0_SERIAL,
		help="Serial number for left-side ODrive (odrv0)",
	)
	parser.add_argument(
		"--odrv1_serial",
		type=str,
		default=ODRV1_SERIAL,
		help="Serial number for right-side ODrive (odrv1)",
	)
	parser.add_argument(
		"--wheel_diameter_m",
		type=float,
		default=DEFAULT_WHEEL_DIAMETER_M,
		help="Wheel diameter in meters",
	)
	parser.add_argument(
		"--track_width_m",
		type=float,
		default=DEFAULT_TRACK_WIDTH_M,
		help="Distance between left/right wheel tracks in meters",
	)
	parser.add_argument(
		"--max_wheel_tps",
		type=float,
		default=DEFAULT_MAX_WHEEL_TPS,
		help="Maximum wheel speed in turns/s sent to ODrive",
	)
	parser.add_argument(
		"--vel_ramp_rate",
		type=float,
		default=DEFAULT_VEL_RAMP_RATE,
		help="ODrive velocity ramp rate (turns/s^2)",
	)

	args, ros_args = parser.parse_known_args()

	if args.wheel_diameter_m <= 0.0:
		parser.error("--wheel_diameter_m must be > 0")
	if args.track_width_m <= 0.0:
		parser.error("--track_width_m must be > 0")
	if args.max_wheel_tps <= 0.0:
		parser.error("--max_wheel_tps must be > 0")
	if args.vel_ramp_rate <= 0.0:
		parser.error("--vel_ramp_rate must be > 0")

	return args, ros_args


def setup_axis(ax, name: str, vel_ramp_rate: float):
	log.info(f"[{name}] -> IDLE")
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
	ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP
	ax.controller.config.vel_ramp_rate = vel_ramp_rate
	ax.controller.input_vel = 0.0

	log.info(f"[{name}] -> CLOSED LOOP")
	ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
	time.sleep(0.6)
	log.info(f"[{name}] state={ax.current_state} error={ax.error}")


def connect_odrives(odrv0_serial: str, odrv1_serial: str, vel_ramp_rate: float):
	log.info("Connecting odrv0...")
	odrv0 = odrive.find_any(serial_number=odrv0_serial)
	time.sleep(1.0)

	log.info("Connecting odrv1...")
	odrv1 = odrive.find_any(serial_number=odrv1_serial)

	log.info(f"odrv0 serial: {odrv0.serial_number}")
	log.info(f"odrv1 serial: {odrv1.serial_number}")

	setup_axis(odrv0.axis0, "odrv0.axis0 FL", vel_ramp_rate)
	setup_axis(odrv0.axis1, "odrv0.axis1 RL", vel_ramp_rate)
	setup_axis(odrv1.axis0, "odrv1.axis0 FR", vel_ramp_rate)
	setup_axis(odrv1.axis1, "odrv1.axis1 RR", vel_ramp_rate)
	return odrv0, odrv1


def drive(odrv0, odrv1, left_tps: float, right_tps: float):
	odrv0.axis0.controller.input_vel = left_tps
	odrv0.axis1.controller.input_vel = left_tps
	odrv1.axis0.controller.input_vel = -right_tps
	odrv1.axis1.controller.input_vel = -right_tps


def stop(odrv0, odrv1) -> bool:
	ok = True
	motors = [
		("FL", lambda: setattr(odrv0.axis0.controller, "input_vel", 0.0)),
		("RL", lambda: setattr(odrv0.axis1.controller, "input_vel", 0.0)),
		("FR", lambda: setattr(odrv1.axis0.controller, "input_vel", 0.0)),
		("RR", lambda: setattr(odrv1.axis1.controller, "input_vel", 0.0)),
	]
	for label, fn in motors:
		try:
			fn()
		except Exception as exc:
			ok = False
			log.error(f"Failed to stop motor {label}: {exc}")
	return ok


def idle_all(odrv0, odrv1):
	for odrv in (odrv0, odrv1):
		try:
			odrv.axis0.requested_state = AXIS_STATE_IDLE
			odrv.axis1.requested_state = AXIS_STATE_IDLE
		except Exception:
			pass


class ODriveCmdVelNode(Node):
	def __init__(
		self,
		odrv0,
		odrv1,
		topic: str,
		wheel_diameter_m: float,
		track_width_m: float,
		max_wheel_tps: float,
	):
		super().__init__("odrive_cmd_vel_node")
		self.odrv0 = odrv0
		self.odrv1 = odrv1
		self.track_width_m = track_width_m
		self.max_wheel_tps = max_wheel_tps
		self.wheel_circumference_m = math.pi * wheel_diameter_m

		self.last_warn_monotonic = 0.0

		self.subscription = self.create_subscription(
			TwistStamped,
			topic,
			self.on_cmd_vel,
			20,
		)

		self.get_logger().info(
			"Listening on %s | wheel_diam=%.3fm track=%.3fm max_wheel_tps=%.2f"
			% (topic, wheel_diameter_m, track_width_m, max_wheel_tps)
		)

	def _clip_wheel_speeds(self, left_tps: float, right_tps: float):
		peak = max(abs(left_tps), abs(right_tps), 1e-9)
		if peak <= self.max_wheel_tps:
			return left_tps, right_tps

		scale = self.max_wheel_tps / peak
		now = time.monotonic()
		if now - self.last_warn_monotonic > 1.0:
			self.get_logger().warn(
				"Command saturated: scale=%.3f left=%.2f right=%.2f"
				% (scale, left_tps, right_tps)
			)
			self.last_warn_monotonic = now
		return left_tps * scale, right_tps * scale

	def _twist_to_wheel_tps(self, vx_mps: float, wz_radps: float):
		v_left_mps = vx_mps - (0.5 * self.track_width_m * wz_radps)
		v_right_mps = vx_mps + (0.5 * self.track_width_m * wz_radps)
		left_tps = v_left_mps / self.wheel_circumference_m
		right_tps = v_right_mps / self.wheel_circumference_m
		return self._clip_wheel_speeds(left_tps, right_tps)

	def on_cmd_vel(self, msg: TwistStamped):
		vx_mps = float(msg.twist.linear.x)
		wz_radps = float(msg.twist.angular.z)

		left_tps, right_tps = self._twist_to_wheel_tps(vx_mps, wz_radps)
		drive(self.odrv0, self.odrv1, left_tps, right_tps)


def main():
	args, ros_args = parse_args()

	log.info("=" * 70)
	log.info("ROS2 ODRIVE CMD_VEL CONFIG:")
	log.info(f"  topic: {args.topic}")
	log.info(f"  odrv0_serial: {args.odrv0_serial}")
	log.info(f"  odrv1_serial: {args.odrv1_serial}")
	log.info(f"  wheel_diameter_m: {args.wheel_diameter_m}")
	log.info(f"  track_width_m: {args.track_width_m}")
	log.info(f"  max_wheel_tps: {args.max_wheel_tps}")
	log.info(f"  vel_ramp_rate: {args.vel_ramp_rate}")
	log.info("=" * 70)

	odrv0 = None
	odrv1 = None
	node = None
	try:
		odrv0, odrv1 = connect_odrives(
			args.odrv0_serial,
			args.odrv1_serial,
			args.vel_ramp_rate,
		)

		rclpy.init(args=ros_args)
		node = ODriveCmdVelNode(
			odrv0=odrv0,
			odrv1=odrv1,
			topic=args.topic,
			wheel_diameter_m=args.wheel_diameter_m,
			track_width_m=args.track_width_m,
			max_wheel_tps=args.max_wheel_tps,
		)
		rclpy.spin(node)
	except KeyboardInterrupt:
		log.info("KeyboardInterrupt received, shutting down")
	finally:
		if node is not None:
			try:
				node.destroy_node()
			except Exception:
				pass

		if odrv0 is not None and odrv1 is not None:
			log.warning("Stopping and idling all motors")
			stop(odrv0, odrv1)
			time.sleep(0.4)
			idle_all(odrv0, odrv1)

		if rclpy.ok():
			rclpy.shutdown()


if __name__ == "__main__":
	main()
