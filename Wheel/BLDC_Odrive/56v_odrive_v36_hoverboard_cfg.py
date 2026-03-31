import argparse
import sys
import time

import odrive
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
                          AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION,
                          CONTROL_MODE_POSITION_CONTROL, ENCODER_MODE_HALL)


class HBMotorConfig:
    """Class for configuring an Odrive 3.6 56V axis for a Hoverboard motor."""

    HOVERBOARD_KV = 16.0

    def __init__(self, axis_num: int) -> None:
        """Init HBMotorConfig object."""
        self.axis_num = axis_num

        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")

    def _find_odrive(self) -> None:
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))

    def configure(self) -> None:
        """Configure and calibrate the ODrive for a hoverboard motor."""

        # Step 1: Erase existing configuration
        print("Erasing pre-existing configuration...")
        try:
            self.odrv.erase_configuration()
        except Exception:
            pass

        print("Waiting for ODrive to reboot after erase...")
        time.sleep(5)
        self._find_odrive()

        # Step 2: Apply motor configuration
        print("Applying motor configuration...")
        self.odrv_axis.motor.config.pole_pairs = 15
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        self.odrv_axis.motor.config.requested_current_range = 25
        self.odrv_axis.motor.config.current_control_bandwidth = 100
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV

        # Step 3: Apply encoder configuration
        print("Applying encoder configuration...")
        self.odrv_axis.encoder.config.mode = ENCODER_MODE_HALL
        self.odrv_axis.encoder.config.cpr = 90
        self.odrv_axis.encoder.config.calib_scan_distance = 150
        self.odrv_axis.encoder.config.bandwidth = 100

        # Step 4: Apply controller configuration
        print("Applying controller configuration...")
        self.odrv_axis.controller.config.pos_gain = 1
        self.odrv_axis.controller.config.vel_gain = (
            0.02
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_integrator_gain = (
            0.1
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_limit = 10
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # Step 5: Save and reboot
        print("Saving configuration and rebooting...")
        try:
            self.odrv.save_configuration()
        except Exception:
            pass

        print("Waiting for ODrive to reboot...")
        time.sleep(5)
        self._find_odrive()

        # Step 6: Motor calibration
        input("Make sure the motor is free to move, then press Enter...")
        print("Running motor calibration (you should hear a beep)...")
        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        # Wait and poll until motor calibration is done
        for _ in range(20):
            time.sleep(1)
            if self.odrv_axis.current_state == AXIS_STATE_IDLE:
                break
        else:
            print("Error: Motor calibration timed out.")
            sys.exit(1)

        if self.odrv_axis.motor.error != 0:
            print("Error during motor calibration: {}\n{}".format(
                self.odrv_axis.motor.error, self.odrv_axis.motor))
            sys.exit(1)

        print("Motor calibration successful.")
        self.odrv_axis.motor.config.pre_calibrated = True

        # Step 7: Encoder offset calibration
        # NOTE: No reboot between motor cal and encoder cal — must run in same session
        print("Running encoder offset calibration...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        # Wait and poll until encoder calibration is done
        for _ in range(60):
            time.sleep(1)
            if self.odrv_axis.current_state == AXIS_STATE_IDLE:
                break
        else:
            print("Error: Encoder offset calibration timed out.")
            sys.exit(1)

        if self.odrv_axis.encoder.error != 0:
            print("Error during encoder offset calibration: {}\n{}".format(
                self.odrv_axis.encoder.error, self.odrv_axis.encoder))
            sys.exit(1)

        print("Encoder offset calibration successful.")
        print("Encoder offset_float: {}".format(
            self.odrv_axis.encoder.config.offset_float))
        self.odrv_axis.encoder.config.pre_calibrated = True

        # Step 8: Save and reboot
        print("Saving calibration and rebooting...")
        try:
            self.odrv.save_configuration()
        except Exception:
            pass

        print("Waiting for ODrive to reboot...")
        time.sleep(5)
        self._find_odrive()

        print("ODrive configuration complete.")

    def mode_idle(self) -> None:
        """Put the motor in idle (moves freely)."""
        self.odrv_axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self) -> None:
        """Put the motor in closed loop control."""
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, angle: float) -> None:
        """Move the motor to a given angle in degrees."""
        self.odrv_axis.controller.input_pos = angle / 360.0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hoverboard Motor Calibration - ODrive 3.6 56V")

    parser.add_argument(
        "--axis_num",
        type=int,
        choices=[0, 1],
        required=True,
        help="Motor axis number (0 or 1).",
    )

    parser.add_argument(
        "--motor_test",
        action="store_true",
        help="After calibration, rotate motor in 30 degree increments from 0 to 360.",
    )

    args = parser.parse_args()

    hb_motor_config = HBMotorConfig(axis_num=args.axis_num)
    hb_motor_config.configure()

    if args.motor_test:
        print("Placing motor in closed loop control. Motor will resist movement.")
        hb_motor_config.mode_close_loop_control()

        print("CONDUCTING ROTATION TEST: 0 to 360 degrees in 30 degree steps.")
        for angle in range(0, 390, 30):
            print("Moving to {} degrees...".format(angle))
            hb_motor_config.move_input_pos(angle)
            time.sleep(5)

        print("Rotation test complete. Placing motor in idle.")
        hb_motor_config.mode_idle()