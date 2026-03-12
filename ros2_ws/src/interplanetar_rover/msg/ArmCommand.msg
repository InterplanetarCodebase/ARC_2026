# ArmCommand.msg
# Discrete motor states + servo angle for the rover arm.
# Each motor field: -1 = reverse, 0 = stop, +1 = forward
# Direction semantics per motor are defined in arm_bridge_node.py.

std_msgs/Header header

int8 base          # -1=CCW   0=STOP  +1=CW
int8 shoulder      # -1=Down  0=STOP  +1=Up
int8 elbow         # -1=Down  0=STOP  +1=Up
int8 roller        # -1=CCW   0=STOP  +1=CW
int8 gripper       # -1=Close 0=STOP  +1=Open
uint8 wrist_angle  # 0-180 degrees
