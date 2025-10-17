import sys
from enum import Enum, auto

from ..controller import TeleopController
from ..constants import (
    EXTEND_POSE,
    HOME_POSE,
    CUSTOM_POSE,
)
from ..errors import ValidationError


class Cmd(Enum):
    INC_LINEAR = auto()   # w
    DEC_LINEAR = auto()   # x
    INC_ANG = auto()      # a
    DEC_ANG = auto()      # d
    STOP = auto()         # s
    GRIP_OPEN = auto()    # g
    GRIP_CLOSE = auto()   # h
    ARM_EXTEND = auto()   # 0
    ARM_HOME = auto()     # 9
    ARM_CUSTOM = auto()   # 8
    QUIT = auto()         # q


KEYMAP = {
    "w": Cmd.INC_LINEAR,
    "x": Cmd.DEC_LINEAR,
    "a": Cmd.INC_ANG,
    "d": Cmd.DEC_ANG,
    "s": Cmd.STOP,
    "g": Cmd.GRIP_OPEN,
    "h": Cmd.GRIP_CLOSE,
    "0": Cmd.ARM_EXTEND,
    "9": Cmd.ARM_HOME,
    "8": Cmd.ARM_CUSTOM,
    "q": Cmd.QUIT,
}


MENU = """
---------------------------
Teleoperation Control of TurtleBot3 + OpenManipulatorX
---------------------------
w : increase linear velocity
x : decrease linear velocity
a : increase angular velocity
d : decrease angular velocity
s : base stop

g : gripper open
h : gripper close

0 : Extend arm forward
9 : Home pose
8 : Custom pose

q to quit
---------------------------
Present Linear Velocity: {lin:.3f}, Angular Velocity: {ang:.3f}
Present Arm Joint Angle J1: {J1:.3f} J2: {J2:.3f} J3: {J3:.3f} J4: {J4:.3f}
Present Base Position X: {x:.3f} Y: {y:.3f} Z: {z:.3f}
---------------------------
"""


def print_menu(ctrl: TeleopController):
    s = ctrl.state
    sys.stdout.write(MENU.format(
        lin=s.base.linear, ang=s.base.angular,
        J1=s.arm.J1, J2=s.arm.J2, J3=s.arm.J3, J4=s.arm.J4,
        x=s.base.x, y=s.base.y, z=s.base.z
    ))
    sys.stdout.flush()


def handle_key(ctrl: TeleopController, key: str):
    if key not in KEYMAP:
        raise ValidationError(f"Unknown input '{key}'. Valid keys: {', '.join(KEYMAP.keys())}")

    cmd = KEYMAP[key]
    if cmd == Cmd.INC_LINEAR:
        ctrl.inc_linear()
    elif cmd == Cmd.DEC_LINEAR:
        ctrl.dec_linear()
    elif cmd == Cmd.INC_ANG:
        ctrl.inc_ang()
    elif cmd == Cmd.DEC_ANG:
        ctrl.dec_ang()
    elif cmd == Cmd.STOP:
        ctrl.stop()
    elif cmd == Cmd.GRIP_OPEN:
        ctrl.gripper_open()
    elif cmd == Cmd.GRIP_CLOSE:
        ctrl.gripper_close()
    elif cmd == Cmd.ARM_EXTEND:
        ctrl.move_pose(EXTEND_POSE)
    elif cmd == Cmd.ARM_HOME:
        ctrl.move_pose(HOME_POSE)
    elif cmd == Cmd.ARM_CUSTOM:
        ctrl.move_pose(CUSTOM_POSE)
    elif cmd == Cmd.QUIT:
        raise SystemExit