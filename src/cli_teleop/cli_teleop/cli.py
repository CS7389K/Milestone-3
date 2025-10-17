import sys
import signal
import rclpy
import time
import threading

from enum import Enum, auto

from .controller import TeleopController
from .util.constants import (
    EXTEND_POSE,
    HOME_POSE,
    CUSTOM_POSE,
)
from .util.errors import ValidationError


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


class CLI():

    def __init__(self):
        self.controller = TeleopController()
        self.running = True

    def start(self):
        try:
            while rclpy.ok() and self.running:
                rclpy.spin_once(self.controller, timeout_sec=0.1)
                time.sleep(0.01)
        finally:
            self.controller.shutdown()
            rclpy.try_shutdown()

        self.print_menu()
        self.kb_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.kb_thread.start()

    def cli_loop(
            self,
            key: str
        ):
        while rclpy.ok() and self.running:
            key = sys.stdin.readline().strip().lower()
            if key not in KEYMAP:
                continue

            cmd = KEYMAP[key]
            if cmd == Cmd.INC_LINEAR:
                self.controller.inc_linear()
            elif cmd == Cmd.DEC_LINEAR:
                self.controller.dec_linear()
            elif cmd == Cmd.INC_ANG:
                self.controller.inc_ang()
            elif cmd == Cmd.DEC_ANG:
                self.controller.dec_ang()
            elif cmd == Cmd.STOP:
                self.controller.stop()
            elif cmd == Cmd.GRIP_OPEN:
                self.controller.gripper_open()
            elif cmd == Cmd.GRIP_CLOSE:
                self.controller.gripper_close()
            elif cmd == Cmd.ARM_EXTEND:
                self.controller.move_pose(EXTEND_POSE)
            elif cmd == Cmd.ARM_HOME:
                self.controller.move_pose(HOME_POSE)
            elif cmd == Cmd.ARM_CUSTOM:
                self.controller.move_pose(CUSTOM_POSE)
            elif cmd == Cmd.QUIT:
                raise SystemExit
        
    def print_menu(self):
        s = self.controller.state
        sys.stdout.write(MENU.format(
            lin=s.base.linear, ang=s.base.angular,
            J1=s.arm.J1, J2=s.arm.J2, J3=s.arm.J3, J4=s.arm.J4,
            x=s.base.x, y=s.base.y, z=s.base.z
        ))
        sys.stdout.flush()

    def shutdown(self):
        self.controller.get_logger().info('SIGINT received, shutting down...')
        self.controller.shutdown()
        self.running = False