from dataclasses import dataclass

from .constants import (
    JOINT_LIMITS,
    LIN_STEP,
    MIN_LINEAR,
    MAX_LINEAR,
    ANG_STEP,
    MIN_ANGULAR,
    MAX_ANGULAR
)
from .errors import ValidationError


@dataclass
class BaseState:
    linear: float = 0.0
    angular: float = 0.0
    x: float = 0.0  # odom estimates — replace with live data if available
    y: float = 0.0
    z: float = 0.0


@dataclass
class ArmState:
    J1: float = 0.0
    J2: float = 0.0
    J3: float = 0.0
    J4: float = 0.0
    gripper_open: bool = True


@dataclass
class RobotState:
    base: BaseState = BaseState()
    arm: ArmState = ArmState()


class TeleopController:
    def __init__(self):
        self.state = RobotState()

    # --- Validation helpers ---
    def _clamp(self, val: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, val))

    def _validate_joint_pose(self, pose: dict):
        missing = [j for j in JOINT_LIMITS if j not in pose]
        if missing:
            raise ValidationError(f"Pose missing joints: {missing}")
        for j, (lo, hi) in JOINT_LIMITS.items():
            v = pose[j]
            if not (lo <= v <= hi):
                raise ValidationError(f"{j}={v:.3f} out of limits [{lo:.3f}, {hi:.3f}]")

    # --- Backend stubs (replace with ROS/SDK calls) ---
    def _send_cmd_vel(self, linear: float, angular: float):
        # Example: publish to /cmd_vel; raise BackendError on failure
        pass

    def _send_arm_pose(self, pose: dict):
        # Example: send JointTrajectory; raise BackendError on failure
        pass

    def _send_gripper(self, open_: bool):
        # Example: control gripper; raise BackendError on failure
        pass

    # --- Command handlers with error checks ---
    def inc_linear(self):
        before = self.state.base.linear
        self.state.base.linear = self._clamp(before + LIN_STEP, MIN_LINEAR, MAX_LINEAR)
        self._send_cmd_vel(self.state.base.linear, self.state.base.angular)

    def dec_linear(self):
        before = self.state.base.linear
        self.state.base.linear = self._clamp(before - LIN_STEP, MIN_LINEAR, MAX_LINEAR)
        self._send_cmd_vel(self.state.base.linear, self.state.base.angular)

    def inc_ang(self):
        before = self.state.base.angular
        self.state.base.angular = self._clamp(before + ANG_STEP, MIN_ANGULAR, MAX_ANGULAR)
        self._send_cmd_vel(self.state.base.linear, self.state.base.angular)

    def dec_ang(self):
        before = self.state.base.angular
        self.state.base.angular = self._clamp(before - ANG_STEP, MIN_ANGULAR, MAX_ANGULAR)
        self._send_cmd_vel(self.state.base.linear, self.state.base.angular)

    def stop(self):
        self.state.base.linear = 0.0
        self.state.base.angular = 0.0
        self._send_cmd_vel(0.0, 0.0)

    def gripper_open(self):
        if self.state.arm.gripper_open:
            # Not an error; just no-op
            return
        self._send_gripper(True)
        self.state.arm.gripper_open = True

    def gripper_close(self):
        if not self.state.arm.gripper_open:
            return
        self._send_gripper(False)
        self.state.arm.gripper_open = False

    def move_pose(self, pose: dict):
        self._validate_joint_pose(pose)
        self._send_arm_pose(pose)
        # If successful, update local state
        for j in ["J1", "J2", "J3", "J4"]:
            setattr(self.state.arm, j, pose[j])