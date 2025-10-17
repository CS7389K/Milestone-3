#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Keyboard teleoperation for a mobile manipulator (ROS 2).
- Publishes base Twist
- Sends joint jogs to MoveIt Servo (JointJog)
- Controls gripper via GripperCommand action
- Subscribes to /joint_states and /odom
- Renders a non-scrolling terminal UI (10 Hz)
- Cross-platform keyboard input (Windows: msvcrt; Linux/Mac: termios+select)

If your project provides util.constants with ROS topic/service names and limits,
those will be used. Otherwise, fallback defaults below are applied.
"""

import sys, threading, time, math, platform
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger

# ========= Fallback constants (used if util.constants is not found) =========
FALLBACK_CONSTANTS = dict(
    CONTROLLER_NAME="teleop_keyboard",
    BASE_TWIST_TOPIC="/cmd_vel",
    ARM_JOINT_TOPIC="/servo_server/joint_jog",
    GRIPPER_ACTION="/gripper_controller/gripper_cmd",
    ROS_QUEUE_SIZE=10,
    BASE_LINEAR_VEL_MAX=0.4,
    BASE_LINEAR_VEL_STEP=0.02,
    BASE_ANGULAR_VEL_MAX=1.2,
    BASE_ANGULAR_VEL_STEP=0.06,
    BASE_FRAME_ID="base_link",
    SERVO_START_SRV="/servo_server/start",
    SERVO_STOP_SRV="/servo_server/stop",
    # Example poses (rad). Replace joint names/targets to match your robot.
    POSES={
        "home":   {"joint1": 0.0,  "joint2": 0.0,  "joint3": 0.0, "joint4": 0.0},
        "extend": {"joint1": 0.0,  "joint2": -0.7, "joint3": 1.0, "joint4": 0.0},
        "custom": {"joint1": 0.4,  "joint2": -0.9, "joint3": 0.8, "joint4": 0.0},
    }
)

try:
    # Your project's constants (override the fallbacks if available)
    from util.constants import (
        CONTROLLER_NAME,
        BASE_TWIST_TOPIC,
        ARM_JOINT_TOPIC,
        GRIPPER_ACTION,
        ROS_QUEUE_SIZE,
        BASE_LINEAR_VEL_MAX,
        BASE_LINEAR_VEL_STEP,
        BASE_ANGULAR_VEL_MAX,
        BASE_ANGULAR_VEL_STEP,
        BASE_FRAME_ID,
        SERVO_START_SRV,
        SERVO_STOP_SRV,
        POSES,
    )
except Exception:
    CONTROLLER_NAME = FALLBACK_CONSTANTS["CONTROLLER_NAME"]
    BASE_TWIST_TOPIC = FALLBACK_CONSTANTS["BASE_TWIST_TOPIC"]
    ARM_JOINT_TOPIC = FALLBACK_CONSTANTS["ARM_JOINT_TOPIC"]
    GRIPPER_ACTION = FALLBACK_CONSTANTS["GRIPPER_ACTION"]
    ROS_QUEUE_SIZE = FALLBACK_CONSTANTS["ROS_QUEUE_SIZE"]
    BASE_LINEAR_VEL_MAX = FALLBACK_CONSTANTS["BASE_LINEAR_VEL_MAX"]
    BASE_LINEAR_VEL_STEP = FALLBACK_CONSTANTS["BASE_LINEAR_VEL_STEP"]
    BASE_ANGULAR_VEL_MAX = FALLBACK_CONSTANTS["BASE_ANGULAR_VEL_MAX"]
    BASE_ANGULAR_VEL_STEP = FALLBACK_CONSTANTS["BASE_ANGULAR_VEL_STEP"]
    BASE_FRAME_ID = FALLBACK_CONSTANTS["BASE_FRAME_ID"]
    SERVO_START_SRV = FALLBACK_CONSTANTS["SERVO_START_SRV"]
    SERVO_STOP_SRV = FALLBACK_CONSTANTS["SERVO_STOP_SRV"]
    POSES = FALLBACK_CONSTANTS["POSES"]


# ========= Cross-platform keyboard reader =========
class KeyboardThread(threading.Thread):
    """
    Non-blocking keyboard input reader.
    - Windows: msvcrt.getwch()
    - POSIX (Linux/Mac): termios + select (works over SSH)
    """
    def __init__(self):
        super().__init__(daemon=True)
        self._last_key: Optional[str] = None
        self._stop = False
        self._lock = threading.Lock()

    def run(self):
        system = platform.system().lower()
        if "windows" in system:
            self._run_windows()
        else:
            self._run_posix()

    def stop(self):
        self._stop = True

    def _run_windows(self):
        import msvcrt
        while not self._stop:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                with self._lock:
                    self._last_key = ch
            time.sleep(0.01)

    def _run_posix(self):
        import sys, select, termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop:
                dr, _, _ = select.select([sys.stdin], [], [], 0.05)
                if dr:
                    ch = sys.stdin.read(1)
                    with self._lock:
                        self._last_key = ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def pop_key(self) -> Optional[str]:
        with self._lock:
            k = self._last_key
            self._last_key = None
            return k


# ========= Main teleop node =========
class TeleopController(Node):
    """
    Keyboard teleoperation node:
      - base Twist publishing
      - arm joint jog via MoveIt Servo (JointJog)
      - gripper control via GripperCommand action
      - subscriptions to /joint_states and /odom
      - non-scrolling terminal UI
    """
    def __init__(self):
        super().__init__(CONTROLLER_NAME)

        # Publishers / Action clients / Service clients
        self.base_twist_pub = self.create_publisher(Twist, BASE_TWIST_TOPIC, ROS_QUEUE_SIZE)
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, ROS_QUEUE_SIZE)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self.servo_start_client = self.create_client(Trigger, SERVO_START_SRV)
        self.servo_stop_client  = self.create_client(Trigger, SERVO_STOP_SRV)

        # Subscriptions
        self.create_subscription(JointState, "/joint_states", self.cb_joint_states, 10)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 10)

        # Timers
        self.pub_timer = self.create_timer(0.01, self.publish_loop)  # 100 Hz
        self.ui_timer  = self.create_timer(0.10, self.ui_loop)       # 10 Hz

        # State
        self.cmd_vel = Twist()
        self.publish_joint_pending = False
        self.pending_jointjog: Optional[JointJog] = None

        self.current_joints: Dict[str, float] = {}
        self.odom_pose = dict(x=0.0, y=0.0, yaw=0.0)

        self.last_gripper_cmd: Optional[float] = None
        self._servo_started = False

        # Init MoveIt Servo services and gripper action
        self.connect_moveit_servo()
        self.start_moveit_servo()
        self.gripper_client.wait_for_server(timeout_sec=3.0)

    # ----- Subscriptions -----
    def cb_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = float(pos)

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation  # quaternion
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_pose.update(x=float(x), y=float(y), yaw=float(yaw))

    # ----- MoveIt Servo start/stop -----
    def connect_moveit_servo(self):
        ok1 = self.servo_start_client.wait_for_service(timeout_sec=5.0)
        ok2 = self.servo_stop_client.wait_for_service(timeout_sec=5.0)
        if ok1 and ok2:
            self.get_logger().info("MoveIt Servo services are available.")
        else:
            self.get_logger().warn("MoveIt Servo services NOT available yet.")

    def start_moveit_servo(self):
        self.get_logger().info("Calling moveit_servo START ...")
        if not self.servo_start_client.service_is_ready():
            self.get_logger().warn("start_servo not ready; skip.")
            return
        future = self.servo_start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self._servo_started = bool(future.done() and future.result())
        if self._servo_started:
            self.get_logger().info("MoveIt Servo STARTED.")
        else:
            self.get_logger().error("MoveIt Servo START failed.")

    def stop_moveit_servo(self):
        if not self.servo_stop_client.service_is_ready():
            return
        self.get_logger().info("Calling moveit_servo STOP ...")
        future = self.servo_stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    # ----- Publishing loop -----
    def publish_loop(self):
        # (1) Publish pending JointJog first
        if self.publish_joint_pending and self.pending_jointjog is not None:
            msg = self.pending_jointjog
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = BASE_FRAME_ID
            self.joint_pub.publish(msg)
            self.publish_joint_pending = False

        # (2) Publish base Twist
        self.base_twist_pub.publish(self.cmd_vel)

    # ----- UI (non-scrolling) -----
    def ui_loop(self):
        # ANSI clear-screen; on Windows 10+ this generally works in default terminals.
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(self._ui_text())
        sys.stdout.flush()

    def _ui_text(self) -> str:
        lin = self.cmd_vel.linear.x
        ang = self.cmd_vel.angular.z
        jtxt = " | ".join([f"{k}:{self.current_joints.get(k, float('nan')): .2f}"
                           for k in sorted(self.current_joints.keys())[:8]])
        pose = self.odom_pose
        gr = self.last_gripper_cmd
        lines = [
            "=== TELEOP: Base + Arm + Gripper (Keyboard) ===",
            f"MoveIt Servo started: {self._servo_started}",
            "",
            "KEYS:",
            "  w/x : +linear / -linear       a/d : +angular / -angular     s: stop",
            "  g/h : gripper open/close      0/9/8 : Home/Extend/Custom    q: quit",
            "",
            f"Current Vel: linear_x={lin:+.2f}  angular_z={ang:+.2f}",
            f"Joint State: {jtxt if jtxt else '(no joint_states yet)'}",
            f"Odom Pose  : x={pose['x']:+.2f}  y={pose['y']:+.2f}  yaw={pose['yaw']:+.2f} rad",
            f"GripperCmd : {gr if gr is not None else '(none)'}",
            "",
            "------------------------------------------------",
        ]
        return "\n".join(lines)

    # ----- Base controls -----
    def inc_linear(self):
        self.cmd_vel.linear.x = min(self.cmd_vel.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX)

    def dec_linear(self):
        self.cmd_vel.linear.x = max(self.cmd_vel.linear.x - BASE_LINEAR_VEL_STEP, -BASE_LINEAR_VEL_MAX)

    def inc_ang(self):
        self.cmd_vel.angular.z = min(self.cmd_vel.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX)

    def dec_ang(self):
        self.cmd_vel.angular.z = max(self.cmd_vel.angular.z - BASE_ANGULAR_VEL_STEP, -BASE_ANGULAR_VEL_MAX)

    def stop_base(self):
        self.cmd_vel = Twist()

    # ----- Gripper controls -----
    def gripper_open(self):
        self._send_gripper_goal(+0.025)

    def gripper_close(self):
        self._send_gripper_goal(-0.015)

    def _send_gripper_goal(self, position: float):
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn("Gripper action server not ready.")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0
        self.last_gripper_cmd = position
        self.gripper_client.send_goal_async(goal)

    # ----- Preset poses (position increments / displacements) -----
    def move_pose(self, key: str):
        """
        Move to a preset pose using JointJog.displacements.
        POSES[key] must be {joint_name: target_position_in_rad}.
        The node computes displacement = target - current.
        """
        if key not in POSES:
            self.get_logger().warn(f"POSE '{key}' not in POSES")
            return
        target_map = POSES[key]
        if not self.current_joints:
            self.get_logger().warn("No joint_states yet; cannot compute displacements.")
            return

        jj = JointJog()
        jj.header.frame_id = BASE_FRAME_ID
        jj.joint_names = []
        jj.displacements = []
        jj.velocities = []  # optional: leave empty to use Servo side limits

        for jn, tgt in target_map.items():
            cur = self.current_joints.get(jn, None)
            if cur is None:
                continue
            disp = float(tgt) - float(cur)
            jj.joint_names.append(jn)
            jj.displacements.append(disp)

        if not jj.joint_names:
            self.get_logger().warn("No matching joints for the pose; check joint names.")
            return

        self.pending_jointjog = jj
        self.publish_joint_pending = True

    # ----- Shutdown -----
    def shutdown(self):
        try:
            self.stop_moveit_servo()
        except Exception:
            pass


# ========= Key bindings =========
KEY_HELP = """
KEY BINDINGS:
  w/x : +linear / -linear
  a/d : +angular / -angular
  s   : stop base
  g/h : gripper open / close
  0/9/8 : pose Home / Extend / Custom
  q   : quit
"""

def handle_key(node: TeleopController, ch: str) -> bool:
    """
    Returns False to request program exit.
    """
    if ch is None:
        return True
    c = ch.lower()
    if c == 'w':
        node.inc_linear()
    elif c == 'x':
        node.dec_linear()
    elif c == 'a':
        node.inc_ang()
    elif c == 'd':
        node.dec_ang()
    elif c == 's':
        node.stop_base()
    elif c == 'g':
        node.gripper_open()
    elif c == 'h':
        node.gripper_close()
    elif c == '0':
        node.move_pose("home")
    elif c == '9':
        node.move_pose("extend")
    elif c == '8':
        node.move_pose("custom")
    elif c == 'q':
        return False
    return True


# ========= Entry point =========
def main():
    rclpy.init()
    node = TeleopController()

    kb = KeyboardThread()
    kb.start()

    execu = MultiThreadedExecutor()
    execu.add_node(node)

    try:
        while rclpy.ok():
            execu.spin_once(timeout_sec=0.05)
            k = kb.pop_key()
            if k is not None:
                if not handle_key(node, k):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("\nBye.")

if __name__ == "__main__":
    main()
