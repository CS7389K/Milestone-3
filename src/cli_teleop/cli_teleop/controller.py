import sys
import os
import signal
import time
import threading
import tty
import termios

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger

from .util.constants import (
    BASE_TWIST_TOPIC,
    ARM_JOINT_TOPIC,
    GRIPPER_ACTION,
    ROS_QUEUE_SIZE,
    BASE_LINEAR_VEL_MAX,
    BASE_LINEAR_VEL_STEP,
    BASE_ANGULAR_VEL_MAX,
    BASE_ANGULAR_VEL_STEP,
    BASE_FRAME_ID,
    ARM_JOINT_VEL,
    SERVO_START_SRV,
    SERVO_STOP_SRV
)
from .util.errors import ValidationError


class TeleopController(Node):
    """
    Based on turtlebot3_manipulation_teleop:
    - https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/humble/turtlebot3_manipulation_teleop/src/turtlebot3_manipulation_teleop.cpp
    - https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/humble/turtlebot3_manipulation_teleop/include/turtlebot3_manipulation_teleop/turtlebot3_manipulation_teleop.hpp
    """

    def __init__(self):
        # Create node interactions
        self.base_twist_pub = self.create_publisher(Twist, BASE_TWIST_TOPIC, ROS_QUEUE_SIZE)
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, ROS_QUEUE_SIZE)
        self.servo_start_client = self.create_client(Trigger, SERVO_START_SRV)
        self.servo_stop_client  = self.create_client(Trigger, SERVO_STOP_SRV)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.pub_timer = self.create_timer(0.01, self._publish_loop)

        # State
        self.cmd_vel = Twist()
        self.publish_joint_pending = False
        self.joint_msg = JointJog()
        self.joint_msg.header.frame_id = BASE_FRAME_ID

        # Start moveit interface
        self._connect_moveit_servo()
        self._start_moveit_servo()

        # Start keyboard loop in a background thread
        self._run = True
        self.kb_thread = threading.Thread(target=self._key_loop, daemon=True)
        self.kb_thread.start()


    def _connect_moveit_servo(self):
        for i in range(10):
            if self.servo_start_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SUCCESS TO CONNECT SERVO START SERVER')
                break
            self.get_logger().warn('WAIT TO CONNECT SERVO START SERVER')
            if i == 9:
                self.get_logger().error(
                    "fail to connect moveit_servo. please launch 'servo.launch' from the MoveIt config package."
                )

        for i in range(10):
            if self.servo_stop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SUCCESS TO CONNECT SERVO STOP SERVER')
                break
            self.get_logger().warn('WAIT TO CONNECT SERVO STOP SERVER')
            if i == 9:
                self.get_logger().error(
                    "fail to connect moveit_servo. please launch 'servo.launch' from the MoveIt config package."
                )

    def _start_moveit_servo(self):
        self.get_logger().info("call 'moveit_servo' start srv.")
        if not self.servo_start_client.service_is_ready():
            self.get_logger().warn("start_servo service not ready; continuing without moveit_servo.")
            return
        future = self.servo_start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.done() and future.result():
            self.get_logger().info("SUCCESS to start 'moveit_servo'")
        else:
            self.get_logger().error("FAIL to start 'moveit_servo', executing without 'moveit_servo'")


    def _stop_moveit_servo(self):
        self.get_logger().info("call 'moveit_servo' END srv.")
        if not self.servo_stop_client.service_is_ready():
            return
        future = self.servo_stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)


    def send_gripper_goal(self, position: float):
        """
        position (meters): positive to open (~0.025), negative to close (~-0.015)
        """
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper action server not ready.')
            return

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = -1.0

        self.get_logger().info('Sending gripper goal')
        self.gripper_client.send_goal_async(goal).add_done_callback(self._on_gripper_goal_sent)


    def _on_gripper_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(self._on_gripper_result)


    def _on_gripper_result(self, future):
        result = future.result()
        _ = result

    def shutdown(self):
        self._stop_moveit_servo()



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