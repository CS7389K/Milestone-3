import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger

from .util.constants import (
    CONTROLLER_NAME,
    BASE_TWIST_TOPIC,
    ARM_JOINT_TOPIC,
    GRIPPER_ACTION,
    ROS_QUEUE_SIZE,
    ARM_TWIST_TOPIC,
    BASE_LINEAR_VEL_MAX,
    BASE_LINEAR_VEL_STEP,
    BASE_ANGULAR_VEL_MAX,
    BASE_ANGULAR_VEL_STEP,
    BASE_FRAME_ID,
    SERVO_START_SRV,
    SERVO_STOP_SRV,
    POSES
)


class TeleopController(Node):
    """
    Based on turtlebot3_manipulation_teleop:
    - https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/humble/turtlebot3_manipulation_teleop/src/turtlebot3_manipulation_teleop.cpp
    - https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/humble/turtlebot3_manipulation_teleop/include/turtlebot3_manipulation_teleop/turtlebot3_manipulation_teleop.hpp
    """

    def __init__(self):
        super().__init__(CONTROLLER_NAME)
        # Create node interactions
        self.servo_start_client = self.create_client(Trigger, SERVO_START_SRV)
        self.servo_stop_client  = self.create_client(Trigger, SERVO_STOP_SRV)
        self.base_twist_pub = self.create_publisher(Twist, BASE_TWIST_TOPIC, ROS_QUEUE_SIZE)
        self.arm_twist_pub = self.create_publisher(TwistStamped, ARM_TWIST_TOPIC, ROS_QUEUE_SIZE)
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, ROS_QUEUE_SIZE)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self.pub_timer = self.create_timer(0.01, self.publish_loop)
        self.cmd_vel = Twist()
        self.joint_msg = JointJog()

        # Start moveit interface
        self.connect_moveit_servo()
        self.start_moveit_servo()

    def connect_moveit_servo(self):
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

    def start_moveit_servo(self):
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

    def stop_moveit_servo(self):
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

        self.gripper_client.send_goal_async(goal)

    def publish_loop(self):
        # self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        # self.joint_msg.header.frame_id = BASE_FRAME_ID

        self.base_twist_pub.publish(self.cmd_vel)
        self.joint_pub.publish(self.joint_msg)

    def inc_linear(self):
        self.cmd_vel.linear.x = min(self.cmd_vel.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.get_logger().info(f'Linear velocity: {self.cmd_vel.linear.x:.3f}')

    def dec_linear(self):
        self.cmd_vel.linear.x = max(self.cmd_vel.linear.x - BASE_LINEAR_VEL_STEP, -BASE_LINEAR_VEL_MAX)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.get_logger().info(f'Linear velocity: {self.cmd_vel.linear.x:.3f}')

    def inc_ang(self):
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = min(self.cmd_vel.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX)
        self.get_logger().info(f'Angular velocity: {self.cmd_vel.angular.z:.3f}')

    def dec_ang(self):
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = max(self.cmd_vel.angular.z - BASE_ANGULAR_VEL_STEP, -BASE_ANGULAR_VEL_MAX)
        self.get_logger().info(f'Angular velocity: {self.cmd_vel.angular.z:.3f}')

    def stop(self):
        self.cmd_vel = Twist()
        # self.get_logger().info('Base stopped')

    def gripper_open(self):
        # self.get_logger().info('Gripper OPEN command sent')
        self.send_gripper_goal(0.025)

    def gripper_close(self):
        # self.get_logger().info('Gripper CLOSE command sent')
        self.send_gripper_goal(-0.015)

    def move_pose(self, key: str):
        if key in POSES:
            for joint, value in POSES[key].items():
                self.joint_msg.joint_names.append(joint)
                self.joint_msg.displacements.append(value)
                self.joint_msg.duration = 0.1

    def shutdown(self):
        self.get_logger().info('Shutting down controller...')
        self.stop_moveit_servo()
