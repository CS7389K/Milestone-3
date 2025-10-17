import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger

from .util.constants import (
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
    SERVO_STOP_SRV
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
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, ROS_QUEUE_SIZE)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.pub_timer = self.create_timer(0.01, self.publish_loop)

        # State
        self.cmd_vel = Twist()
        self.publish_joint_pending = False
        self.joint_msg = JointJog()
        self.joint_msg.header.frame_id = BASE_FRAME_ID

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

        self.get_logger().info('Sending gripper goal')
        self.gripper_client.send_goal_async(goal).add_done_callback(self.on_gripper_goal_sent)

    def on_gripper_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(self.on_gripper_result)


    def on_gripper_result(self, future):
        result = future.result()
        _ = result

    def publish_loop(self):
        if self.publish_joint_pending:
            self.joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_msg.header.frame_id = BASE_FRAME_ID
            self.joint_pub.publish(self.joint_msg)
            self.publish_joint_pending = False
            self.get_logger().info("Joint PUB")

        self.base_twist_pub.publish(self.cmd_vel)

    def inc_linear(self):
        self.cmd_vel.linear.x = min(self.cmd_vel.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0

    def dec_linear(self):
        self.cmd_vel.linear.x = max(self.cmd_vel.linear.x - BASE_LINEAR_VEL_STEP, -BASE_LINEAR_VEL_MAX)
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0

    def inc_ang(self):
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = min(self.cmd_vel.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX) 

    def dec_ang(self):
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = max(self.cmd_vel.angular.z - BASE_ANGULAR_VEL_STEP, -BASE_ANGULAR_VEL_MAX)

    def stop(self):
        self.cmd_vel = Twist()
        # self._send_cmd_vel(0.0, 0.0)

    def gripper_open(self):
        pass

    def gripper_close(self):
        pass

    def move_pose(self, pose: dict):
        pass

    def shutdown(self):
        self.stop_moveit_servo()