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
        
        # DEBUG: Print what we imported
        self.get_logger().info('='*70)
        self.get_logger().info('INITIALIZATION DEBUG INFO')
        self.get_logger().info('='*70)
        self.get_logger().info(f'ARM_JOINT_TOPIC: {ARM_JOINT_TOPIC}')
        self.get_logger().info(f'POSES type: {type(POSES)}')
        self.get_logger().info(f'POSES keys: {list(POSES.keys())}')
        self.get_logger().info(f'POSES content: {POSES}')
        self.get_logger().info('='*70)
        
        # Create node interactions
        self.servo_start_client = self.create_client(Trigger, SERVO_START_SRV)
        self.servo_stop_client  = self.create_client(Trigger, SERVO_STOP_SRV)
        self.base_twist_pub = self.create_publisher(Twist, BASE_TWIST_TOPIC, ROS_QUEUE_SIZE)
        self.joint_pub = self.create_publisher(JointJog, ARM_JOINT_TOPIC, ROS_QUEUE_SIZE)
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.pub_timer = self.create_timer(0.01, self.publish_loop)

        # State
        self.cmd_vel = Twist()
        
        # Joint command state
        self.active_joint_cmd = None
        self.joint_cmd_count = 0
        self.joint_cmd_duration = 150  # 1.5 seconds
        
        self.joint_msg = JointJog()
        self.joint_msg.header.frame_id = BASE_FRAME_ID

        # Start moveit interface
        self.connect_moveit_servo()
        self.start_moveit_servo()
        
        # DEBUG: Final init message
        self.get_logger().info('Controller initialized successfully')

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

        self.gripper_client.send_goal_async(goal).add_done_callback(self.on_gripper_goal_sent)

    def on_gripper_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        goal_handle.get_result_async()

    def publish_loop(self):
        """
        Published at 100Hz.
        """
        # DEBUG: Log when actively publishing joint commands
        if self.active_joint_cmd is not None:
            # Log every 50 iterations (every 0.5 seconds)
            if self.joint_cmd_count % 50 == 0 or self.joint_cmd_count == self.joint_cmd_duration:
                self.get_logger().info(f'[PUBLISH_LOOP] Publishing joint cmd, count remaining: {self.joint_cmd_count}')
        
        # Publish joint commands if active
        if self.active_joint_cmd is not None and self.joint_cmd_count > 0:
            # Update timestamp for each publish
            self.active_joint_cmd.header.stamp = self.get_clock().now().to_msg()
            
            # Publish the command
            self.joint_pub.publish(self.active_joint_cmd)
            
            # Decrement counter
            self.joint_cmd_count -= 1
            
            # Clear when done
            if self.joint_cmd_count == 0:
                self.active_joint_cmd = None
                self.get_logger().info('*** JOINT COMMAND SEQUENCE COMPLETE (1.5s elapsed) ***')

        # Always publish base velocity
        self.base_twist_pub.publish(self.cmd_vel)

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
        self.get_logger().info('Base stopped')

    def gripper_open(self):
        self.get_logger().info('Gripper OPEN command sent')
        self.send_gripper_goal(0.025)

    def gripper_close(self):
        self.get_logger().info('Gripper CLOSE command sent')
        self.send_gripper_goal(-0.015)

    def move_pose(self, key: str):
        """
        Send joint velocities for the specified pose.
        FULL DEBUG VERSION
        """
        # DEBUG SECTION 1: Entry
        self.get_logger().info('='*70)
        self.get_logger().info(f'[move_pose] CALLED with key = "{key}"')
        self.get_logger().info('='*70)
        
        # DEBUG SECTION 2: POSES check
        self.get_logger().info(f'[move_pose] POSES type: {type(POSES)}')
        self.get_logger().info(f'[move_pose] POSES keys available: {list(POSES.keys())}')
        self.get_logger().info(f'[move_pose] Checking if "{key}" in POSES...')
        
        if key not in POSES:
            self.get_logger().error('='*70)
            self.get_logger().error(f'[move_pose] ERROR: Key "{key}" NOT FOUND in POSES!')
            self.get_logger().error(f'[move_pose] Available keys: {list(POSES.keys())}')
            self.get_logger().error(f'[move_pose] Check that key matches exactly (case-sensitive)')
            self.get_logger().error('='*70)
            return
        
        self.get_logger().info(f'[move_pose] ✓ Key "{key}" found in POSES')
        
        # DEBUG SECTION 3: Pose data extraction
        pose_data = POSES[key]
        self.get_logger().info(f'[move_pose] pose_data type: {type(pose_data)}')
        self.get_logger().info(f'[move_pose] pose_data content: {pose_data}')
        
        # DEBUG SECTION 4: Create JointJog message
        self.get_logger().info('[move_pose] Creating JointJog message...')
        joint_cmd = JointJog()
        joint_cmd.header.frame_id = BASE_FRAME_ID
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'[move_pose] ✓ JointJog message created, frame_id: {BASE_FRAME_ID}')
        
        # DEBUG SECTION 5: Extract joint names and velocities
        try:
            joint_names = list(pose_data.keys())
            velocities = list(pose_data.values())
            self.get_logger().info(f'[move_pose] ✓ Extracted joint_names: {joint_names}')
            self.get_logger().info(f'[move_pose] ✓ Extracted velocities: {velocities}')
        except Exception as e:
            self.get_logger().error(f'[move_pose] ERROR extracting data: {e}')
            self.get_logger().error(f'[move_pose] pose_data structure is wrong!')
            return
        
        # DEBUG SECTION 6: Assign to message
        joint_cmd.joint_names = joint_names
        joint_cmd.velocities = velocities
        joint_cmd.duration = 0.0
        self.get_logger().info('[move_pose] ✓ Message fields assigned:')
        self.get_logger().info(f'[move_pose]   - joint_names: {joint_cmd.joint_names}')
        self.get_logger().info(f'[move_pose]   - velocities: {joint_cmd.velocities}')
        self.get_logger().info(f'[move_pose]   - duration: {joint_cmd.duration}')
        
        # DEBUG SECTION 7: Set for continuous publishing
        self.get_logger().info(f'[move_pose] Setting up continuous publishing...')
        self.get_logger().info(f'[move_pose]   - joint_cmd_duration: {self.joint_cmd_duration}')
        
        self.active_joint_cmd = joint_cmd
        self.joint_cmd_count = self.joint_cmd_duration
        
        self.get_logger().info(f'[move_pose] ✓ active_joint_cmd set: {self.active_joint_cmd is not None}')
        self.get_logger().info(f'[move_pose] ✓ joint_cmd_count set to: {self.joint_cmd_count}')
        
        # DEBUG SECTION 8: Summary
        self.get_logger().info('='*70)
        self.get_logger().info(f'[move_pose] SUMMARY:')
        self.get_logger().info(f'  Pose: {key}')
        self.get_logger().info(f'  Joints: {joint_cmd.joint_names}')
        self.get_logger().info(f'  Velocities: {joint_cmd.velocities}')
        self.get_logger().info(f'  Will publish for: {self.joint_cmd_duration * 0.01:.2f} seconds')
        self.get_logger().info(f'  Publishing to topic: {ARM_JOINT_TOPIC}')
        self.get_logger().info('='*70)
        self.get_logger().info('[move_pose] Waiting for publish_loop to start publishing...')

    def shutdown(self):
        self.get_logger().info('Shutting down controller...')
        self.stop_moveit_servo()
