import sys
import signal
import rclpy
import time
import threading
import termios
import tty
import select
import math

from enum import Enum, auto
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from .controller import TeleopController


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


MENU_TEMPLATE = """
===========================================================================
 Teleoperation Control of TurtleBot3 + OpenManipulatorX
===========================================================================
 BASE CONTROL:
   w : increase linear velocity (forward)
   x : decrease linear velocity (backward)
   a : increase angular velocity (turn left)
   d : decrease angular velocity (turn right)
   s : base stop

 GRIPPER CONTROL:
   g : gripper open
   h : gripper close

 ARM CONTROL:
   0 : Extend arm forward
   9 : Home pose
   8 : Custom pose

   q : quit program
---------------------------------------------------------------------------
 Present Linear Velocity:  {lin:>6.3f} m/s
 Present Angular Velocity: {ang:>6.3f} rad/s

 Arm Joint Angles:
   J1: {J1:>6.3f} rad  J2: {J2:>6.3f} rad
   J3: {J3:>6.3f} rad  J4: {J4:>6.3f} rad
 Gripper Position: {gripper:>6.3f}

 Base Odometry:
   X: {x:>7.3f} m
   Y: {y:>7.3f} m
   Theta: {theta:>6.3f} rad
===========================================================================
"""


class StateMonitor:
    """
    Monitor robot state from sensor topics.
    Separated from controller to keep state tracking independent.
    """
    
    def __init__(self, node):
        """
        Initialize state monitor.
        
        Args:
            node: ROS2 node to use for creating subscriptions
        """
        self.node = node
        
        # State storage
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.gripper_position = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.state_lock = threading.Lock()
        
        # QoS profile for best-effort sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        self.node.get_logger().info('State monitor initialized')
    
    def joint_state_callback(self, msg):
        """
        Process joint state messages to extract arm joint angles and gripper position.
        
        Handles different joint naming conventions:
        - joint1, joint2, joint3, joint4 (arm)
        - gripper (gripper joint)
        """
        with self.state_lock:
            for i, name in enumerate(msg.name):
                try:
                    # Check for arm joints (joint1, joint2, joint3, joint4)
                    if name.startswith('joint') and len(name) > 5 and name[5:].isdigit():
                        joint_num = int(name[5:]) - 1  # joint1 -> index 0
                        if 0 <= joint_num < 4 and i < len(msg.position):
                            self.joint_angles[joint_num] = msg.position[i]
                    
                    # Check for gripper
                    elif 'gripper' in name.lower() and i < len(msg.position):
                        self.gripper_position = msg.position[i]
                        
                except (ValueError, IndexError) as e:
                    self.node.get_logger().debug(f'Error parsing joint {name}: {e}')
                    continue
    
    def odom_callback(self, msg):
        """
        Process odometry messages to extract position and orientation.
        Converts quaternion to Euler yaw angle.
        """
        with self.state_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            
            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.odom_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def get_state(self):
        """
        Get current robot state in a thread-safe manner.
        
        Returns:
            dict: Current state with all tracked values
        """
        with self.state_lock:
            return {
                'joint_angles': self.joint_angles.copy(),
                'gripper': self.gripper_position,
                'odom_x': self.odom_x,
                'odom_y': self.odom_y,
                'odom_theta': self.odom_theta
            }


class CLI:
    """
    Command-Line Interface for TurtleBot3 teleoperation with static display.
    
    This CLI uses ANSI escape codes to create a non-scrolling terminal interface
    that updates in place, providing real-time feedback without cluttering the screen.
    
    Key Features:
    - Non-blocking keyboard input (no need to press Enter)
    - Static display that updates in place (no scrolling)
    - Real-time state feedback (velocities, joint angles, odometry)
    - Thread-safe state management
    """

    def __init__(self):
        """Initialize CLI with controller and state monitoring."""
        self.controller = TeleopController()
        self.state_monitor = StateMonitor(self.controller)
        self.running = True
        
        # Terminal settings management
        self.old_terminal_settings = None
        if sys.stdin.isatty():
            self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        
        # ANSI escape codes for terminal control
        self.CLEAR_SCREEN = '\033[2J'      # Clear entire screen
        self.CURSOR_HOME = '\033[H'        # Move cursor to top-left (1,1)
        self.HIDE_CURSOR = '\033[?25l'     # Hide cursor
        self.SHOW_CURSOR = '\033[?25h'     # Show cursor
        
        # Display update control
        self.display_lock = threading.Lock()
        self.last_update_time = 0
        self.update_interval = 0.05  # Update display at 20Hz

    def start(self):
        """
        Main entry point for the CLI.
        Sets up the terminal, starts threads, and runs the main loop.
        """
        try:
            # Initialize terminal for non-blocking input
            self.setup_terminal()
            
            # Start keyboard input thread
            self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
            self.kb_thread.start()
            
            # Start display update thread
            self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
            self.display_thread.start()
            
            # Initial display
            self.clear_screen()
            self.hide_cursor()
            self.update_display()
            
            # Main ROS spin loop - controller handles all ROS interactions
            while rclpy.ok() and self.running:
                rclpy.spin_once(self.controller, timeout_sec=0.01)
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            self.controller.get_logger().info('Keyboard interrupt received')
        finally:
            self.cleanup()

    def setup_terminal(self):
        """
        Configure terminal for non-blocking, non-echoing input.
        This allows us to capture keypresses without Enter and without displaying them.
        """
        if not sys.stdin.isatty():
            self.controller.get_logger().warn('Not running in a TTY, input may not work correctly')
            return
        
        # Set terminal to raw mode (no echo, no buffering)
        tty.setraw(sys.stdin.fileno())

    def keyboard_loop(self):
        """
        Non-blocking keyboard input loop.
        Runs in a separate thread to capture keypresses without blocking ROS.
        
        Uses select() to check for available input without blocking.
        """
        while rclpy.ok() and self.running:
            try:
                # Check if input is available (non-blocking)
                if sys.stdin.isatty():
                    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if ready:
                        key = sys.stdin.read(1).lower()
                        self.process_key(key)
                else:
                    time.sleep(0.1)
            except Exception as e:
                self.controller.get_logger().error(f'Keyboard loop error: {e}')
                break

    def display_loop(self):
        """
        Display update loop.
        Runs in a separate thread to update the display at a fixed rate (20Hz).
        """
        while rclpy.ok() and self.running:
            current_time = time.time()
            if current_time - self.last_update_time >= self.update_interval:
                self.update_display()
                self.last_update_time = current_time
            time.sleep(0.01)

    def process_key(self, key):
        """
        Process a single keypress and execute the corresponding command.
        
        Args:
            key (str): The key that was pressed
        """
        if key not in KEYMAP:
            return

        cmd = KEYMAP[key]
        
        try:
            if cmd == Cmd.INC_LINEAR:
                self.controller.inc_linear()
                self.controller.get_logger().debug('Increased linear velocity')
            elif cmd == Cmd.DEC_LINEAR:
                self.controller.dec_linear()
                self.controller.get_logger().debug('Decreased linear velocity')
            elif cmd == Cmd.INC_ANG:
                self.controller.inc_ang()
                self.controller.get_logger().debug('Increased angular velocity (turn left)')
            elif cmd == Cmd.DEC_ANG:
                self.controller.dec_ang()
                self.controller.get_logger().debug('Decreased angular velocity (turn right)')
            elif cmd == Cmd.STOP:
                self.controller.stop()
                self.controller.get_logger().info('Base stopped')
            elif cmd == Cmd.GRIP_OPEN:
                self.controller.gripper_open()
            elif cmd == Cmd.GRIP_CLOSE:
                self.controller.gripper_close()
            elif cmd == Cmd.ARM_EXTEND:
                self.controller.move_pose("extend")
                self.controller.get_logger().info('Moving arm to extend pose')
            elif cmd == Cmd.ARM_HOME:
                self.controller.move_pose("home")
                self.controller.get_logger().info('Moving arm to home pose')
            elif cmd == Cmd.ARM_CUSTOM:
                self.controller.move_pose("custom")
                self.controller.get_logger().info('Moving arm to custom pose')
            elif cmd == Cmd.QUIT:
                self.running = False
                
        except Exception as e:
            self.controller.get_logger().error(f'Command execution error: {e}')

    def update_display(self):
        """
        Update the terminal display with current robot state.
        Uses ANSI codes to update in place without scrolling.
        
        The display shows:
        - Current commanded velocities
        - Arm joint angles from sensors
        - Gripper position
        - Base odometry (position and orientation)
        """
        with self.display_lock:
            # Get current velocities from controller
            linear_vel = self.controller.cmd_vel.linear.x
            angular_vel = self.controller.cmd_vel.angular.z
            
            # Get sensor state from state monitor
            state = self.state_monitor.get_state()
            
            # Format the menu with current values
            display_text = MENU_TEMPLATE.format(
                lin=linear_vel,
                ang=angular_vel,
                J1=state['joint_angles'][0],
                J2=state['joint_angles'][1],
                J3=state['joint_angles'][2],
                J4=state['joint_angles'][3],
                gripper=state['gripper'],
                x=state['odom_x'],
                y=state['odom_y'],
                theta=state['odom_theta']
            )
            
            # Move cursor to home and write (overwrites previous content)
            sys.stdout.write(self.CURSOR_HOME)
            sys.stdout.write(display_text)
            sys.stdout.flush()

    def clear_screen(self):
        """Clear the terminal screen."""
        sys.stdout.write(self.CLEAR_SCREEN)
        sys.stdout.flush()

    def hide_cursor(self):
        """Hide the terminal cursor for cleaner display."""
        sys.stdout.write(self.HIDE_CURSOR)
        sys.stdout.flush()

    def show_cursor(self):
        """Show the terminal cursor."""
        sys.stdout.write(self.SHOW_CURSOR)
        sys.stdout.flush()

    def cleanup(self):
        """
        Clean up terminal state and shutdown gracefully.
        Restores terminal to original settings.
        """
        self.show_cursor()
        self.running = False
        
        # Restore terminal settings
        if self.old_terminal_settings and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
        
        # Clear screen and move cursor to bottom
        self.clear_screen()
        
        # Shutdown controller
        self.controller.shutdown()
        rclpy.try_shutdown()
        
        print("\nTeleoperation terminated. Goodbye!")

    def shutdown(self):
        """Handle shutdown signal (SIGINT)."""
        self.controller.get_logger().info('SIGINT received, shutting down...')
        self.running = False


def main():
    """Main entry point for the CLI application."""
    rclpy.init()
    
    cli = CLI()
    
    # Setup signal handler for graceful shutdown
    def signal_handler(sig, frame):
        cli.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        cli.start()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        cli.cleanup()


if __name__ == '__main__':
    main()