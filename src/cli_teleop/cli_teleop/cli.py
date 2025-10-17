import sys
import signal
import rclpy
import time
import threading
import termios
import tty
import select
import math
import os

from enum import Enum, auto
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from .controller import TeleopController
from .util.constants import (
    EXTEND_POSE,
    HOME_POSE,
    CUSTOM_POSE,
)


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


# Fixed-width template to ensure consistent overwrites
MENU_TEMPLATE = """╔═══════════════════════════════════════════════════════════════════════════╗
║        Teleoperation Control of TurtleBot3 + OpenManipulatorX            ║
╚═══════════════════════════════════════════════════════════════════════════╝

 BASE CONTROL:                      GRIPPER CONTROL:
   w : increase linear velocity       g : gripper open
   x : decrease linear velocity       h : gripper close
   a : increase angular velocity    
   d : decrease angular velocity     ARM CONTROL:
   s : base stop                       0 : Extend arm forward
                                       9 : Home pose
   q : quit program                    8 : Custom pose

─────────────────────────────────────────────────────────────────────────────
 STATUS:
─────────────────────────────────────────────────────────────────────────────
 Base Velocity:
   Linear  (X): {lin:>7.3f} m/s
   Angular (Z): {ang:>7.3f} rad/s

 Arm Joint Angles:
   J1: {J1:>7.3f} rad    J2: {J2:>7.3f} rad
   J3: {J3:>7.3f} rad    J4: {J4:>7.3f} rad
   Gripper: {gripper:>7.3f}

 Base Odometry:
   X:     {x:>8.3f} m
   Y:     {y:>8.3f} m
   Theta: {theta:>7.3f} rad
─────────────────────────────────────────────────────────────────────────────
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
    
    def joint_state_callback(self, msg):
        """Process joint state messages."""
        with self.state_lock:
            for i, name in enumerate(msg.name):
                try:
                    if name.startswith('joint') and len(name) > 5 and name[5:].isdigit():
                        joint_num = int(name[5:]) - 1
                        if 0 <= joint_num < 4 and i < len(msg.position):
                            self.joint_angles[joint_num] = msg.position[i]
                    elif 'gripper' in name.lower() and i < len(msg.position):
                        self.gripper_position = msg.position[i]
                except (ValueError, IndexError):
                    continue
    
    def odom_callback(self, msg):
        """Process odometry messages."""
        with self.state_lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.odom_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def get_state(self):
        """Get current robot state in a thread-safe manner."""
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
    """

    def __init__(self):
        """Initialize CLI with controller and state monitoring."""
        # CRITICAL: Suppress ROS logging to stdout to prevent interference
        os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = ''
        
        self.controller = TeleopController()
        self.state_monitor = StateMonitor(self.controller)
        self.running = True
        
        # Terminal settings management
        self.old_terminal_settings = None
        if sys.stdin.isatty():
            self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        
        # ANSI escape codes
        self.CLEAR_SCREEN = '\033[2J'
        self.CURSOR_HOME = '\033[H'
        self.HIDE_CURSOR = '\033[?25l'
        self.SHOW_CURSOR = '\033[?25h'
        self.CLEAR_LINE = '\033[2K'
        
        # Display control
        self.display_lock = threading.Lock()
        self.update_interval = 0.1  # 10Hz is sufficient

    def start(self):
        """Main entry point for the CLI."""
        try:
            self.setup_terminal()
            
            # Start threads
            self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
            self.kb_thread.start()
            
            self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
            self.display_thread.start()
            
            # Initial setup
            self.clear_screen()
            self.hide_cursor()
            time.sleep(0.1)  # Let terminal settle
            self.update_display()
            
            # Main ROS spin loop
            while rclpy.ok() and self.running:
                rclpy.spin_once(self.controller, timeout_sec=0.01)
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def setup_terminal(self):
        """Configure terminal for raw mode."""
        if not sys.stdin.isatty():
            return
        
        tty.setraw(sys.stdin.fileno())
        
        # Disable line buffering on stdout
        sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

    def keyboard_loop(self):
        """Non-blocking keyboard input loop."""
        while rclpy.ok() and self.running:
            try:
                if sys.stdin.isatty():
                    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if ready:
                        key = sys.stdin.read(1).lower()
                        self.process_key(key)
                else:
                    time.sleep(0.1)
            except Exception:
                break

    def display_loop(self):
        """Display update loop at fixed rate."""
        last_update = 0
        while rclpy.ok() and self.running:
            current = time.time()
            if current - last_update >= self.update_interval:
                self.update_display()
                last_update = current
            time.sleep(0.02)

    def process_key(self, key):
        """Process keypress and execute command."""
        if key not in KEYMAP:
            return

        cmd = KEYMAP[key]
        
        try:
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
                self.running = False
        except Exception:
            pass

    def update_display(self):
        """Update terminal display with current state."""
        with self.display_lock:
            try:
                # Get current state
                linear_vel = self.controller.cmd_vel.linear.x
                angular_vel = self.controller.cmd_vel.angular.z
                state = self.state_monitor.get_state()
                
                # Format display
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
                
                # Update display (cursor home + overwrite)
                output = self.CURSOR_HOME + display_text
                
                # Write atomically
                sys.stdout.write(output)
                sys.stdout.flush()
                
            except Exception:
                pass

    def clear_screen(self):
        """Clear terminal screen."""
        sys.stdout.write(self.CLEAR_SCREEN)
        sys.stdout.flush()

    def hide_cursor(self):
        """Hide terminal cursor."""
        sys.stdout.write(self.HIDE_CURSOR)
        sys.stdout.flush()

    def show_cursor(self):
        """Show terminal cursor."""
        sys.stdout.write(self.SHOW_CURSOR)
        sys.stdout.flush()

    def cleanup(self):
        """Clean up and restore terminal."""
        self.show_cursor()
        self.running = False
        
        if self.old_terminal_settings and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
        
        # Move cursor below display before exiting
        sys.stdout.write('\n' * 5)
        sys.stdout.flush()
        
        self.controller.shutdown()
        rclpy.try_shutdown()
        
        print("Teleoperation terminated.")

    def shutdown(self):
        """Handle shutdown signal."""
        self.running = False


def main():
    """Main entry point."""
    # Suppress ROS logs BEFORE rclpy.init()
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = ''
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'
    
    rclpy.init()
    
    cli = CLI()
    
    def signal_handler(sig, frame):
        cli.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        cli.start()
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
    finally:
        cli.cleanup()


if __name__ == '__main__':
    main()
