# CONTROLLER_NAME = 'servo_keyboard_input'
# SERVO_START_SRV = '/servo_server/start_servo'
# SERVO_STOP_SRV  = '/servo_server/stop_servo'

# BASE_TWIST_TOPIC = 'cmd_vel'
# ARM_JOINT_TOPIC = '/servo_node/delta_joint_cmds'
# GRIPPER_ACTION = 'gripper_controller/gripper_cmd'

# ROS_QUEUE_SIZE = 10

# BASE_LINEAR_VEL_MAX = 0.26  # m/s
# BASE_LINEAR_VEL_STEP = 0.01  # m/s

# BASE_ANGULAR_VEL_MAX = 1.8   # rad/s
# BASE_ANGULAR_VEL_STEP = 0.1  # rad/s


# BASE_FRAME_ID = 'link0'
# ARM_JOINT_VEL = 0.5  # rad/s

# POSES = dict(
#     home={"joint1": 0.0, "joint2": 0.5, "joint3": 0.3, "joint4": -0.8},
#     extend={"joint1": 0.0, "joint2": 0.5, "joint3": 0.3, "joint4": -0.8},
#     custom={"joint1": 0.3, "joint3": 0.2, "joint3": 0.2, "joint4": -0.3}
# )

# ==========================
# Teleop / Servo constants
# ==========================

# Name of your ROS 2 node (shown in logs). Any string is fine.
CONTROLLER_NAME = 'servo_keyboard_input'

# MoveIt Servo start/stop services.
# Most MoveIt Servo configs use '/servo_server/start' and '/servo_server/stop'.
# If your setup exposes '/servo_server/start_servo' and '/servo_server/stop_servo',
# change these two lines accordingly (check with: `ros2 service list | grep servo`).
SERVO_START_SRV = '/servo_server/start'
SERVO_STOP_SRV  = '/servo_server/stop'

# Base (mobile platform) cmd_vel topic. Leading slash avoids namespace surprises.
BASE_TWIST_TOPIC = '/cmd_vel'

# JointJog topic that MoveIt Servo SUBSCRIBES to.
# On Humble the default is usually '/servo_server/delta_joint_cmds'.
# Verify with: `ros2 topic list | grep -Ei 'servo|joint_jog|delta_joint'`
ARM_JOINT_TOPIC  = '/servo_server/delta_joint_cmds'

# Gripper action (control_msgs/GripperCommand). Leading slash recommended.
GRIPPER_ACTION   = '/gripper_controller/gripper_cmd'

# ROS publisher queue depth (QoS history depth).
ROS_QUEUE_SIZE = 10

# ---------- Base (mobile) speed limits ----------
# Linear (m/s)
BASE_LINEAR_VEL_MAX  = 0.26
BASE_LINEAR_VEL_STEP = 0.01  # increment per key press

# Angular (rad/s)
BASE_ANGULAR_VEL_MAX  = 1.8
BASE_ANGULAR_VEL_STEP = 0.1   # increment per key press

# Frame used in message headers. Many robots use 'base_link'.
# If your URDF/TF uses 'link0', keep 'link0'; otherwise 'base_link' is typical.
BASE_FRAME_ID = 'base_link'

# Default joint jogging speed (rad/s) if you need a single common value.
ARM_JOINT_VEL = 0.5

# ---------- Preset poses for the ARM ----------
# IMPORTANT:
# 1) Keys ('joint1'...'joint4' here) MUST match exactly the names in /joint_states.name.
#    Check once with: `ros2 topic echo /joint_states -n 1`
# 2) Your current move_pose() streams JointJog.VELOCITIES (rad/s) for a short time,
#    so these values are interpreted as PER-JOINT VELOCITIES, not absolute angles.
#    Start with moderate values (0.2–0.6 rad/s) to verify motion.
POSES = {
    "home":   {"joint1": 0.0,  "joint2": -0.4, "joint3": -0.4, "joint4": 0.0},
    "extend": {"joint1": 0.0,  "joint2":  0.4, "joint3":  0.4, "joint4": 0.0},
    "custom": {"joint1": 0.3,  "joint2":  0.2, "joint3": -0.2, "joint4": 0.0},
}

# If your joints are named differently (e.g., 'waist','shoulder','elbow','wrist'),
# replace the keys above with EXACT names from /joint_states.
