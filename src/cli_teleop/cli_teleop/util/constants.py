CONTROLLER_NAME = 'servo_keyboard_input'
SERVO_START_SRV = '/servo_server/start_servo'
SERVO_STOP_SRV  = '/servo_server/stop_servo'

BASE_TWIST_TOPIC = 'cmd_vel'
ARM_JOINT_TOPIC = '/servo_node/delta_joint_cmds'
GRIPPER_ACTION = 'gripper_controller/gripper_cmd'

ROS_QUEUE_SIZE = 10

BASE_LINEAR_VEL_MAX = 0.26  # m/s
BASE_LINEAR_VEL_STEP = 0.01  # m/s

BASE_ANGULAR_VEL_MAX = 1.8   # rad/s
BASE_ANGULAR_VEL_STEP = 0.1  # rad/s


BASE_FRAME_ID = 'link0'
ARM_JOINT_VEL = 0.5  # rad/s

POSES = dict(
    home={"joint1": 0.0, "joint2": 0.5, "joint3": 0.3, "joint4": -0.8},
    extend={"joint1": 0.0, "joint2": 0.5, "joint3": 0.3, "joint4": -0.8},
    custom={"joint1": 0.3, "joint3": 0.2, "joint3": 0.2, "joint4": -0.3}
)
