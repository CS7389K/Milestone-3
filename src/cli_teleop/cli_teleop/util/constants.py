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
ARM_JOINT_VEL = 10.0  # rad/s

HOME_POSE = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}
EXTEND_POSE = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}
CUSTOM_POSE = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}