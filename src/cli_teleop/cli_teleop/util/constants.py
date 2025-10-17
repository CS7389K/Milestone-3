BASE_TWIST_TOPIC = 'cmd_vel'
ARM_JOINT_TOPIC = '/servo_node/delta_joint_cmds'
GRIPPER_ACTION = 'gripper_controller/gripper_cmd'  # action name

ROS_QUEUE_SIZE = 10

BASE_LINEAR_VEL_MAX = 0.26  # m/s
BASE_LINEAR_VEL_STEP = 0.01  # m/s

BASE_ANGULAR_VEL_MAX = 1.8   # rad/s
BASE_ANGULAR_VEL_STEP = 0.1  # rad/s

BASE_FRAME_ID = 'link0'
ARM_JOINT_VEL = 10.0  # rad/s

SERVO_START_SRV = '/servo_node/start_servo'
SERVO_STOP_SRV  = '/servo_node/stop_servo'