CONTROLLER_NAME = 'servo_keyboard_input'
SERVO_START_SRV = '/servo_server/start_servo'
SERVO_STOP_SRV  = '/servo_server/stop_servo'

BASE_TWIST_TOPIC = 'cmd_vel'
ARM_TWIST_TOPIC = "/servo_server/delta_twist_cmds"
ARM_JOINT_TOPIC  = '/servo_server/delta_joint_cmds'
GRIPPER_ACTION   = '/gripper_controller/gripper_cmd'

ROS_QUEUE_SIZE = 10

BASE_LINEAR_VEL_MAX  = 0.26
BASE_LINEAR_VEL_STEP = 0.01
BASE_ANGULAR_VEL_MAX  = 1.8
BASE_ANGULAR_VEL_STEP = 0.1
BASE_FRAME_ID = 'base_link'
ARM_JOINT_VEL = 10


POSES = {
    "home":   {"joint1": 10.0,  "joint2": 0.0, "joint3": 0.0, "joint4": 0.0},
    "extend": {"joint1": 5.0,  "joint2":  0.0, "joint3":  0.0, "joint4": 0.0},
    "custom": {"joint1": -0.5,  "joint2":  0.0, "joint3": 0.0, "joint4": 0.0},
}