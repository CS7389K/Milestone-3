# ========= Domain Limits (tune for your robot) =========
MAX_LINEAR = 0.22       # m/s (TurtleBot3 Waffle Pi default-ish)
MIN_LINEAR = -0.22
MAX_ANGULAR = 2.84      # rad/s
MIN_ANGULAR = -2.84

LIN_STEP = 0.01         # m/s per keypress
ANG_STEP = 0.1          # rad/s per keypress

# Arm joint limits (radians) — example values, replace with your robot's spec
JOINT_LIMITS = {
    "J1": (-2.9, 2.9),
    "J2": (-1.7, 1.7),
    "J3": (-1.7, 1.7),
    "J4": (-2.9, 2.9),
}

# Poses
HOME_POSE = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}
EXTEND_POSE = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}  # ← fill with real "extend forward" values
CUSTOM_POSE = {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J4": 0.0}  # ← set at runtime if desired
