"""
Configuration constants for KaudaV2 Control Application
"""

# Application metadata
APP_ID = 'axelhabeillon.kauda.control.1.0'
APP_NAME = 'KAuda 5 axes Control'

# Robot segment lengths (mm)
L1 = 120.0   # shoulder -> elbow
L2 = 100.0   # elbow -> wrist
L3 = 80.0    # wrist -> tool (effector)
LBASE = 20.0  # base height
RADIUS = 10.0  # base radius for cylinders

# Cartesian slider limits (mm)
X_MIN, X_MAX = -250, 250
Y_MIN, Y_MAX = -250, 250
Z_MIN, Z_MAX = 0, 350
GRIP_MIN, GRIP_MAX = -90, 90
TOOL_MIN, TOOL_MAX = -180, 180

# Joint limits (degrees)
J1_MIN, J1_MAX = -165, 165   # base
J2_MIN, J2_MAX = -100, 120   # shoulder
J3_MIN, J3_MAX = -60, 150    # elbow
J4_MIN, J4_MAX = -175, 175   # wrist
J5_MIN, J5_MAX = -30, 130    # gripper rotation

# Serial communication
BAUDRATE = 115200

# Animation
ANIMATION_INTERVAL_MS = 30
