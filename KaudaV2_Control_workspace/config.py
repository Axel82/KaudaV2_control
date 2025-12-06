"""
Configuration constants for KaudaV2 Control Application
"""

# Application metadata
APP_ID = 'axelhabeillon.kauda.control.1.0'
APP_NAME = 'KAuda 5 axes Control'

# Serial communication
BAUDRATE = 115200

# Animation
ANIMATION_INTERVAL_MS = 30

# Import all robot-specific constants from robot_config
from robot_config import (
    # Geometry
    L1, L2, L3, LBASE, RADIUS,
    # Joint limits
    J1_MIN, J1_MAX, J2_MIN, J2_MAX, J3_MIN, J3_MAX,
    J4_MIN, J4_MAX, J5_MIN, J5_MAX,
    # Cartesian workspace
    X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX,
    GRIP_MIN, GRIP_MAX, TOOL_MIN, TOOL_MAX,
    # Gripper configuration
    JAW_LENGTH, JAW_WIDTH, JAW_HEIGHT,
    JAW_OFFSET_CLOSED, JAW_OFFSET_OPEN, JAW_FORWARD_OFFSET,
    # Kinematic configuration
    JOINT_AXES, HOME_ANGULAR_POSITION,
    # Helper functions
    get_joint_limits, get_segment_length, get_workspace_limits
)

