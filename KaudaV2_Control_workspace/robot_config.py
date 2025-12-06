"""
Robot configuration constants for KaudaV2
All physical parameters and limits for the 5-axis robotic arm
"""

# ============================================================================
# ROBOT GEOMETRY - Physical dimensions (mm)
# ============================================================================

# Segment lengths
L1 = 120.0      # Shoulder to elbow
L2 = 100.0      # Elbow to wrist
L3 = 80.0       # Wrist to tool (effector)
LBASE = 20.0    # Base height
RADIUS = 10.0   # Base radius for visualization cylinders

# ============================================================================
# JOINT LIMITS - Angular limits (degrees)
# ============================================================================

# Joint 1: Base (azimuthal rotation)
J1_MIN = -165
J1_MAX = 165

# Joint 2: Shoulder (elevation)
J2_MIN = -100
J2_MAX = 120

# Joint 3: Elbow
J3_MIN = -60
J3_MAX = 150

# Joint 4: Wrist (rotation)
J4_MIN = -175
J4_MAX = 175

# Joint 5: Gripper (rotation)
J5_MIN = -30
J5_MAX = 130

# ============================================================================
# CARTESIAN WORKSPACE - Position limits (mm) and tool angles (degrees)
# ============================================================================

# Position limits
X_MIN = -250
X_MAX = 250
Y_MIN = -250
Y_MAX = 250
Z_MIN = 0
Z_MAX = 350

# Tool and gripper angles
GRIP_MIN = -90
GRIP_MAX = 90
TOOL_MIN = -180
TOOL_MAX = 180

# ============================================================================
# GRIPPER CONFIGURATION - 3D visualization parameters (mm)
# ============================================================================

# Jaw dimensions (each jaw)
JAW_LENGTH = 20.0       # Length along X axis
JAW_WIDTH = 3.0         # Width along Y axis
JAW_HEIGHT = 8.0        # Height along Z axis

# Jaw spacing
JAW_OFFSET_CLOSED = 2.0     # Lateral offset when closed (±2mm = 4mm gap)
JAW_OFFSET_OPEN = 7.5       # Lateral offset when open (±7.5mm = 15mm gap)
JAW_FORWARD_OFFSET = 10.0   # Forward offset from wrist end

# ============================================================================
# KINEMATIC CONFIGURATION
# ============================================================================

# Rotation axes for each joint (for forward kinematics)
# Format: ['axis1', 'axis2', 'axis3', 'axis4', 'axis5']
JOINT_AXES = ['z', 'y', 'y', 'y', 'x']

# Home position (default safe position)
# Format: [J1, J2, J3, J4, J5] in degrees
HOME_ANGULAR_POSITION = [0, -90, 0, 0, 0]

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_joint_limits(joint_num):
    """
    Get the min/max limits for a specific joint
    Args:
        joint_num: Joint number (1-5)
    Returns:
        tuple: (min_angle, max_angle) in degrees
    """
    limits = {
        1: (J1_MIN, J1_MAX),
        2: (J2_MIN, J2_MAX),
        3: (J3_MIN, J3_MAX),
        4: (J4_MIN, J4_MAX),
        5: (J5_MIN, J5_MAX),
    }
    return limits.get(joint_num, (0, 0))


def get_segment_length(segment_num):
    """
    Get the length of a specific segment
    Args:
        segment_num: Segment number (0=base, 1-3=links)
    Returns:
        float: Length in mm
    """
    lengths = {
        0: LBASE,
        1: L1,
        2: L2,
        3: L3,
    }
    return lengths.get(segment_num, 0.0)


def get_workspace_limits():
    """
    Get the cartesian workspace limits
    Returns:
        dict: Dictionary with x, y, z limits
    """
    return {
        'x': (X_MIN, X_MAX),
        'y': (Y_MIN, Y_MAX),
        'z': (Z_MIN, Z_MAX),
        'grip': (GRIP_MIN, GRIP_MAX),
        'tool': (TOOL_MIN, TOOL_MAX),
    }
