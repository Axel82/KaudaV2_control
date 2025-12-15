# kinematics_5dof.py
import numpy as np
import math
from config import *

# -------------------------
# Helpers: transforms
# -------------------------
def rotz(theta):
    c = np.cos(theta); s = np.sin(theta)
    R = np.array([[c, -s, 0, 0],
                  [s,  c, 0, 0],
                  [0,  0, 1, 0],
                  [0,  0, 0, 1]], dtype=float)
    return R

def rotx(theta):
    c = np.cos(theta); s = np.sin(theta)
    R = np.array([[1, 0,  0, 0],
                  [0, c, -s, 0],
                  [0, s,  c, 0],
                  [0, 0,  0, 1]], dtype=float)
    return R

def roty(theta):
    c = np.cos(theta); s = np.sin(theta)
    R = np.array([[ c, 0, s, 0],
                  [ 0, 1, 0, 0],
                  [-s, 0, c, 0],
                  [ 0, 0, 0, 1]], dtype=float)
    return R

def tx(a):
    T = np.array([[1,0,0,a],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1]], dtype=float)
    return T

# -------------------------
# Forward kinematics
# -------------------------
# -------------------------
# Forward kinematics
# -------------------------
def forward_kinematics(thetas, links, axes=None):
    """
    thetas: iterable of 5 joint angles in radians [θ1,θ2,θ3,θ4,θ5]
    links: iterable of 5 link lengths [LBASE, L1, L2, L3, L4] in same units (mm)
    axes: iterable of 5 rotation axes (e.g. ['z', 'x', 'x', 'y', 'x']). 
          If None, uses JOINT_AXES from config.
    Returns:
      T: 4x4 numpy array total transform from base to tool
      pts: list of intermediate joint positions in world frame:
           [p0, p1, p2, p3, p4, p5] where p0 is origin, p5 is tool tip
    Convention:
      Joint i: rotate about its axis, then translate along LOCAL Z.
      Assumption: Robot is a vertical stack (Snake-like) or segments defined along Z.
      This ensures Rx/Ry rotations provide Pitch/Yaw deflection from vertical.
    """
    if len(thetas) != 5 or len(links) != 5:
        raise ValueError("Require 5 thetas and 5 links")

    if axes is None:
        axes = JOINT_AXES

    T = np.eye(4)
    pts = [T[0:3,3].copy()]  # p0

    for i in range(5):
        th = thetas[i]
        L = links[i]
        axis = axes[i]
        
        # Rotation
        if axis == 'z':
            R = rotz(th)
        elif axis == 'x':
            R = rotx(th)
        elif axis == 'y':
            R = roty(th)
        else:
            raise ValueError(f"Unknown axis {axis}")
            
        # Translation
        # ALWAYS translate along Z axis of the rotated frame.
        # This assumes current link extends along Z.
        Tr = tz(L)
            
        Ti = R @ Tr
        T = T @ Ti
        pts.append(T[0:3,3].copy())

    return T, pts

# -------------------------
# Numeric Jacobian (position only)
# -------------------------
def jacobian_numeric(thetas, links, axes=None, eps=1e-7):
    """
    Compute numeric Jacobian (3x5) of end-effector position w.r.t joint angles.
    """
    thetas = np.array(thetas, dtype=float)
    base_T, base_pts = forward_kinematics(thetas, links, axes)
    p0 = base_T[0:3,3]
    n = len(thetas)
    J = np.zeros((3, n), dtype=float)
    for i in range(n):
        th_p = thetas.copy()
        th_p[i] += eps
        Tp, _ = forward_kinematics(th_p, links, axes)
        pp = Tp[0:3,3]
        J[:,i] = (pp - p0) / eps
    return J

# -------------------------
# IK: Jacobian-transpose method (position only)
# -------------------------
def ik_jacobian_transpose(target_pos, thetas0, links, axes=None,
                          max_iter=1500, tol=1e-2, alpha=0.1): # Tuned alpha/tol
    """
    Simple iterative IK to reach target_pos (3-vector).
    - thetas0: initial guess in radians (len=5)
    - links: 5-length list
    Returns: (success:boolean, thetas_solution(array radians), info:str)
    """
    thetas = np.array(thetas0, dtype=float)
    target = np.array(target_pos, dtype=float)
    
    # joint limits in radians
    limits_rad = []
    for i in range(1, 6):
        mn, mx = get_joint_limits(i)
        limits_rad.append((math.radians(mn), math.radians(mx)))

    for it in range(max_iter):
        T, _ = forward_kinematics(thetas, links, axes)
        pos = T[0:3,3]
        err = target - pos
        err_norm = np.linalg.norm(err)
        
        if err_norm < tol:
            return True, thetas, f"Converged in {it} iters, err={err_norm:.6f}"
            
        J = jacobian_numeric(thetas, links, axes)
        # Jacobian transpose step
        dtheta = alpha * (J.T @ err)
        
        # Adaptive step size/clipping
        max_step = 0.5  # radians per iteration
        step_norm = np.max(np.abs(dtheta))
        if step_norm > max_step:
            dtheta = dtheta * (max_step / step_norm)

        thetas += dtheta
        
        # Clamp to limits
        for i in range(5):
             thetas[i] = max(limits_rad[i][0], min(limits_rad[i][1], thetas[i]))
             
    return False, thetas, f"Not converged after {max_iter} iters, err={err_norm:.6f}"

# ---------------------------
# Inverse kinematics Wrapper
# ---------------------------
def inverse_kinematics(x, y, z, tool_angle_deg=0.0, gripper_angle_deg=0.0):
    """
    Wrapper for Numerical IK to replace the old analytical one.
    - tool_angle_deg/gripper_angle_deg: mostly ignored for position solving 
      but passed through or used if we add orientation constraints later.
      For now, we map gripper angle to J5 if needed, or just solve XYZ for the arm.
    """
    # Target
    target = [x, y, z]
    
    # Links
    links = [
        get_segment_length(0), # LBASE
        get_segment_length(1), # L1
        get_segment_length(2), # L2
        get_segment_length(3), # L3
        get_segment_length(4)  # L4
    ]
    
    # Use Home as initial guess or last known? 
    # For stateless function, use Home.
    guess_deg = HOME_ANGULAR_POSITION
    guess_rad = deg2rad(guess_deg)
    
    success, res_rad, info = ik_jacobian_transpose(target, guess_rad, links)
    
    res_deg = rad2deg(res_rad)
    
    # Map J5 directly from input? Or include in IK?
    # If J5 is the gripper/tool rotation, and we want to control it manually:
    # The prompt says "Segment4 ... measures 30mm".
    # Usually the last joint controls orientation.
    # Let's trust the IK for the full chain if we want the TIP to reach XYZ.
    
    # Ensure J5 is controlled by the slider if it's purely a gripper?
    # But J5 is Segment 4 (Rx).
    # The user input "tool_angle_deg" was previously J4.
    # "gripper_angle_deg" was J5.
    # With 5 DOF, we define the pose. 
    # Let's perform IK for X,Y,Z. Numerical IK gives J1-J5.
    
    return [
        res_deg[0],
        res_deg[1],
        res_deg[2],
        res_deg[3],
        res_deg[4]
    ]

# ---------------------------
# Helpers needed for FK
# ---------------------------
def tz(a):
    T = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,a],
                  [0,0,0,1]], dtype=float)
    return T

# -------------------------
# Utility: degrees <-> radians conversion
# -------------------------
def deg2rad(arr):
    return np.array(arr, dtype=float) * np.pi / 180.0

def rad2deg(arr):
    return np.array(arr, dtype=float) * 180.0 / np.pi