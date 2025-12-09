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
def forward_kinematics(thetas, links):
    """
    thetas: iterable of 5 joint angles in radians [θ1,θ2,θ3,θ4,θ5]
    links: iterable of 5 link lengths [L1,L2,L3,L4,L5] in same units (mm)
    Returns:
      T: 4x4 numpy array total transform from base to tool
      pts: list of intermediate joint positions in world frame:
           [p0, p1, p2, p3, p4, p5] where p0 is origin, p5 is tool tip
    Convention:
      Joint i: rotate about its axis (z/y/y/y/x respectively), then translate along local X by L_i.
    """
    if len(thetas) != 5 or len(links) != 5:
        raise ValueError("Require 5 thetas and 5 links")

    # axes sequence
    axes = ['z','y','y','y','x']
    T = np.eye(4)
    pts = [T[0:3,3].copy()]  # p0
    for i in range(5):
        th = thetas[i]
        L = links[i]
        axis = axes[i]
        if axis == 'z':
            Ti = rotz(th) @ tx(L)
        elif axis == 'x':
            Ti = rotx(th) @ tx(L)
        elif axis == 'y':
            Ti = roty(th) @ tx(L)
        else:
            raise ValueError("Unknown axis")
        T = T @ Ti
        pts.append(T[0:3,3].copy())

    return T, pts

# ---------------------------
# Inverse kinematics
# ---------------------------
def inverse_kinematics(x, y, z, tool_angle_deg=0.0, gripper_angle_deg=0.0):
    """
    Return angles [theta1, theta2, theta3, theta4, theta5] in degrees.
    theta1: base azimuth (J1)
    theta2: shoulder elevation (J2)
    theta3: elbow (J3)
    theta4: wrist rotation (J4)
    theta5: gripper (J5)
    """
    # base
    theta1 = math.degrees(math.atan2(y, x)) if (x != 0 or y != 0) else 0.0
    # planar distance and vertical
    r = math.hypot(x, y)
    z_eff = z - LBASE  # consider base offset
    # law of cosines for elbow
    denom = 2 * L1 * L2
    num = r*r + z_eff*z_eff - L1*L1 - L2*L2
    cos_theta3 = num / denom
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # Clamp to avoid numerical errors
    theta3_rad = math.acos(cos_theta3)
    # choose elbow-down by default; to invert use -theta3_rad
    k1 = L1 + L2 * math.cos(theta3_rad)
    k2 = L2 * math.sin(theta3_rad)
    theta2_rad = math.atan2(z_eff, r) - math.atan2(k2, k1)
    theta_tool_rad = math.radians(tool_angle_deg)
    theta4_rad = theta_tool_rad - theta2_rad - theta3_rad
    return [
        math.degrees(theta1),
        math.degrees(theta2_rad),
        math.degrees(theta3_rad),
        math.degrees(theta4_rad),
        float(gripper_angle_deg)
    ]

# -------------------------
# Numeric Jacobian (position only)
# -------------------------
def jacobian_numeric(thetas, links, eps=1e-7):
    """
    Compute numeric Jacobian (3x5) of end-effector position w.r.t joint angles.
    """
    thetas = np.array(thetas, dtype=float)
    base_T, base_pts = forward_kinematics(thetas, links)
    p0 = base_T[0:3,3]
    n = len(thetas)
    J = np.zeros((3, n), dtype=float)
    for i in range(n):
        th_p = thetas.copy()
        th_p[i] += eps
        Tp, _ = forward_kinematics(th_p, links)
        pp = Tp[0:3,3]
        J[:,i] = (pp - p0) / eps
    return J

# -------------------------
# IK: Jacobian-transpose method (position only)
# -------------------------
def ik_jacobian_transpose(target_pos, thetas0, links,
                          max_iter=1000, tol=1e-3, alpha=0.5):
    """
    Simple iterative IK to reach target_pos (3-vector).
    - thetas0: initial guess in radians (len=5)
    - links: 5-length list
    Returns: (success:boolean, thetas_solution(array radians), info:str)
    Notes:
      - This solver minimizes position error only (no orientation constraint).
      - alpha is step gain (adjustable). If unstable, reduce alpha.
    """
    thetas = np.array(thetas0, dtype=float)
    target = np.array(target_pos, dtype=float)

    for it in range(max_iter):
        T, _ = forward_kinematics(thetas, links)
        pos = T[0:3,3]
        err = target - pos
        err_norm = np.linalg.norm(err)
        if err_norm < tol:
            return True, thetas, f"Converged in {it} iters, err={err_norm:.6f}"
        J = jacobian_numeric(thetas, links)
        # Jacobian transpose step
        dtheta = alpha * (J.T @ err)
        # optionally limit step size:
        max_step = 0.2  # radians per iteration
        step_norm = np.max(np.abs(dtheta))
        if step_norm > max_step:
            dtheta = dtheta * (max_step / step_norm)

        thetas += dtheta

    return False, thetas, f"Not converged after {max_iter} iters, err={err_norm:.6f}"

# -------------------------
# Utility: degrees <-> radians conversion
# -------------------------
def deg2rad(arr):
    return np.array(arr, dtype=float) * np.pi / 180.0

def rad2deg(arr):
    return np.array(arr, dtype=float) * 180.0 / np.pi