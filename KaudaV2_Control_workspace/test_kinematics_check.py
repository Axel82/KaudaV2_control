
import numpy as np
import math
from kinematics_5dof import forward_kinematics, inverse_kinematics, deg2rad
from robot_config import *

def test_fk():
    print("Testing Forward Kinematics...")
    # Test Home Position (all zero)
    # LBASE=80, L1=190, L2=140, L3=35, L4=30
    # J1(Z), J2(X), J3(X), J4(Y), J5(X)
    # 0 angles -> Up Z, Out X?
    # J1 RotZ(0) @ Tz(80) -> [0,0,80] facing X? 
    # J2 RotX(0) @ Tx(190) -> [190, 0, 80]?
    # J3 RotX(0) @ Tx(140) -> [330, 0, 80]?
    # J4 RotY(0) @ Tx(35)  -> [365, 0, 80]?
    # J5 RotX(0) @ Tx(30)  -> [395, 0, 80]?
    
    links = [LBASE, L1, L2, L3, L4]
    thetas = [0,0,0,0,0]
    T, pts = forward_kinematics(thetas, links)
    tip = pts[-1]
    print(f"Home Tip Position: {tip}")
    expected_x = 0 + 0 + 190 + 140 + 35 + 30 # Wait, Tx is along local X.
    # Base is Z up. Frame 1 at [0,0,80]. Orientation? RotZ(0)=Identity.
    # Frame 1 X axis is Global X.
    # Frame 2 is F1 @ RotX(0) @ Tx(190). RotX(0) is Identity. F1 @ Tx(190) -> moves along X by 190.
    # So yes, should be sum of lengths along X, except Base is Z.
    # Base contributes 0 to X.
    # Total X = 190 + 140 + 35 + 30 = 395.
    # Total Z = 80.
    
    expected = np.array([0.0, 0.0, 475.0]) # 80+190+140+35+30 = 475
    if np.allclose(tip, expected, atol=1e-3):
        print("PASS: Home Position FK correct.")
    else:
        print(f"FAIL: Expected {expected}, got {tip}")

def test_ik():
    print("\nTesting Inverse Kinematics...")
    links = [LBASE, L1, L2, L3, L4]
    target = [100, 0, 300] # Close to vertical axis
    print(f"Target: {target}")
    
    # Run IK
    angles = inverse_kinematics(target[0], target[1], target[2])
    print(f"IK Result Angles (deg): {angles}")
    
    # Verify with FK
    angles_rad = deg2rad(angles)
    T, pts = forward_kinematics(angles_rad, links)
    tip = pts[-1]
    print(f"FK check of IK result: {tip}")
    
    if np.allclose(tip, target, atol=1.0): # 1mm tolerance
        print("PASS: IK reached target.")
    else:
        print(f"FAIL: IK missed target. Diff: {np.linalg.norm(tip - target)}")

if __name__ == "__main__":
    test_fk()
    test_ik()
