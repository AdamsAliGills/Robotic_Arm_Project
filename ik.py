import numpy as np

# Link lengths in mm
L1 = 50.0   
L2 = 93.0   
L3 = 93.0   
L4 = 50.0   # Wrist/Gripper length (approx based on image)

# Reference orientation for the end effector (wrist angle)
THETA_REF = -np.deg2rad(30.0)

# Joint limits (Degrees)
JOINT_MIN = np.array([-130.0, -110.0, -130.0, -120.0])
JOINT_MAX = np.array([ 130.0,   110.0,  130.0,  120.0])

def inverse_kinematics(x, y, z):
    """
    Calculates joint angles [theta1, theta2, theta3, theta4] in degrees
    for a target point (x, y, z) relative to the robot base frame.
    """
    z = z - 55.0
    # 1. Base rotation (Theta 1)
    theta1 = np.arctan2(y, x)

    # 2. Wrist position relative to shoulder
    # We work in the plane defined by theta1
    z3 = z + L4 * np.sin(THETA_REF)
    
    # Horizontal distance to wrist center
    x_horiz = np.sqrt(x**2 + y**2)
    x3 = x_horiz - L4 * np.cos(THETA_REF)

    # 3. Geometric IK for 2-link planar arm (L2, L3)
    # r = horizontal dist, s = vertical dist relative to shoulder joint
    r = x3
    s = z3 - L1 # Subtract base height

    # Cosine Rule for Theta 3 (Elbow)
    cos_t3 = (r*r + s*s - L2*L2 - L3*L3) / (2.0 * L2 * L3)
    
    # Check for reachability
    if cos_t3 > 1.0 or cos_t3 < -1.0:
        print("[WARNING] Target unreachable!")
        cos_t3 = np.clip(cos_t3, -1.0, 1.0)
    
    # Elbow down solution usually preferred
    theta3 = -np.arccos(cos_t3)

    # Theta 2 (Shoulder)
    k1 = L2 + L3 * np.cos(theta3)
    k2 = L3 * np.sin(theta3)
    theta2 = np.arctan2(s, r) - np.arctan2(k2, k1)

    # Theta 4 (Wrist) - compensating for previous angles to maintain THETA_REF
    theta4 = THETA_REF - theta2 - theta3

    # Convert to degrees
    angles = np.array([theta1, theta2, theta3, theta4])
    degrees = np.rad2deg(angles)
    
    # Clip to safety limits
    degrees = np.clip(degrees, JOINT_MIN, JOINT_MAX)
    
    return degrees