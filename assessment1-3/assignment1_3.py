import numpy as np
L0 = 138 # height of shoulder raise joint above the ground plane
L1 = 135 # length of the upper arm
L2 = 147 # length of the lower arm
L3 = 30  # horizontal tool displacement from wrist passive joint
L4 = -90 # vertical tool displacement from wrist passive joint
# --------- Question 1 ---------- #


def rot_mat(axis, angle):
    if axis == 'x':
        r = np.array([[1, 0, 0],
                            [0, np.cos(angle), -np.sin(angle)],
                            [0, np.sin(angle), np.cos(angle)]])
    elif axis == 'y':
        r = np.array([[np.cos(angle), 0, np.sin(angle)],
                            [0, 1, 0],
                            [-np.sin(angle), 0, np.cos(angle)]])
    elif axis == 'z':
        r = np.array([[np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0],
                            [0, 0, 1]])
    rot_mat = np.eye(4)
    rot_mat[:3, :3] = r
    return rot_mat

def transform_mat(x, y, z):
    return np.array([[1, 0, 0, x],
                        [0, 1, 0, y],
                        [0, 0, 1, z],
                        [0, 0, 0, 1]])


def forward_kinematics_q(q):
    """
    Calculate the forward kinematics of the robot for a given set of joint angles.
    
    Parameters
    ----------
    q : numpy array of shape (4,)
        Joint angles [q1, q2, q3, q4] in radians.
    
    Returns
    -------
    p : numpy array of shape (3,)
        Position of the end effector in the world frame (mm).
    """
    q1, q2, q3, q4 = q
    return (rot_mat('z', q1) @ transform_mat(0, 0, L0) @ rot_mat('y', q2) @ transform_mat(0, 0, L1) @ rot_mat('y', q3) @ transform_mat(L2, 0, 0) @ rot_mat('y', q4) @ transform_mat(L3, 0, L4))[:3, 3]
# --------- Question 2 ---------- #

def joint_mapping(th):
    """
    Map the physical joint angles to the kinematic joint angles.
    
    Parameters
    ----------
    th : numpy array of shape (3,)
        Physical joint angles [theta1, theta2, theta3] in radians.
    
    Returns
    -------
    q : numpy array of shape (4,)
        Kinematic joint angles [q1, q2, q3, q4] in radians.
    """
    theta1, theta2, theta3 = th
    q1 = theta1
    q2 = theta2
    q3 = theta3 - theta2
    q4 = -theta3
    return np.array([q1, q2, q3, q4])

# --------- Question 3 ---------- #

def forward_kinematics(theta):
    """
    Calculate the forward kinematics of the robot for a given set of physical joint angles.
    
    Parameters
    ----------
    theta : numpy array of shape (3,)
        Physical joint angles [theta1, theta2, theta3] in radians.
    
    Returns
    -------
    p : numpy array of shape (3,)
        Position of the end effector in the world frame (mm).
    """
    # First, map physical joint angles to kinematic joint angles
    q = joint_mapping(theta)
    
    # Now compute forward kinematics using kinematic joint angles
    return forward_kinematics_q(q)


if __name__ == "__main__":
    # run unit tests
    import os
    import pytest

    # Change to directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Run pytest on the private_tests directory
    pytest.main(["."])
