import numpy as n
import numpy as np

import scipy.optimize


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



# --------- Question 1 ---------- #

L0 = 138 # height of shoulder raise joint above the ground plane

L1 = 135 # length of upper arm

L2 = 147 # length of lower arm

L3 = 50  # horizontal tool displacement from wrist passive joint

L4 = -120 # vertical tool displacement from wrist passive joint


def cost(theta, pstar):

    """

    Cost function for inverse kinematics


    Parameters

    ----------

    theta

        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)

    pstar

        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)


    Returns

    -------

    c

        A scalar value cost which is the Euclidean distance between

        forward kinematics and pstar


    """

    p = forward_kinematics(theta)

    c = np.linalg.norm(p - pstar)

    return c

pass


# --------- Question 2 ---------- #



def inverse_kinematics(pstar):

    """

    Inverse kinematics using optimisation


    Parameters

    ----------

    pstar

        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)


    Returns

    -------

    theta

        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)

    """

    # Initial guess for theta

    theta0 = np.array([0.0, 0.0, 0.0])

    # Optimize using scipy's minimize function

    result = scipy.optimize.minimize(cost, theta0, args=(pstar,), method='L-BFGS-B')

    return result.x


pass


# --------- Question 3 ---------- #



def inverse_kinematics_geom(pstar):

    """

    Inverse kinematics using geometry


    Parameters

    ----------

    pstar

        A numpy array of shape (3,) of the desired end-effector position [x, y, z] (mm)


    Returns

    -------

    theta

        A numpy array of shape (3,) of joint angles [theta1, theta2, theta3] (radians)


    """

    x, y, z = pstar

    # Calculate theta1

    theta1 = np.arctan2(y, x)

    # Calculate r

    r = y / np.sin(theta1)

    # Calculate b

    b = L0 + L4 - z

    # Calculate a

    a = r - L3

    # Calculate sigma

    sigma = np.arctan2(a, b)

    # Calcultate c

    c = a / np.sin(sigma)

    # Calculate alpha

    alpha = np.arccos( (L1**2 + L2**2 - c**2) / (2 * L1 * L2) )

    # Calculate beta

    beta = np.arcsin( (L2 * np.sin(alpha)) / c )

    # Calculate eta

    eta = np.pi - (alpha + beta)

    # Calculate theta2 and theta3

    theta2 = np.pi - (beta + sigma)

    theta3 = eta + (np.pi/2 - sigma)

    return np.array([theta1, theta2, theta3])


if __name__ == "__main__":

    # run unit tests

    import os

    import pytest


    # Change to directory of this file

    os.chdir(os.path.dirname(os.path.abspath(__file__)))


    # Run pytest on the private_tests directory





