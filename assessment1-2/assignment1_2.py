import numpy as np
import matplotlib.pyplot as plt


# --------- Question 1 ---------- #

eps = 1e-6
def rotation_matrix(axis, angle):
    """
    Compute 3D rotation matrix for a given axis and angle

    Parameters
    ----------
    axis
        the axis of rotation, as a string "x", "y" or "z"
    angle
        the angle of rotation in radians

    Returns
    -------
    R
        the SO(3) rotation matrix
    """
    if (axis == "x"):
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
    elif (axis == "y"):
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
    elif (axis == "z"):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])
    pass

# --------- Question 2 ---------- #


def using_so3():
    """
    Using a 3D rotation matrix

    Returns
    -------
    R
        the SO(3) rotation matrix describing the orientation of frame {A} with respect
        to the world frame
    p_0
        the coordinate vector P with respect to the world frame (1D array)
    q_A
        the coordinate vector Q with respect to the {A} frame (1D array)
 
    """
# Consecutive intrinsic rotations: x=0.2, y=0.3, z=0.4
    R = rotation_matrix("x", 0.2) @ rotation_matrix("y", 0.3) @ rotation_matrix("z", 0.4)

    # Example: point P in {A}
    P_A = np.array([1.0, 2.0, 3.0])
    P_0 = R @ P_A

    # Example: point Q in {0}
    Q_0 = np.array([3.0, 4.0, 1.0])
    # Because R is orthonormal, inverse = transpose
    Q_A = R.T @ Q_0

    return R, P_0, Q_A
# --------- Question 3 ---------- #
def transformation_matrix(axis, angle, t):
    """
    Create a 3D homogeneous transformation matrix

    Parameters
    ----------
    axis
        the axis of rotation, as a string "x", "y" or "z"
    angle
        the angle of rotation in radians
    t
        the translation vector (1D array)

    Returns
    -------
    T
        the SE(3) transformation matrix
    """
    R = rotation_matrix(axis, angle)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, float)
    return T

# --------- Question 4 ---------- #

def is_se3(matrix):
    """
    Check if a given matrix is an SE(3) matrix.

    Parameters
    ----------
    matrix : numpy array
        The matrix to check

    Returns
    -------
    bool
        True if the matrix is an SE(3) homogeneous transformation matrix,
        False otherwise.
    """
    if (matrix.shape != (4,4)):
        return False
    if not np.allclose(matrix[3,:], [0,0,0,1], atol=eps):
        return False
    R = matrix[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=eps):
        return False
    if not np.isclose(np.linalg.det(R), 1.0, atol=eps):
        return False
    return True

# --------- Question 5 ---------- #
def using_se3():
    """
    Using a 3D homogeneous transformation matrix

    Returns
    -------
    p_0
        the point P in the world frame (1D array)
    q_A
        the point Q in the A frame (1D array)
    """
# Rotation part (same as Q2)
    R = rotation_matrix("x", 0.2) @ rotation_matrix("y", 0.3) @ rotation_matrix("z", 0.4)


    t = np.array([7.0, 8.0, 9.0])

    # SE(3) matrix
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3]  = t

    # P in {A}
    P_A = np.array([1.0, 2.0, 3.0])
    P_0 = R @ P_A + t

    # Q in {0}
    Q_0 = np.array([3.0, 4.0, 1.0])
    Q_A = R.T @ (Q_0 - t)

    return P_0, Q_A

if __name__ == "__main__":
    # run unit tests
    import os, pytest

    # Change to directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Run pytest on the private_tests directory
    pytest.main(["."])
