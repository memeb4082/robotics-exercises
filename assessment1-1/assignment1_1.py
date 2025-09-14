import math as m
import numpy as np
import matplotlib.pyplot as plt

# --------- Question 1 ---------- #


def rot_mat(theta):
    """
    Compute a 2D rotation matrix

    Parameters
    ----------
    theta
        the angle of rotation in radians

    Returns 
    -------
    R
        the rotation as an SO(2) matrix
    """
    return np.array([[np.cos(theta),-np.sin(theta)], [np.sin(theta), np.cos(theta)]])

# --------- Question 2 ---------- #

def is_so2(matrix):
    """
    Check if a given matrix is an SO(2) rotation matrix

    Parameters
    ----------
    matrix : numpy array
        the matrix to check

    Returns
    -------
    bool
        True if the matrix is an SO(2) rotation matrix, False otherwise
    """
    theta = m.acos(matrix[1, 1]) # use the arccos of the first element to get the angle
    matrix_theta = rot_mat(theta)
    # check similarity with tolerance for floating point approximations 
    if (np.allclose(matrix, matrix_theta, rtol=1e-05, atol=1e-08)): return True
    return False

# --------- Question 3 ---------- #


def rotate_2d(ap, theta):
    """
    Transform points between 2D rotated reference frames

    Parameters
    ----------
    ap
        the point with respect to {A} as a coordinate vector (1D array)
    theta
        the angle in radians of frame {B} with respect to frame {A}

    Returns
    -------
    bp
        the point with respect to {B} as a coordinate vector (1D array)
    """
    return np.dot(rot_mat(theta), ap)

# --------- Question 4 ---------- #


def se_in_2d(x, y, theta):
    """
    Compute a 2D homogeneous transformation matrix

    Parameters
    ----------
    x
        the x coordinate of the translation
    y
        the y coordinate of the translation
    theta
        the angle of rotation in radians

    Returns
    -------
    T
        the transformation as an SE(2) matrix
    """
    return np.array([[np.cos(theta), -np.sin(theta), x],
                     [np.sin(theta), np.cos(theta), y],
                     [0, 0, 1]])
    
    pass

# --------- Question 5 ---------- #


def transform_2d(ap, T):
    """
    Transform points between 2D rotated and translated coordinate frames

    Parameters
    ----------
    ap
        the point with respect to {A} as a coordinate vector (1D array)
    T
        the transformation of frame {B} with respect to frame {A} as an SE(2) matrix

    Returns
    -------
    bp
        the point with respect to {B} as a coordinate vector (1D array)
    """
    ap_homogeneous = np.array([ap[0], ap[1], 1])
    bp_homogeneous = np.dot(np.linalg.inv(T), ap_homogeneous)
    return bp_homogeneous[:2]

if __name__ == "__main__":
    # run unit tests
    import os, pytest

    # Change to directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Run pytest on the private_tests directory
    pytest.main(["."])
