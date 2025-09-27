#!/usr/bin/env python

import machinevisiontoolbox as mvt
import numpy as np
import math
from typing import Tuple
import cv2


# --------- Question 1 ---------- #
def calculate_surface_area(H: np.ndarray) -> float:
    """
    Given the Homography H, return the area of the triangle with image vertices (650, 640), (580, 810), and (530, 640)
    H is defined from the image plane to the work-surface plane.
    Hint: You might find the cv2.contourArea function useful.

    Parameters
    ----------
    H
        Homography from image plane to the work-surface plane

    Returns
    -------
    float
        Area of the triangle
    """

    pass

# --------- Question 2 ---------- #
def get_image_coordinates(H: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """
    Given Homography H and point Q in world coordinates, return the image
    coordinates of P (the location of Q in the image).
    H is defined from the image plane to the work-surface plane.

    Parameters
    ----------
    H
        Homography from image plane to work-surface plane
    Q
        Cartesian coordinates of point Q on the planar work-surface. Q will be numpy array of shape (2,1).

    Returns
    -------
    np.ndarray
        Cartesian coordinates of point P in image coordinates. Return as a numpy array of shape (2,1).

    """

    pass

if __name__ == "__main__":
    example_H = np.array([[-0.8,0,0],[0,0.8,800],[0,0,2]])
    print(f"Surface area: {calculate_surface_area(example_H)}")

    example_P = np.array([[320],[250]])
    print(f"P in image coordinates:\n{get_image_coordinates(example_H, example_P)}")


    # run unit tests
    import os
    import pytest

    # Change to directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Run pytest on the private_tests directory
    pytest.main(["."])
