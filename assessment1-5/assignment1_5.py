#!/usr/bin/env python

import machinevisiontoolbox as mvt
import numpy as np
import math
import os


# --------- Question 1 ---------- #
def shape_classification(area: int, perimeter: int) -> str:
    """
    Determine the type of shape (circle, square, or
    something else) from area and perimeter of a segment.

    Parameters
    ----------
    area
        Area of the segment in pixels^2
    perimeter
        Perimeter of the segment in pixels

    Returns
    -------
    str
        A string representation of the shape, either "circle", "square", or "other"

    """

    # Calculate circularity
    circularity = (4 * math.pi * area) / (perimeter ** 2)
    if circularity > 0.8:
        return "circle"
    elif 0.6 < circularity <= 0.8:
        return "square"
    else:
        return "other"
    pass

if __name__ == "__main__":
    # run unit tests
    import pytest

    # Change to directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Run pytest on the private_tests directory
    pytest.main(["."])
