#!/usr/bin/env python

import machinevisiontoolbox as mvt
import numpy as np
import math
from typing import Tuple
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
    try:
        circularity = (4 * math.pi * area) / (perimeter ** 2)
    except ZeroDivisionError:
        return "other"
    print(f"area: {area}, perimeter: {perimeter}")
    if circularity > 0.8:
        return "circle"
    elif 0.6 < circularity <= 0.8:
        return "square"
    else:
        return "other"
    pass



# --------- Question 1 ---------- #
def coloured_objects(img: mvt.Image) -> Tuple[int, int, int]:
    """
    This function takes in an RGB image and returns the number of red, green, and blue squares.
    img
        RGB image. For testing, you can use img = mvt.Image.Read('sample_image.png')

    Returns
    -------
    Tuple[int, int, int]
        The number of red, green, and blue squares

    """

    red_green_chrom = img.chromaticity('RG')
    r_channel = red_green_chrom.plane('r')
    g_channel = red_green_chrom.plane('g')

    red_mask = (r_channel > 0.6) & (g_channel < 0.3)
    green_mask = (g_channel > 0.6) & (r_channel < 0.3)
    blue_mask = (r_channel < 0.3) & (g_channel < 0.3)

    red_blobs = red_mask.blobs()
    green_blobs = green_mask.blobs()
    blue_blobs = blue_mask.blobs()

    red_squares = len(red_blobs)
    green_squares = len(green_blobs)
    blue_squares = len(blue_blobs)

    return red_squares, green_squares, blue_squares


    pass

if __name__ == "__main__":
    sample_image = mvt.Image.Read('sample_image.png')
    r, g, b = coloured_objects(sample_image)
    print(f"Number of red squares: {r}")
    print(f"Number of green squares: {g}")
    print(f"Number of blue squares: {b}")

    # run unit tests
    import pytest

    # Change to directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # Run pytest on the private_tests directory
    pytest.main(["."])
