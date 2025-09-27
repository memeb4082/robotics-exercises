"""
@author: Tobias Fischer, Marisa Bucolo and Jesse Haviland
"""

import unittest
from gradescope_utils.autograder_utils.decorators import (
    weight,
    number,
    visibility,
)
import os
import machinevisiontoolbox as mvt
import cv2


class TestAssignment1_5_Q1(unittest.TestCase):
    # All marks here add up to 0.0%

    @number("Q1 Test import")
    @weight(0)
    @visibility("visible")
    def test_question_1_import(self):
        """
        Test that the shape_classification method can be imported
        """

        try:
            from assignment1_5 import shape_classification  # noqa: F401
        except ImportError:
            self.fail("Could not import shape_classification from assignment4_1.py")


    @number("Q1 Test 2")
    @weight(0)
    @visibility("visible")
    def test_question_1_simple(self):
        """
        Test that the shape_classification method works for the sample images
        """

        from assignment1_5 import shape_classification

        shape = shape_classification(120, 87)

        if shape not in ["circle", "square", "other"]:
            print("Your solution did not return a valid shape type.")
            print("It should be one of 'circle', 'square', or 'other'.")

        self.assertTrue(shape in ["circle", "square", "other"], "Your solution did not return a valid shape type.")


if __name__ == "__main__":
    unittest.main()
