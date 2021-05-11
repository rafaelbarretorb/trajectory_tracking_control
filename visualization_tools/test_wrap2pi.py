

import unittest
import numpy as np
from linear_controller import wrapToPi, wrapToPi2

PI = np.pi

class TestWrap2Pi(unittest.TestCase):

    def test1(self):
        self.assertAlmostEqual(wrapToPi( np.radians(360)), np.radians(0.0), places=2)  # x axis positive
        self.assertAlmostEqual(wrapToPi( np.radians(-270)), np.radians(90.0), places=2)  # y axis positive
        self.assertAlmostEqual(wrapToPi( np.radians(-540)), np.radians(-180.0), places=2)  # x axis negative
        self.assertAlmostEqual(wrapToPi( np.radians(270)), np.radians(-90.0), places=2)  # y axis negative
        self.assertAlmostEqual(wrapToPi( np.radians(-270-45)), np.radians(45), places=2)  # 1st quadrant.
        self.assertAlmostEqual(wrapToPi( np.radians(-180-45)), np.radians(90+45), places=2)  # 2nd quadrant.
        self.assertAlmostEqual(wrapToPi( np.radians(180+45)), np.radians(-180+45), places=2)  # 3rd quadrant.
        self.assertAlmostEqual(wrapToPi( np.radians(270+45)), np.radians(-90+45), places=2)  # 4th quadrant.

    def test2(self):
        self.assertAlmostEqual(wrapToPi2( np.radians(270)), np.radians(-90.0), places=2)
        self.assertAlmostEqual(wrapToPi2( np.radians(360)), np.radians(0.0), places=2)  # x axis positive
        self.assertAlmostEqual(wrapToPi2( np.radians(-270)), np.radians(90.0), places=2)  # y axis positive
        self.assertAlmostEqual(wrapToPi2( np.radians(-540)), np.radians(-180.0), places=2)  # x axis negative
        self.assertAlmostEqual(wrapToPi2( np.radians(270)), np.radians(-90.0), places=2)  # y axis negative
        self.assertAlmostEqual(wrapToPi2( np.radians(-270-45)), np.radians(45), places=2)  # 1st quadrant.
        self.assertAlmostEqual(wrapToPi2( np.radians(-180-45)), np.radians(90+45), places=2)  # 2nd quadrant.
        self.assertAlmostEqual(wrapToPi2( np.radians(180+45)), np.radians(-180+45), places=2)  # 3rd quadrant.
        self.assertAlmostEqual(wrapToPi2( np.radians(270+45)), np.radians(-90+45), places=2)  # 4th quadrant.


if __name__ == '__main__':
    unittest.main()
