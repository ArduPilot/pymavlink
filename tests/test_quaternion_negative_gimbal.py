import unittest
import numpy as np
from numpy.testing import assert_allclose
from pymavlink.quaternion import QuaternionBase


class NegativePitchGimbalLockTest(unittest.TestCase):
    def test_euler_roundtrip_preserves_attitude_at_negative_gimbal_lock(self):
        for roll in [-1.1, 0.0, 0.8]:
            for yaw in [-2.0, -0.4, 0.7, 2.3]:
                with self.subTest(roll=roll, yaw=yaw):
                    original = QuaternionBase([roll, -np.pi / 2, yaw]).dcm
                    euler = QuaternionBase(original).euler
                    reconstructed = QuaternionBase(euler).dcm
                    assert_allclose(reconstructed, original, atol=1e-14)
