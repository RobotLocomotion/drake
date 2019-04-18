import unittest

import numpy as np

from pydrake.multibody.math import (
    SpatialAcceleration,
    SpatialForce,
    SpatialVelocity,
)


class TestMultibodyTreeMath(unittest.TestCase):
    def check_spatial_vector(
            self, cls, coeffs_name, rotation_name, translation_name):
        vec = cls()
        # - Accessors.
        self.assertTrue(isinstance(vec.rotational(), np.ndarray))
        self.assertTrue(isinstance(vec.translational(), np.ndarray))
        self.assertEqual(vec.rotational().shape, (3,))
        self.assertEqual(vec.translational().shape, (3,))
        # - Fully-parameterized constructor.
        rotation_expected = [0.1, 0.3, 0.5]
        translation_expected = [0., 1., 2.]
        vec_args = {
            rotation_name: rotation_expected,
            translation_name: translation_expected,
        }
        vec1 = cls(**vec_args)
        self.assertTrue(np.allclose(vec1.rotational(), rotation_expected))
        self.assertTrue(
            np.allclose(vec1.translational(), translation_expected))
        vec_zero = cls()
        vec_zero.SetZero()
        self.assertTrue(
            np.allclose(cls.Zero().get_coeffs(), vec_zero.get_coeffs()))
        coeffs_expected = np.hstack((rotation_expected, translation_expected))
        coeffs_args = {coeffs_name: coeffs_expected}
        self.assertTrue(
            np.allclose(cls(**coeffs_args).get_coeffs(), coeffs_expected))

    def test_spatial_vector_types(self):
        self.check_spatial_vector(SpatialVelocity, "V", "w", "v")
        self.check_spatial_vector(SpatialAcceleration, "A", "alpha", "a")
        self.check_spatial_vector(SpatialForce, "F", "tau", "f")
