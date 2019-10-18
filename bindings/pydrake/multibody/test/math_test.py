import unittest

import numpy as np
import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.multibody.math import (
    SpatialAcceleration_,
    SpatialForce_,
    SpatialVelocity_,
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
        numpy_compare.assert_float_equal(vec1.rotational(), rotation_expected)
        numpy_compare.assert_float_equal(
                vec1.translational(), translation_expected)
        vec_zero = cls()
        vec_zero.SetZero()
        vec_zero_to_float = numpy_compare.to_float(vec_zero.get_coeffs())
        numpy_compare.assert_float_equal(
                cls.Zero().get_coeffs(), vec_zero_to_float)
        coeffs_expected = np.hstack((rotation_expected, translation_expected))
        coeffs_args = {coeffs_name: coeffs_expected}
        numpy_compare.assert_float_equal(
                cls(**coeffs_args).get_coeffs(), coeffs_expected)

    @numpy_compare.check_all_types
    def test_spatial_vector_types(self, T):
        self.check_spatial_vector(SpatialVelocity_[T], "V", "w", "v")
        self.check_spatial_vector(
                SpatialAcceleration_[T], "A", "alpha", "a")
        self.check_spatial_vector(SpatialForce_[T], "F", "tau", "f")
