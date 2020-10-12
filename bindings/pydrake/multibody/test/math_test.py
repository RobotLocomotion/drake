import copy
import unittest

import numpy as np
from pydrake.autodiffutils import AutoDiffXd
from pydrake.common.cpp_param import List
from pydrake.common.value import Value
import pydrake.common.test_utilities.numpy_compare as numpy_compare
from pydrake.common.test_utilities.pickle_compare import assert_pickle
from pydrake.symbolic import Expression
from pydrake.math import RotationMatrix_
from pydrake.multibody.math import (
    SpatialAcceleration_,
    SpatialForce_,
    SpatialMomentum_,
    SpatialVelocity_,
)


class TestMultibodyTreeMath(unittest.TestCase):
    def check_spatial_vector(
            self, T, cls, coeffs_name, rotation_name, translation_name):
        vec = cls()
        # - Accessors.
        if T == Expression:
            # TODO(eric.cousineau): Teach `numpy_compare` to handle NaN in
            # symbolic expressions, without having to evaluate the expressions.
            self.assertEqual(vec.rotational().shape, (3,))
            self.assertEqual(vec.translational().shape, (3,))
        else:
            numpy_compare.assert_float_equal(
                vec.rotational(), np.full(3, np.nan))
            numpy_compare.assert_float_equal(
                vec.translational(), np.full(3, np.nan))
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
        # Test operators.
        numpy_compare.assert_float_equal(
            (-vec1).get_coeffs(), -coeffs_expected)
        new = copy.copy(vec1)
        # - Ensure in-place ops do not return a new object.
        pre_inplace = new
        new += vec1
        self.assertIs(pre_inplace, new)
        numpy_compare.assert_float_equal(new.get_coeffs(), 2 * coeffs_expected)
        numpy_compare.assert_float_equal(
            (vec1 + vec1).get_coeffs(), 2 * coeffs_expected)
        new = copy.copy(vec1)
        pre_inplace = new
        new -= vec1
        self.assertIs(pre_inplace, new)
        numpy_compare.assert_float_equal(new.get_coeffs(), np.zeros(6))
        numpy_compare.assert_float_equal(
            (vec1 - vec1).get_coeffs(), np.zeros(6))
        new = copy.copy(vec1)
        pre_inplace = new
        new *= T(2)
        self.assertIs(pre_inplace, new)
        numpy_compare.assert_float_equal(new.get_coeffs(), 2 * coeffs_expected)
        numpy_compare.assert_float_equal(
            (vec1 * T(2)).get_coeffs(), 2 * coeffs_expected)
        numpy_compare.assert_float_equal(
            (T(2) * vec1).get_coeffs(), 2 * coeffs_expected)
        R = RotationMatrix_[T]()
        numpy_compare.assert_float_equal((
            vec1.Rotate(R_FE=R)).get_coeffs(), coeffs_expected)
        # Test pickling.
        assert_pickle(self, vec1, cls.get_coeffs, T=T)

    @numpy_compare.check_all_types
    def test_spatial_vector_types(self, T):
        self.check_spatial_vector(T, SpatialVelocity_[T], "V", "w", "v")
        self.check_spatial_vector(T, SpatialMomentum_[T], "L", "h", "l")
        self.check_spatial_vector(
            T, SpatialAcceleration_[T], "A", "alpha", "a")
        self.check_spatial_vector(T, SpatialForce_[T], "F", "tau", "f")

    @numpy_compare.check_all_types
    def test_value_instantiations(self, T):
        # Existence checks.
        Value[SpatialVelocity_[T]]
        Value[List[SpatialVelocity_[T]]]
        Value[SpatialMomentum_[T]]
        Value[List[SpatialMomentum_[T]]]
        Value[SpatialAcceleration_[T]]
        Value[List[SpatialAcceleration_[T]]]
        Value[SpatialForce_[T]]
        Value[List[SpatialForce_[T]]]

    @numpy_compare.check_all_types
    def test_spatial_velocity(self, T):
        """
        Provides basic acceptance tests for SpatialVelocity, beyond what's
        tested in `test_spatial_vector_types`.
        """
        V = SpatialVelocity_[T].Zero()
        to_T = np.vectorize(T)
        p = to_T(np.zeros(3))
        self.assertIsInstance(V.Shift(p_BpBq_E=p), SpatialVelocity_[T])
        self.assertIsInstance(
            V.ComposeWithMovingFrameVelocity(p_PoBo_E=p, V_PB_E=V),
            SpatialVelocity_[T])
        F = SpatialForce_[T].Zero()
        self.assertIsInstance(V.dot(F_Q_E=F), T)
        self.assertIsInstance(V @ F, T)
        L = SpatialMomentum_[T].Zero()
        self.assertIsInstance(V.dot(L_NBp_E=L), T)
        self.assertIsInstance(V @ L, T)

    @numpy_compare.check_all_types
    def test_spatial_acceleration(self, T):
        """
        Provides basic acceptance tests for SpatialAcceleration, beyond what's
        tested in `test_spatial_vector_types`.
        """
        to_T = np.vectorize(T)
        z = to_T(np.zeros(3))
        V = SpatialVelocity_[T].Zero()
        dut = SpatialAcceleration_[T].Zero()
        self.assertIsInstance(
            dut.Shift(p_PoQ_E=z, w_WP_E=z),
            SpatialAcceleration_[T])
        self.assertIsInstance(
            dut.Shift(p_PoQ_E=z),
            SpatialAcceleration_[T])
        self.assertIsInstance(
            dut.ComposeWithMovingFrameAcceleration(
                p_PB_E=z, w_WP_E=z, V_PB_E=V, A_PB_E=dut),
            SpatialAcceleration_[T])

    @numpy_compare.check_all_types
    def test_spatial_force(self, T):
        """
        Provides basic acceptance tests for SpatialForce, beyond what's
        tested in `test_spatial_vector_types`.
        """
        F = SpatialForce_[T].Zero()
        to_T = np.vectorize(T)
        p = to_T(np.zeros(3))
        self.assertIsInstance(F.Shift(p_BpBq_E=p), SpatialForce_[T])
        V = SpatialVelocity_[T].Zero()
        self.assertIsInstance(F.dot(V_IBp_E=V), T)
        self.assertIsInstance(F @ V, T)

    @numpy_compare.check_all_types
    def test_spatial_momentum(self, T):
        """
        Provides basic acceptance tests for SpatialMomentum, beyond what's
        tested in `test_spatial_vector_types`.
        """
        to_T = np.vectorize(T)
        p = to_T(np.zeros(3))
        V = SpatialVelocity_[T].Zero()
        dut = SpatialMomentum_[T].Zero()
        self.assertIsInstance(dut.Shift(p_BpBq_E=p), SpatialMomentum_[T])
        self.assertIsInstance(dut.dot(V_IBp_E=V), T)
        self.assertIsInstance(dut @ V, T)
