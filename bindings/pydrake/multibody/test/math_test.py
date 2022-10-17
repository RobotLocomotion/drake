import copy
import textwrap
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
    SpatialAcceleration,
    SpatialAcceleration_,
    SpatialForce,
    SpatialForce_,
    SpatialMomentum,
    SpatialMomentum_,
    SpatialVelocity,
    SpatialVelocity_,
)


class TestMultibodyTreeMath(unittest.TestCase):
    def test_default_aliases(self):
        # N.B. This is tested in general via `cpp_template`, but we add this to
        # test to also appease flake8.
        self.assertIs(SpatialAcceleration, SpatialAcceleration_[float])
        self.assertIs(SpatialForce, SpatialForce_[float])
        self.assertIs(SpatialMomentum, SpatialMomentum_[float])
        self.assertIs(SpatialVelocity, SpatialVelocity_[float])

    def check_spatial_vector(
            self, *, T, cls, base_name,
            coeffs_name, rotation_name, translation_name):
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
        # Repr.
        z = repr(T(0.0))
        type_suffix = {
            float: "",
            AutoDiffXd: "_[AutoDiffXd]",
            Expression: "_[Expression]",
        }[T]
        repr_cls_name = f"{base_name}{type_suffix}"
        self.assertEqual(
            repr(cls.Zero()),
            textwrap.dedent(f"""\
                {repr_cls_name}(
                  {rotation_name}=[{z}, {z}, {z}],
                  {translation_name}=[{z}, {z}, {z}],
                )"""))
        if T == float:
            # TODO(jwnimmer-tri) Once AutoDiffXd and Expression implement an
            # eval-able repr, then we can test more than just T=float here.
            original = cls.Zero()
            roundtrip = eval(repr(original))
            self.assertIsInstance(roundtrip, cls)
            numpy_compare.assert_float_equal(
                original.get_coeffs(), roundtrip.get_coeffs())

    @numpy_compare.check_all_types
    def test_spatial_vector_types(self, T):
        self.check_spatial_vector(
            T=T, cls=SpatialVelocity_[T], base_name="SpatialVelocity",
            coeffs_name="V", rotation_name="w", translation_name="v")
        self.check_spatial_vector(
            T=T, cls=SpatialMomentum_[T], base_name="SpatialMomentum",
            coeffs_name="L", rotation_name="h", translation_name="l")
        self.check_spatial_vector(
            T=T, cls=SpatialAcceleration_[T], base_name="SpatialAcceleration",
            coeffs_name="A", rotation_name="alpha", translation_name="a")
        self.check_spatial_vector(
            T=T, cls=SpatialForce_[T], base_name="SpatialForce",
            coeffs_name="F", rotation_name="tau", translation_name="f")

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
        self.assertIsInstance(V.Shift(offset=p), SpatialVelocity_[T])
        self.assertIsInstance(
            V.ComposeWithMovingFrameVelocity(position_of_moving_frame=p,
                                             velocity_of_moving_frame=V),
            SpatialVelocity_[T])
        F = SpatialForce_[T].Zero()
        self.assertIsInstance(V.dot(force=F), T)
        self.assertIsInstance(V @ F, T)
        L = SpatialMomentum_[T].Zero()
        self.assertIsInstance(V.dot(momentum=L), T)
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
            dut.ShiftWithZeroAngularVelocity(offset=z),
            SpatialAcceleration_[T])
        self.assertIsInstance(
            dut.Shift(offset=z, angular_velocity_of_this_frame=z),
            SpatialAcceleration_[T])
        self.assertIsInstance(
            dut.ComposeWithMovingFrameAcceleration(
                position_of_moving_frame=z,
                angular_velocity_of_this_frame=z,
                velocity_of_moving_frame=V,
                acceleration_of_moving_frame=dut),
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
        self.assertIsInstance(F.Shift(offset=p), SpatialForce_[T])
        V = SpatialVelocity_[T].Zero()
        self.assertIsInstance(F.dot(velocity=V), T)
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
        self.assertIsInstance(dut.Shift(offset=p), SpatialMomentum_[T])
        self.assertIsInstance(dut.dot(velocity=V), T)
        self.assertIsInstance(dut @ V, T)
