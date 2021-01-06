import pydrake.common.eigen_geometry as mut

import copy
import unittest

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.common.value import Value
import pydrake.common.test.eigen_geometry_test_util as test_util
from pydrake.common.test_utilities import numpy_compare
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.test_utilities.pickle_compare import assert_pickle


def normalize(x):
    return x / np.linalg.norm(x)


class TestEigenGeometry(unittest.TestCase):
    def check_cast(self, template, T):
        value = template[T]()
        # Refer to docstrings for `CastUPack` in `default_scalars_pybind.h`.
        if T == float:
            U_list = [float, AutoDiffXd, Expression]
        else:
            U_list = [T]
        for U in U_list:
            self.assertIsInstance(value.cast[U](), template[U], U)

    @numpy_compare.check_all_types
    def test_argument_deduction(self, T):
        # Brief check for argument deduction (#11667).
        q = mut.Quaternion_(w=T(1), x=T(0), y=T(0), z=T(0))
        self.assertIsInstance(q, mut.Quaternion_[T])

    @numpy_compare.check_all_types
    def test_quaternion(self, T):
        # Simple API.
        Quaternion = mut.Quaternion_[T]
        cast = np.vectorize(T)
        q_identity = Quaternion()
        self.assertEqual(numpy_compare.resolve_type(q_identity.wxyz()), T)
        numpy_compare.assert_float_equal(q_identity.wxyz(), [1., 0, 0, 0])
        numpy_compare.assert_float_equal(
                copy.copy(q_identity).wxyz(), [1., 0, 0, 0])
        numpy_compare.assert_equal(
                q_identity.wxyz(), Quaternion.Identity().wxyz())
        if T == float:
            self.assertEqual(
                str(q_identity),
                "Quaternion_[float](w=1.0, x=0.0, y=0.0, z=0.0)")
        self.check_cast(mut.Quaternion_, T)
        # Test ordering.
        q_wxyz = normalize([0.1, 0.3, 0.7, 0.9])
        q = Quaternion(w=q_wxyz[0], x=q_wxyz[1], y=q_wxyz[2], z=q_wxyz[3])
        # - Accessors.
        numpy_compare.assert_float_equal(q.w(), q_wxyz[0])
        numpy_compare.assert_float_equal(q.x(), q_wxyz[1])
        numpy_compare.assert_float_equal(q.y(), q_wxyz[2])
        numpy_compare.assert_float_equal(q.z(), q_wxyz[3])
        numpy_compare.assert_float_equal(q.xyz(), q_wxyz[1:])
        numpy_compare.assert_float_equal(q.wxyz(), q_wxyz)
        # - Mutators.
        q_wxyz_new = q_wxyz[::-1]
        numpy_compare.assert_not_equal(q_wxyz, q_wxyz_new)
        q.set_wxyz(wxyz=q_wxyz_new)
        numpy_compare.assert_float_equal(q.wxyz(), q_wxyz_new)
        q.set_wxyz(
            w=q_wxyz_new[0], x=q_wxyz_new[1], y=q_wxyz_new[2], z=q_wxyz_new[3])
        numpy_compare.assert_float_equal(q.wxyz(), q_wxyz_new)
        # Alternative constructors.
        q_other = Quaternion(wxyz=q_wxyz)
        numpy_compare.assert_float_equal(q_other.wxyz(), q_wxyz)
        R = np.array([
            [0., 0, 1],
            [1, 0, 0],
            [0, 1, 0]])
        q_wxyz_expected = np.array([0.5, 0.5, 0.5, 0.5])
        q_other = Quaternion(q_wxyz_expected)
        numpy_compare.assert_float_equal(q_other.rotation(), R)
        R_I = np.eye(3, 3)
        q_other.set_rotation(R_I)
        numpy_compare.assert_equal(q_other.wxyz(), q_identity.wxyz())
        # - Copy constructor.
        cp = Quaternion(other=q)
        numpy_compare.assert_equal(q.wxyz(), cp.wxyz())
        # Bad values.
        if T != Expression:
            q = Quaternion.Identity()
            # - wxyz
            q_wxyz_bad = [1., 2, 3, 4]
            with self.assertRaises(RuntimeError):
                q.set_wxyz(q_wxyz_bad)
            numpy_compare.assert_float_equal(q.wxyz(), [1., 0, 0, 0])
            # - Rotation.
            R_bad = np.copy(R)
            R_bad[0, 0] = 10
            with self.assertRaises(RuntimeError):
                q_other.set_rotation(R_bad)
            numpy_compare.assert_float_equal(q_other.rotation(), R_I)

        # Operations.
        q_AB = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        q_I = q_AB.inverse().multiply(q_AB)
        numpy_compare.assert_float_equal(q_I.wxyz(), [1., 0, 0, 0])
        numpy_compare.assert_float_equal(
            (q_AB.inverse() @ q_AB).wxyz(), [1., 0, 0, 0])
        v_B = np.array([1., 2, 3])
        v_A = np.array([3., 1, 2])
        numpy_compare.assert_float_allclose(q_AB.multiply(vector=v_B), v_A)
        vlist_B = np.array([v_B, v_B]).T
        vlist_A = np.array([v_A, v_A]).T
        numpy_compare.assert_float_equal(
            q_AB.multiply(vector=vlist_B), vlist_A)

        q_AB_conj = q_AB.conjugate()
        numpy_compare.assert_float_equal(
                q_AB_conj.wxyz(), [0.5, -0.5, -0.5, -0.5])

        numpy_compare.assert_float_equal(
            q_I.slerp(t=0, other=q_I).wxyz(), [1., 0, 0, 0])

        # - Test shaping (#13885).
        v = np.array([0., 0., 0.])
        vs = np.array([[1., 2., 3.], [4., 5., 6.]]).T
        self.assertEqual((q_AB @ v).shape, (3,))
        self.assertEqual((q_AB @ v.reshape((3, 1))).shape, (3, 1))
        self.assertEqual((q_AB @ vs).shape, (3, 2))

        # Test `type_caster`s.
        if T == float:
            value = test_util.create_quaternion()
            self.assertTrue(isinstance(value, mut.Quaternion))
            test_util.check_quaternion(value)

        assert_pickle(self, q_AB, Quaternion.wxyz, T=T)

    @numpy_compare.check_all_types
    def test_isometry3(self, T):
        Isometry3 = mut.Isometry3_[T]
        # - Default constructor
        transform = Isometry3()
        self.assertEqual(numpy_compare.resolve_type(transform.matrix()), T)
        X_I_np = np.eye(4, 4)
        numpy_compare.assert_float_equal(transform.matrix(), X_I_np)
        numpy_compare.assert_float_equal(copy.copy(transform).matrix(), X_I_np)
        if T == float:
            self.assertEqual(str(transform), str(X_I_np))
        # - Constructor with (X_I_np)
        transform = Isometry3(matrix=X_I_np)
        numpy_compare.assert_float_equal(transform.matrix(), X_I_np)
        # - Copy constructor.
        cp = Isometry3(other=transform)
        numpy_compare.assert_equal(transform.matrix(), cp.matrix())
        # - Identity
        transform = Isometry3.Identity()
        numpy_compare.assert_float_equal(transform.matrix(), X_I_np)
        # - Constructor with (R, p)
        R_AB = np.array([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        p_AB = np.array([1., 2, 3])
        X_AB_np = np.eye(4)
        X_AB_np[:3, :3] = R_AB
        X_AB_np[:3, 3] = p_AB
        X_AB = Isometry3(rotation=R_AB, translation=p_AB)
        numpy_compare.assert_float_equal(X_AB.matrix(), X_AB_np)
        numpy_compare.assert_float_equal(X_AB.translation(), p_AB)
        numpy_compare.assert_float_equal(X_AB.rotation(), R_AB)
        # - Setters.
        X_AB = Isometry3()
        X_AB.set_translation(p_AB)
        numpy_compare.assert_float_equal(X_AB.translation(), p_AB)
        X_AB.set_rotation(R_AB)
        numpy_compare.assert_float_equal(X_AB.rotation(), R_AB)
        # - Cast
        self.check_cast(mut.Isometry3_, T)
        # - Check transactions for bad values.
        if T != Expression:
            X_temp = Isometry3(rotation=R_AB, translation=p_AB)
            R_bad = np.copy(R_AB)
            R_bad[0, 0] = 10.
            with self.assertRaises(RuntimeError):
                X_temp.set_rotation(R_bad)
            numpy_compare.assert_float_equal(X_temp.rotation(), R_AB)
            X_bad_np = np.copy(X_I_np)
            X_bad_np[:3, :3] = R_bad
            with self.assertRaises(RuntimeError):
                X_temp.set_matrix(X_bad_np)
            numpy_compare.assert_float_equal(X_temp.matrix(), X_AB_np)
        # Test `type_caster`s.
        if T == float:
            value = test_util.create_isometry()
            self.assertTrue(isinstance(value, mut.Isometry3))
            test_util.check_isometry(value)
        # Operations.
        X_AB = Isometry3(rotation=R_AB, translation=p_AB)
        X_I = X_AB.inverse().multiply(X_AB)
        numpy_compare.assert_float_equal(X_I.matrix(), X_I_np)
        p_BQ = [10, 20, 30]
        p_AQ = [21., -8, 33]
        numpy_compare.assert_float_equal(X_AB.multiply(position=p_BQ), p_AQ)
        p_BQlist = np.array([p_BQ, p_BQ]).T
        p_AQlist = np.array([p_AQ, p_AQ]).T
        numpy_compare.assert_float_equal(
            X_AB.multiply(position=p_BQlist), p_AQlist)
        numpy_compare.assert_float_equal(
            (X_AB.inverse() @ X_AB).matrix(), X_I_np)
        numpy_compare.assert_float_equal(X_AB @ p_BQ, p_AQ)
        # - Test shaping (#13885).
        v = np.array([0., 0., 0.])
        vs = np.array([[1., 2., 3.], [4., 5., 6.]]).T
        self.assertEqual((X_AB @ v).shape, (3,))
        self.assertEqual((X_AB @ v.reshape((3, 1))).shape, (3, 1))
        self.assertEqual((X_AB @ vs).shape, (3, 2))

        assert_pickle(self, X_AB, Isometry3.matrix, T=T)

    def test_translation(self):
        # Test `type_caster`s.
        value = test_util.create_translation()
        self.assertEqual(value.shape, (3,))
        test_util.check_translation(value)

    @numpy_compare.check_all_types
    def test_angle_axis(self, T):
        AngleAxis = mut.AngleAxis_[T]
        value_identity = AngleAxis.Identity()
        self.assertEqual(numpy_compare.resolve_type(value_identity.angle()), T)
        numpy_compare.assert_float_equal(value_identity.angle(), 0.)
        numpy_compare.assert_float_equal(value_identity.axis(), [1., 0, 0])

        # Construct with rotation matrix.
        R = np.array([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        value = AngleAxis(rotation=R)
        numpy_compare.assert_float_allclose(value.rotation(), R)
        numpy_compare.assert_float_allclose(copy.copy(value).rotation(), R)
        numpy_compare.assert_float_allclose(value.inverse().rotation(), R.T)
        numpy_compare.assert_float_allclose(
            value.multiply(value.inverse()).rotation(), np.eye(3))
        numpy_compare.assert_float_allclose(
            (value @ value.inverse()).rotation(), np.eye(3))
        value.set_rotation(np.eye(3))
        numpy_compare.assert_float_equal(value.rotation(), np.eye(3))

        # Construct with quaternion.
        Quaternion = mut.Quaternion_[T]
        q = Quaternion(R)
        value = AngleAxis(quaternion=q)
        numpy_compare.assert_float_allclose(
            value.quaternion().wxyz(), numpy_compare.to_float(q.wxyz()))
        value.set_quaternion(Quaternion.Identity())
        numpy_compare.assert_float_equal(
                value.quaternion().wxyz(), [1., 0, 0, 0])

        # Test setters.
        value = AngleAxis(value_identity)
        value.set_angle(np.pi / 4)
        v = normalize(np.array([0.1, 0.2, 0.3]))
        if T != Expression:
            with self.assertRaises(RuntimeError):
                value.set_axis([0.1, 0.2, 0.3])
        value.set_axis(v)
        numpy_compare.assert_float_equal(value.angle(), np.pi / 4)
        numpy_compare.assert_float_equal(value.axis(), v)

        # Cast.
        self.check_cast(mut.AngleAxis_, T)

        # Test symmetry based on accessors.
        # N.B. `Eigen::AngleAxis` does not disambiguate by restricting internal
        # angles and axes to a half-plane.
        angle = np.pi / 6
        axis = normalize([0.1, 0.2, 0.3])
        value = AngleAxis(angle=angle, axis=axis)
        value_sym = AngleAxis(angle=-angle, axis=-axis)
        numpy_compare.assert_equal(value.rotation(), value_sym.rotation())
        numpy_compare.assert_equal(value.angle(), -value_sym.angle())
        numpy_compare.assert_equal(value.axis(), -value_sym.axis())

        # N.B. AngleAxis does not support multiplication with vectors, so we
        # need not test it here.
        def get_vector(value):
            return np.hstack((value.angle(), value.axis()))

        assert_pickle(self, value, get_vector, T=T)

    @numpy_compare.check_all_types
    def test_value_instantiations(self, T):
        # Existence checks.
        Value[mut.Isometry3_[T]]
