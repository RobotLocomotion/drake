import pydrake.common.eigen_geometry as mut

import copy
import unittest

import numpy as np
import six

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
import pydrake.common.test.eigen_geometry_test_util as test_util
from pydrake.common.test_utilities import numpy_compare


def normalize(x):
    return x / np.linalg.norm(x)


class TestEigenGeometry(unittest.TestCase):
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
        q = Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        numpy_compare.assert_float_equal(
                q.multiply(position=[1, 2, 3]), [3., 1, 2])
        q_I = q.inverse().multiply(q)
        numpy_compare.assert_float_equal(q_I.wxyz(), [1., 0, 0, 0])
        if six.PY3:
            numpy_compare.assert_float_equal(
                eval("q.inverse() @ q").wxyz(), [1., 0, 0, 0])
        q_conj = q.conjugate()
        numpy_compare.assert_float_equal(
                q_conj.wxyz(), [0.5, -0.5, -0.5, -0.5])

        # Test `type_caster`s.
        if T == float:
            value = test_util.create_quaternion()
            self.assertTrue(isinstance(value, mut.Quaternion))
            test_util.check_quaternion(value)

    @numpy_compare.check_all_types
    def test_isometry3(self, T):
        Isometry3 = mut.Isometry3_[T]
        # - Default constructor
        transform = Isometry3()
        self.assertEqual(numpy_compare.resolve_type(transform.matrix()), T)
        X = np.eye(4, 4)
        numpy_compare.assert_float_equal(transform.matrix(), X)
        numpy_compare.assert_float_equal(copy.copy(transform).matrix(), X)
        if T == float:
            self.assertEqual(str(transform), str(X))
        # - Constructor with (X)
        transform = Isometry3(matrix=X)
        numpy_compare.assert_float_equal(transform.matrix(), X)
        # - Copy constructor.
        cp = Isometry3(other=transform)
        numpy_compare.assert_equal(transform.matrix(), cp.matrix())
        # - Identity
        transform = Isometry3.Identity()
        numpy_compare.assert_float_equal(transform.matrix(), X)
        # - Constructor with (R, p)
        R = np.array([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        p = np.array([1., 2, 3])
        X = np.vstack((np.hstack((R, p.reshape((-1, 1)))), [0, 0, 0, 1]))
        transform = Isometry3(rotation=R, translation=p)
        numpy_compare.assert_float_equal(transform.matrix(), X)
        numpy_compare.assert_float_equal(transform.translation(), p)
        transform.set_translation(-p)
        numpy_compare.assert_float_equal(transform.translation(), -p)
        numpy_compare.assert_float_equal(transform.rotation(), R)
        transform.set_rotation(R.T)
        numpy_compare.assert_float_equal(transform.rotation(), R.T)
        # - Check transactions for bad values.
        if T != Expression:
            transform = Isometry3(rotation=R, translation=p)
            R_bad = np.copy(R)
            R_bad[0, 0] = 10.
            with self.assertRaises(RuntimeError):
                transform.set_rotation(R_bad)
            numpy_compare.assert_float_equal(transform.rotation(), R)
            X_bad = np.copy(X)
            X_bad[:3, :3] = R_bad
            with self.assertRaises(RuntimeError):
                transform.set_matrix(X_bad)
            numpy_compare.assert_float_equal(transform.matrix(), X)
        # Test `type_caster`s.
        if T == float:
            value = test_util.create_isometry()
            self.assertTrue(isinstance(value, mut.Isometry3))
            test_util.check_isometry(value)
        # Operations.
        transform = Isometry3(rotation=R, translation=p)
        transform_I = transform.inverse().multiply(transform)
        numpy_compare.assert_float_equal(transform_I.matrix(), np.eye(4))
        numpy_compare.assert_float_equal(
            transform.multiply(position=[10, 20, 30]), [21., -8, 33])
        if six.PY3:
            numpy_compare.assert_float_equal(
                eval("transform.inverse() @ transform").matrix(), np.eye(4))
            numpy_compare.assert_float_equal(
                eval("transform @ [10, 20, 30]"), [21., -8, 33])

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
        if six.PY3:
            numpy_compare.assert_float_allclose(
                eval("value @ value.inverse()").rotation(), np.eye(3))
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
