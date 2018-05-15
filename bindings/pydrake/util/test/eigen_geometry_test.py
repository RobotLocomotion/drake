import pydrake.util.eigen_geometry as mut

import numpy as np
import unittest

import pydrake.util.test.eigen_geometry_test_util as test_util


def normalize(x):
    return x / np.linalg.norm(x)


class TestEigenGeometry(unittest.TestCase):
    def test_quaternion(self):
        # Simple API.
        q_identity = mut.Quaternion()
        self.assertTrue(np.allclose(q_identity.wxyz(), [1, 0, 0, 0]))
        self.assertTrue(np.allclose(
            q_identity.wxyz(), mut.Quaternion.Identity().wxyz()))
        self.assertEquals(
            str(q_identity), "Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)")
        # Test ordering.
        q_wxyz = normalize([0.1, 0.3, 0.7, 0.9])
        q = mut.Quaternion(w=q_wxyz[0], x=q_wxyz[1], y=q_wxyz[2], z=q_wxyz[3])
        # - Accessors.
        self.assertEquals(q.w(), q_wxyz[0])
        self.assertEquals(q.x(), q_wxyz[1])
        self.assertEquals(q.y(), q_wxyz[2])
        self.assertEquals(q.z(), q_wxyz[3])
        self.assertTrue(np.allclose(q.xyz(), q_wxyz[1:]))
        self.assertTrue(np.allclose(q.wxyz(), q_wxyz))
        # - Mutators.
        q_wxyz_new = q_wxyz[::-1]
        self.assertFalse(np.allclose(q_wxyz, q_wxyz_new))
        q.set_wxyz(wxyz=q_wxyz_new)
        self.assertTrue(np.allclose(q.wxyz(), q_wxyz_new))
        q.set_wxyz(
            w=q_wxyz_new[0], x=q_wxyz_new[1], y=q_wxyz_new[2], z=q_wxyz_new[3])
        self.assertTrue(np.allclose(q.wxyz(), q_wxyz_new))
        # Alternative constructors.
        q_other = mut.Quaternion(wxyz=q_wxyz)
        self.assertTrue(np.allclose(q_other.wxyz(), q_wxyz))
        R = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]])
        q_wxyz_expected = np.array([0.5, 0.5, 0.5, 0.5])
        q_other = mut.Quaternion(q_wxyz_expected)
        self.assertTrue(np.allclose(q_other.rotation(), R))
        R_I = np.eye(3, 3)
        q_other.set_rotation(R_I)
        self.assertTrue(np.allclose(q_other.wxyz(), q_identity.wxyz()))
        # - Copy constructor.
        cp = mut.Quaternion(other=q)
        self.assertTrue(np.allclose(q.wxyz(), cp.wxyz()))
        # Bad values.
        q = mut.Quaternion.Identity()
        # - wxyz
        q_wxyz_bad = [1., 2, 3, 4]
        with self.assertRaises(RuntimeError):
            q.set_wxyz(q_wxyz_bad)
        self.assertTrue(np.allclose(q.wxyz(), [1, 0, 0, 0]))
        # - Rotation.
        R_bad = np.copy(R)
        R_bad[0, 0] = 10
        with self.assertRaises(RuntimeError):
            q_other.set_rotation(R_bad)
        self.assertTrue(np.allclose(q_other.rotation(), R_I))

        # Operations.
        q = mut.Quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
        q_I = q.inverse().multiply(q)
        self.assertTrue(np.allclose(q_I.wxyz(), [1, 0, 0, 0]))

        # Test `type_caster`s.
        value = test_util.create_quaternion()
        self.assertTrue(isinstance(value, mut.Quaternion))
        test_util.check_quaternion(value)

    def test_transform(self):
        # - Default constructor
        transform = mut.Isometry3()
        X = np.eye(4, 4)
        self.assertTrue(np.allclose(transform.matrix(), X))
        self.assertEquals(str(transform), str(X))
        # - Constructor with (X)
        transform = mut.Isometry3(matrix=X)
        self.assertTrue(np.allclose(transform.matrix(), X))
        # - Copy constructor.
        cp = mut.Isometry3(other=transform)
        self.assertTrue(np.allclose(transform.matrix(), cp.matrix()))
        # - Identity
        transform = mut.Isometry3.Identity()
        self.assertTrue(np.allclose(transform.matrix(), X))
        # - Constructor with (R, p)
        R = np.array([
            [0., 1, 0],
            [-1, 0, 0],
            [0, 0, 1]])
        p = np.array([1., 2, 3])
        X = np.vstack((np.hstack((R, p.reshape((-1, 1)))), [0, 0, 0, 1]))
        transform = mut.Isometry3(rotation=R, translation=p)
        self.assertTrue(np.allclose(transform.matrix(), X))
        self.assertTrue(np.allclose(transform.translation(), p))
        transform.set_translation(-p)
        self.assertTrue(np.allclose(transform.translation(), -p))
        self.assertTrue(np.allclose(transform.rotation(), R))
        transform.set_rotation(R.T)
        self.assertTrue(np.allclose(transform.rotation(), R.T))
        # - Check transactions for bad values.
        transform = mut.Isometry3(rotation=R, translation=p)
        R_bad = np.copy(R)
        R_bad[0, 0] = 10.
        with self.assertRaises(RuntimeError):
            transform.set_rotation(R_bad)
        self.assertTrue(np.allclose(R, transform.rotation()))
        X_bad = np.copy(X)
        X_bad[:3, :3] = R_bad
        with self.assertRaises(RuntimeError):
            transform.set_matrix(X_bad)
        self.assertTrue(np.allclose(X, transform.matrix()))
        # Test `type_caster`s.
        value = test_util.create_isometry()
        self.assertTrue(isinstance(value, mut.Isometry3))
        test_util.check_isometry(value)
        # Operations.
        transform = mut.Isometry3(rotation=R, translation=p)
        transform_I = transform.inverse().multiply(transform)
        self.assertTrue(np.allclose(transform_I.matrix(), np.eye(4)))

    def test_translation(self):
        # Test `type_caster`s.
        value = test_util.create_translation()
        self.assertEquals(value.shape, (3,))
        test_util.check_translation(value)
