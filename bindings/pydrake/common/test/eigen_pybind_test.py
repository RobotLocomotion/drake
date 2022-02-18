import unittest

import numpy as np

import pydrake.common.test.eigen_pybind_test_util as test_util


class TestEigenPybind(unittest.TestCase):
    def test_passing_eigen_ptr(self):
        m0 = np.ones(shape=(2, 3), order="F")
        m1 = test_util.takes_returns_matrix_pointer(m0)
        np.testing.assert_allclose(m0, m1)

    def test_manipulating_eigen_ptr(self):
        """
        Test on a matrix created with numpy
        Matrix values modified in-place,
        but still check the returned matrix
        to validate the C++ -> Python conversion
        """
        m0 = np.ones(shape=(2, 3), order="F")
        np.testing.assert_allclose(m0, 1)

        m1 = test_util.scale_matrix_ptr(m0, 2.0)
        np.testing.assert_allclose(m1, 2.0)
        np.testing.assert_allclose(m1, m0)

        m2 = test_util.scale_matrix_ptr(m1, 3.0)
        np.testing.assert_allclose(m2, 6.0)
        np.testing.assert_allclose(m2, m0)

        m3 = test_util.scale_matrix_ptr(m2, 4.0)
        np.testing.assert_allclose(m3, 24.0)
        np.testing.assert_allclose(m3, m0)

    def test_null_eigen_ptr(self):
        m = test_util.takes_returns_matrix_pointer(None)
        self.assertIsNone(m)
        m = test_util.return_null_ptr()
        self.assertIsNone(m)

    def test_invalid_eigen_ptr(self):
        with self.assertRaises(TypeError):
            test_util.takes_returns_matrix_pointer("drake")

        with self.assertRaises(TypeError):
            test_util.takes_returns_matrix_pointer([0, 1, 2])
