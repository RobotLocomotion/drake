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
        m = np.ones(shape=(2, 3), order="F")
        np.testing.assert_allclose(m, 1)

        test_util.scale_matrix_ptr(m, 2.0)
        np.testing.assert_allclose(m, 2.0)

        test_util.scale_matrix_ptr(m, 3.0)
        np.testing.assert_allclose(m, 6.0)

        test_util.scale_matrix_ptr(m, 4.0)
        np.testing.assert_allclose(m, 24.0)

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
