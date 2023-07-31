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

    def test_wrap_to_match_input_shape(self):
        obj = test_util.MyObject()
        x_1d = np.array([1.0, 2.0])
        x_2d = np.array([[1.0], [2.0]])

        np.testing.assert_equal(obj.PassThroughNoWrap(x_1d), x_1d)
        # Note: Without wrapping, this "warps" the shape.
        np.testing.assert_equal(obj.PassThroughNoWrap(x_2d), x_1d)
        # With wrapping, they match.
        np.testing.assert_equal(obj.PassThroughWithWrap(x_1d), x_1d)
        np.testing.assert_equal(obj.PassThroughWithWrap(x_2d), x_2d)

    def test_wrap_to_match_input_dimension(self):
        obj = test_util.MyObject()
        x_1d = np.zeros(3)
        x_2d = np.zeros((3, 1))

        np.testing.assert_equal(obj.ReturnOnesNoWrap(x_1d), np.ones(2))
        np.testing.assert_equal(obj.ReturnOnesNoWrap(x_2d), np.ones(2))

        np.testing.assert_equal(obj.ReturnOnesWithWrap(x_1d), np.ones(2))
        np.testing.assert_equal(obj.ReturnOnesWithWrap(x_2d), np.ones((2, 1)))
