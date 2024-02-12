import unittest

import numpy as np
import scipy.sparse


class Test(unittest.TestCase):
    def test_scipy_import(self):
        self.assertIn("/scipy_stub/", scipy.__file__)

    def test_scipy_sparse_ctor(self):
        A_dense = np.array([
            [1.0, 2.0, 0.0],
            [0.0, 3.0, 4.0],
        ])

        data = [1.0, 2.0, 3.0, 4.0]
        indices = [0, 0, 1, 1]
        indptr = [0, 1, 3, 4]
        shape = (2, 3)

        def check_matrix(A_sparse):
            np.testing.assert_equal(A_sparse.data, data)
            np.testing.assert_equal(A_sparse.indices, indices)
            np.testing.assert_equal(A_sparse.indptr, indptr)
            self.assertEqual(A_sparse.shape, shape)
            self.assertEqual(A_sparse.nnz, 4)
            np.testing.assert_equal(A_dense, A_sparse.todense())

        A_sparse = scipy.sparse.csc_matrix(
            (data, indices, indptr), shape=shape,
        )
        check_matrix(A_sparse)

        A_sparse_2 = scipy.sparse.csc_matrix(A_dense)
        check_matrix(A_sparse_2)
