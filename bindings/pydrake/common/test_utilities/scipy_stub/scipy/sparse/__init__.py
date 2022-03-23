"""A minimal implementation of scipy.sparse that's barely functional enough
to support unit testing the Drake APIs that return Eigen::Sparse matrices.
The pybind11 type caster for Eigen::Sparse maps it to scipy.sparse.
"""

import numpy as np


class csc_matrix:
    """A minimal implementation of the scipy.sparse matrix type.
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.csc_matrix.html
    """

    def __init__(self, arg1, shape):
        (self._data, self._indices, self._indptr) = arg1
        self._shape = shape

        # To sanity-check our arguments, convert the data to triplets.
        self._triplets = []
        for col in range(len(self._indptr) - 1):
            start = self._indptr[col]
            end = self._indptr[col+1]
            rows = self._indices[start:end]
            values = self._data[start:end]
            for row, value in zip(rows, values):
                self._triplets.append((row, col, value))

    def todense(self):
        result = np.zeros(shape=self._shape)
        for row, col, value in self._triplets:
            result[row, col] = value
        return result
