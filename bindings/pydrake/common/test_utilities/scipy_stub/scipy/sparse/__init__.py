"""A minimal implementation of scipy.sparse that's barely functional enough
to support unit testing the Drake APIs that return Eigen::Sparse matrices.
The pybind11 type caster for Eigen::Sparse maps it to scipy.sparse.
"""

import numpy as np


class csc_matrix:
    """A minimal implementation of the scipy.sparse matrix type.
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.csc_matrix.html
    """

    def __init__(self, arg1, shape=None):
        """The scipy.csc_matrix constructor supports five possible overloads.
        For this stub, we only support these two:
          csc_array((data, indices, indptr), shape=(M, N))
          csc_array(dense)
        """
        if shape is not None:
            (self.data, self.indices, self.indptr) = arg1
            self.shape = shape
        else:
            dense = arg1
            rows, cols = dense.shape
            self.shape = (rows, cols)
            self.data = []
            self.indices = []
            self.indptr = [0]
            for c in range(cols):
                for r in range(rows):
                    value = dense[r, c]
                    if not value:
                        continue
                    self.indices.append(r)
                    self.data.append(value)
                self.indptr.append(len(self.indices))

        self.data = np.asarray(self.data)
        self.indices = np.asarray(self.indices)
        self.indptr = np.asarray(self.indptr)

        # To sanity-check our arguments, convert the data to triplets.
        self._triplets = []
        for col in range(len(self.indptr) - 1):
            start = self.indptr[col]
            end = self.indptr[col+1]
            rows = self.indices[start:end]
            values = self.data[start:end]
            for row, value in zip(rows, values):
                self._triplets.append((row, col, value))

        self.nnz = 0
        for _, _, value in self._triplets:
            if value:
                self.nnz += 1

    def todense(self):
        result = np.zeros(shape=self.shape)
        for row, col, value in self._triplets:
            result[row, col] = value
        return result
