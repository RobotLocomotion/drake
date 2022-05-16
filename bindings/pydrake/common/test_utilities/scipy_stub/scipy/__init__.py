"""A minimal implementation of scipy.sparse that's barely functional enough
to support unit testing the Drake APIs that return Eigen::Sparse matrices.
The pybind11 type caster for Eigen::Sparse maps it to scipy.sparse.
"""
