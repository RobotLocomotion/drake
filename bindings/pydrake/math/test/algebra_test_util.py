"""
Provides utilities to test different algebra.
"""

import operator

import numpy as np

import pydrake.math as drake_math


class BaseAlgebra:
    # Base class for defining scalar or vectorized (array) algebra and math.
    # Checks on custom types that have numeric relations to `float`.
    # Note that linear algebra itself is not "pluggable" for these operations,
    # so care should be taken when defining it.
    def __init__(self):
        # Derived classes should define extra math functions.
        pass

    def to_algebra(self, scalar):
        # Morphs a scalar to the given algebra (e.g. a scalar or array).
        raise NotImplemented


class ScalarAlgebra(BaseAlgebra):
    # Scalar algebra.
    def __init__(self):
        BaseAlgebra.__init__(self)
        # Math functions:
        self.log = drake_math.log
        self.abs = drake_math.abs
        self.exp = drake_math.exp
        self.sqrt = drake_math.sqrt
        self.pow = drake_math.pow
        self.sin = drake_math.sin
        self.cos = drake_math.cos
        self.tan = drake_math.tan
        self.arcsin = drake_math.asin
        self.arccos = drake_math.acos
        self.arctan2 = drake_math.atan2
        self.sinh = drake_math.sinh
        self.cosh = drake_math.cosh
        self.tanh = drake_math.tanh
        self.min = drake_math.min
        self.max = drake_math.max
        self.ceil = drake_math.ceil
        self.floor = drake_math.floor
        # Comparison. Defined in same order as in `_math_extra.py`.
        self.lt = operator.lt
        self.le = operator.le
        self.eq = operator.eq
        self.ne = operator.ne
        self.ge = operator.ge
        self.gt = operator.gt

    def to_algebra(self, scalar):
        return scalar


class VectorizedAlgebra(BaseAlgebra):
    # Vectorized (array) algebra.
    def __init__(self):
        BaseAlgebra.__init__(self)
        # Math functions:
        self.log = np.log
        self.abs = np.abs
        self.exp = np.exp
        self.sqrt = np.sqrt
        self.pow = np.power
        self.sin = np.sin
        self.cos = np.cos
        self.tan = np.tan
        self.arcsin = np.arcsin
        self.arccos = np.arccos
        self.arctan2 = np.arctan2
        self.sinh = np.sinh
        self.cosh = np.cosh
        self.tanh = np.tanh
        self.min = np.fmin  # N.B. Do not use `np.min`
        self.max = np.fmax
        self.ceil = np.ceil
        self.floor = np.floor
        # Comparison. Defined in same order as in `_math_extra.py`.
        self.lt = drake_math.lt
        self.le = drake_math.le
        self.eq = drake_math.eq
        self.ne = drake_math.ne
        self.ge = drake_math.ge
        self.gt = drake_math.gt

    def to_algebra(self, scalar):
        return np.array([scalar, scalar])
