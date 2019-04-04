"""
Provides utilities to test different algebra.
"""

import operator

import numpy as np

import pydrake.math as drake_math


class BaseAlgebra(object):
    # Base class for defining scalar or vectorized (array) algebra and math.
    # Checks on custom types that have numeric relations to `float`.
    # Note that linear algebra itself is not "pluggable" for these operations,
    # so care should be taken when defining it.
    def __init__(self, check_value_impl, scalar_to_float):
        # Derived classes should define extra math functions.
        self._check_value_impl = check_value_impl
        self._scalar_to_float = scalar_to_float
        pass

    def to_algebra(self, scalar):
        # Morphs a scalar to the given algebra (e.g. a scalar or array).
        raise NotImplemented

    def algebra_to_float(self, value):
        # Casts from an algebra of `T` to the same algebra but of type `float`.
        raise NotImplemented

    def check_value(self, actual, expected_scalar):
        # Checks if `actual` is equal to `expected_scalar`, which is
        # broadcasting to match `actual`s shape.
        expected = self.to_algebra(expected_scalar)
        self._check_value_impl(actual, expected)

    def check_logical(self, func, a, b, expected_scalar):
        # Checks logical operations, morphing `expected_scalar` to match
        # `actual`s algebra, and checking that `a` and `b` (of type `T`)
        # have compatible logical operators when the left or right operatnds
        # are `float`s. Specifically, tests:
        # - f(T, T)
        # - f(T, float)
        # - f(float, T)
        expected = self.to_algebra(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        af = self.algebra_to_float(a)
        bf = self.algebra_to_float(b)
        self._check_value_impl(func(a, bf), expected)
        self._check_value_impl(func(af, b), expected)


class ScalarAlgebra(BaseAlgebra):
    # Scalar algebra.
    def __init__(self, check_value_impl, scalar_to_float):
        BaseAlgebra.__init__(self, check_value_impl, scalar_to_float)
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

    def algebra_to_float(self, value):
        return self._scalar_to_float(value)


class VectorizedAlgebra(BaseAlgebra):
    # Vectorized (array) algebra.
    def __init__(self, check_value_impl, scalar_to_float):
        BaseAlgebra.__init__(self, check_value_impl, scalar_to_float)
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

    def algebra_to_float(self, value):
        value_f = np.array([self._scalar_to_float(x) for x in value.flat])
        value_f.reshape(value.shape)
        return value_f
