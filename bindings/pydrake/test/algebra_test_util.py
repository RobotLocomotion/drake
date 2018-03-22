"""
Provides utilities to test different algebra.
"""

import numpy as np
import pydrake.math as drake_math


class BaseAlgebra(object):
    # Base class for defining scalar or vectorized (array) algebra and math.
    # checks. Note that linear algebra itself is not "pluggable" for these
    # operations, so care should be taken when defining it.
    def __init__(self):
        # Derived classes should define extra math functions.
        pass

    def to_algebra(self, scalar):
        # Reformats a scalar to the given form.
        raise NotImplemented

    def check_value(self, actual, expected_scalar):
        # Checks if `actual` is equal to `expected_scalar`, which is
        # broadcasting to match `actual`s shape.
        raise NotImplemented

    def check_logical(self, func, a, b, actual, expected_scalar):
        # Checks logical operations (which should be invariant of
        # reverse-operator ordering, e.g. `x < y` should be equivalent to
        # `y > x`), broadcasting `expected_scalar` to match `actual`s shape,
        # and possible permuting on scalar vs. array setup.
        raise NotImplemented


class ScalarAlegbra(BaseAlgebra):
    # Scalar algebra.
    def __init__(self, check_value_impl):
        BaseAlgebra.__init__(self)
        self._check_value_impl = check_value_impl
        # Math functions:
        self.sin = drake_math.sin
        self.cos = drake_math.cos
        self.tan = drake_math.tan
        self.arcsin = drake_math.asin
        self.arccos = drake_math.acos
        self.arctan2 = drake_math.atan2
        self.sinh = drake_math.sinh
        self.cosh = drake_math.cosh
        self.tanh = drake_math.tanh

    def to_algebra(self, scalar):
        return scalar

    def check_value(self, actual, expected_scalar):
        # `actual` should be a scalar.
        self._check_value_impl(actual, expected_scalar)

    def check_logical(self, func, a, b, expected_scalar):
        # Test overloads which have the same return values for:
        # - f(T, T)
        # - f(T, float)
        # - f(float, T)
        expected = self.to_algebra(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        self._check_value_impl(func(a, b.value()), expected)
        self._check_value_impl(func(a.value(), b), expected)


class VectorizedAlgebra(BaseAlgebra):
    # Vectorized (array) algebra.
    def __init__(self, check_value_impl, scalar_to_float):
        BaseAlgebra.__init__(self)
        self._check_value_impl = check_value_impl
        self._scalar_to_float = scalar_to_float
        # Math functions:
        self.sin = np.sin
        self.cos = np.cos
        self.tan = np.tan
        self.arcsin = np.arcsin
        self.arccos = np.arccos
        self.arctan2 = np.arctan2
        self.sinh = np.sinh
        self.cosh = np.cosh
        self.tanh = np.tanh

    def to_algebra(self, scalar):
        return np.array([scalar, scalar])

    def check_value(self, actual, expected_scalar):
        # `actual` should be an array, so ensure `expected` is also an array.
        expected = self.to_algebra(expected_scalar)
        self._check_value_impl(actual, expected)

    def _array_to_float(self, a):
        return np.array(
            [self._scalar_to_float(ai) for ai in a.flat]).reshape(a.shape)

    def check_logical(self, func, a, b, expected_scalar):
        # See `ScalarAlgebra.check_logical`.
        af = self._array_to_float(a)
        bf = self._array_to_float(b)
        expected = self.to_algebra(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        self._check_value_impl(func(a, bf), expected)
        self._check_value_impl(func(af, b), expected)
