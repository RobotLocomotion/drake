import numpy as np
import pydrake.math as drake_math


class BaseMath(object):
    # Base class for defining scalar or vectorized (array) math checks.
    def __init__(self):
        # Derived classes should define extra math functions.
        pass

    def reformat(self, scalar):
        # Reformats a scalar to the given form.
        raise NotImplemented

    def check_value(self, actual, expected_scalar):
        raise NotImplemented

    def check_logical(self, actual, expected_scalar):
        raise NotImplemented


class ScalarMath(BaseMath):
    # Basic scalar element math.
    def __init__(self, check_value_impl):
        BaseMath.__init__(self)
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

    def reformat(self, scalar):
        return scalar

    def check_value(self, actual, expected_scalar):
        # `actual` should be a scalar.
        self._check_value_impl(actual, expected_scalar)

    def check_logical(self, func, a, b, expected_scalar):
        # Test overloads which have the same return values for:
        # - f(T, T)
        # - f(T, float)
        # - f(float, T)
        expected = self.reformat(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        self._check_value_impl(func(a, b.value()), expected)
        self._check_value_impl(func(a.value(), b), expected)


class VectorizedMath(BaseMath):
    # Vectorized math for arrays.
    def __init__(self, check_value_impl):
        BaseMath.__init__(self)
        self._check_value_impl = check_value_impl
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

    def reformat(self, scalar):
        return np.array([scalar, scalar])

    def check_value(self, actual, expected_scalar):
        # `actual` should be an array, so ensure `expected` is also an array.
        expected = self.reformat(expected_scalar)
        self._check_value_impl(actual, expected)

    def _array_to_float(self, a):
        return np.array([ai.value() for ai in a.flat]).reshape(a.shape)

    def check_logical(self, func, a, b, expected_scalar):
        # See above.
        af = self._array_to_float(a)
        bf = self._array_to_float(b)
        expected = self.reformat(expected_scalar)
        self._check_value_impl(func(a, b), expected)
        self._check_value_impl(func(a, bf), expected)
        self._check_value_impl(func(af, b), expected)
