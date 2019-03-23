"""
Provides utilities to test overload algebra.

The actual tests are separated into different processes which test different
loading orders, which guarantees the same behavior regardless of when a module
is imported.

Generally, we would try to unload the modules; however, this does not work well
with pybind11 modules that rely on NumPy.
"""

from __future__ import print_function

import math
import subprocess
import sys
import unittest

# Change this to inspect output.
VERBOSE = False


def debug_print(*args):
    # Prints only if `VERBOSE` is true.
    if VERBOSE:
        print(*args)


def qualname(obj):
    return "{}.{}".format(obj.__module__, obj.__name__)


class Overloads(object):
    # Provides interface for testing function overloads for a given type, `T`.
    def supports(self, func):
        # Determines if `func` is supported by this overload.
        raise NotImplemented

    def to_float(self, y_T):
        # Converts `y_T` (a value of type `T`) to a float.
        raise NotImplemented

    def to_type(self, y_float):
        # Converts `y_float` (a float value) to a value of type `T`.
        raise NotImplemented


class FloatOverloads(Overloads):
    # Imports `math` and provides support for testing `float` overloads.
    def __init__(self):
        import pydrake.math as m
        self.m = m
        self.T = float

    def supports(self, func):
        return True

    def to_float(self, y_T):
        return y_T

    def to_type(self, y_float):
        return y_float


class AutoDiffOverloads(Overloads):
    # Imports `pydrake.autodiffutils` and provides support for testing its
    # overloads.
    def __init__(self):
        import pydrake.autodiffutils as m
        self.m = m
        self.T = m.AutoDiffXd

    def supports(self, func):
        backwards_compat = [
            "cos", "sin",
        ]
        supported = backwards_compat + [
            "log",
            "tan", "asin", "acos", "atan2",
            "sinh", "cosh", "tanh",
        ]
        if func.__name__ in backwards_compat:
            # Check backwards compatibility.
            assert hasattr(self.T, func.__name__)
        return func.__name__ in supported

    def to_float(self, y_T):
        return y_T.value()

    def to_type(self, y_float):
        return self.T(y_float, [])


class SymbolicOverloads(Overloads):
    # Imports `pydrake.symbolic` and provides support for testing its
    # overloads.
    def __init__(self):
        import pydrake.symbolic as m
        self.m = m
        self.T = m.Expression

    def supports(self, func):
        backwards_compat = [
            "log", "abs", "exp", "sqrt",
            "sin", "cos", "tan", "asin", "acos", "atan",
            "sinh", "cosh", "tanh", "ceil", "floor",
            "min", "max", "pow", "atan2",
        ]
        supported = backwards_compat
        if func.__name__ in backwards_compat:
            # Check backwards compatibility.
            assert hasattr(self.m, func.__name__)
        return func.__name__ in supported

    def to_float(self, y_T):
        return y_T.Evaluate()

    def to_type(self, y_float):
        return self.T(y_float)


class MathOverloadsBase(unittest.TestCase):
    """Tests overloads of math functions, specifically ensuring that we will be
    robust against import order."""

    def setUp(self):
        # Explicitly ensure we're loading the modules anew.
        should_be_fresh = [
            "pydrake.autodiffutils",
            "pydrake.math",
            "pydrake.symoblic"
        ]
        for m in should_be_fresh:
            self.assertNotIn(m, sys.modules, m)

    def check_overload(self, overload):
        # TODO(eric.cousineau): Consider comparing against `numpy` ufunc
        # methods.
        import pydrake.math as drake_math
        unary = [
            (drake_math.log, math.log),
            (drake_math.abs, math.fabs),
            (drake_math.exp, math.exp),
            (drake_math.sqrt, math.sqrt),
            (drake_math.sin, math.sin),
            (drake_math.cos, math.cos),
            (drake_math.tan, math.tan),
            (drake_math.asin, math.asin),
            (drake_math.acos, math.acos),
            (drake_math.atan, math.atan),
            (drake_math.sinh, math.sinh),
            (drake_math.cosh, math.cosh),
            (drake_math.tanh, math.tanh),
            (drake_math.ceil, math.ceil),
            (drake_math.floor, math.floor),
        ]
        binary = [
            (drake_math.min, min),
            (drake_math.max, max),
            (drake_math.pow, pow),
            (drake_math.atan2, math.atan2),
        ]

        # Arbitrary values to test overloads with.
        args_float_all = [0.1, 0.2]

        def check_eval(functions, nargs):
            # Generate arguments.
            args_float = args_float_all[:nargs]
            args_T = list(map(overload.to_type, args_float))
            # Check each supported function.
            for f_drake, f_builtin in functions:
                if not overload.supports(f_drake):
                    continue
                debug_print(
                    "- Functions: ", qualname(f_drake), qualname(f_builtin))
                y_builtin = f_builtin(*args_float)
                y_float = f_drake(*args_float)
                debug_print(" - - Float Eval:", repr(y_builtin), repr(y_float))
                self.assertEqual(y_float, y_builtin)
                self.assertIsInstance(y_float, float)
                # Test method current overload, and ensure value is accurate.
                y_T = f_drake(*args_T)
                y_T_float = overload.to_float(y_T)
                debug_print(" - - Overload Eval:", repr(y_T), repr(y_T_float))
                self.assertIsInstance(y_T, overload.T)
                # - Ensure the translated value is accurate.
                self.assertEqual(y_T_float, y_float)

        debug_print("\n\nOverload: ", qualname(type(overload)))
        float_overload = FloatOverloads()
        # Check each number of arguments.
        debug_print("Unary:")
        check_eval(unary, 1)
        debug_print("Binary:")
        check_eval(binary, 2)
