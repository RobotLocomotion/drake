from __future__ import print_function

import math
import subprocess
import sys
import unittest

# N.B. Do not import modules here, as we want to remove them as part of the
# test teardown.

# Change this to inspect output.
verbose = False


def debug_print(*args):
    # Prints only if `verbose` is true.
    if verbose:
        print(*args)


class Overloads(object):
    # Provides interface for testing function overloads.
    def get_test_values(self):
        raise NotImplemented

    def supports(self, func):
        raise NotImplemented

    def to_float(self, y_type):
        raise NotImplemented

    def eval(self, func, nargs):
        args = self.get_test_values()[:nargs]
        return func(*args)


class FloatOverloads(Overloads):
    # Imports `math` and provides support for testing `float` overloads.
    def __init__(self):
        from pydrake import math as drake_math
        self.m = drake_math
        self.type = float

    def get_test_values(self):
        return (0.1, 0.2)

    def supports(self, func):
        return True

    def to_float(self, y_type):
        return y_type


class AutoDiffOverloads(Overloads):
    # Imports `pydrake.autodiffutils` and provides support for testing its
    # overloads.
    def __init__(self):
        from pydrake import autodiffutils
        self.m = autodiffutils
        self.type = self.m.AutoDiffXd

    def get_test_values(self):
        a, b = FloatOverloads().get_test_values()
        return (self.type(a, []), self.type(b, []))

    def supports(self, func):
        backwards_compat = [
            'cos', 'sin',
        ]
        if func.__name__ in backwards_compat:
            # Check backwards compatibility.
            assert hasattr(self.type, func.__name__)
            return True
        supported = [
            "tan", "asin", "acos", "atan2",
            "sinh", "cosh", "tanh",
        ]
        return func.__name__ in supported

    def to_float(self, y_type):
        return y_type.value()


class SymbolicOverloads(Overloads):
    # Imports `pydrake.symbolic` and provides support for testing its
    # overloads.
    def __init__(self):
        from pydrake import symbolic
        self.m = symbolic
        self.type = self.m.Expression

    def get_test_values(self):
        a, b = FloatOverloads().get_test_values()
        return (self.type(a), self.type(b))

    def supports(self, func):
        backwards_compat = [
            'abs', 'exp', 'sqrt',
            'sin', 'cos', 'tan', 'asin', 'acos', 'atan',
            'sinh', 'cosh', 'tanh', 'ceil', 'floor',
            'min', 'max', 'pow', 'atan2',
        ]
        if func.__name__ in backwards_compat:
            # Check backwards compatibility.
            assert hasattr(self.m, func.__name__)
            return True
        return False

    def to_float(self, y_type):
        return y_type.Evaluate()


class TestMathOverloads(unittest.TestCase):
    """Tests overloads of math functions, specifically ensuring that we will be
    robust against import order."""

    def _check_overload(self, overload):
        import pydrake.math as drake_math
        unary = [
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

        def check_eval(functions, nargs):

            def float_eval(f):
                return float_overload.eval(f, nargs)

            def overload_eval(f):
                return overload.eval(f, nargs)

            for f_drake, f_builtin in functions:
                if not overload.supports(f_drake):
                    continue
                debug_print("- Functions: ", f_drake, f_builtin)
                y_builtin = float_eval(f_builtin)
                y_float = float_eval(f_drake)
                debug_print(" - - Float Eval:", y_builtin, y_float)
                self.assertEquals(y_float, y_builtin)
                self.assertIsInstance(y_float, float)
                # Test method current overload, and ensure value is accurate.
                y_type = overload_eval(f_drake)
                y_type_float = overload.to_float(y_type)
                debug_print(" - - Overload Eval:", y_type, y_type_float)
                self.assertIsInstance(y_type, overload.type)
                # - Ensure the translated value is accurate.
                self.assertEquals(y_type_float, y_float)

        debug_print("\n\nOverload: ", overload)
        float_overload = FloatOverloads()
        # Check each number of arguments.
        debug_print("Unary:")
        check_eval(unary, 1)
        debug_print("Binary:")
        check_eval(binary, 2)

    def _check_overloads(self, order):
        for overload_cls in order:
            self._check_overload(overload_cls())

    def test_overloads(self):
        # Each of these orders implies the relevant module is imported in this
        # test, in the order specified. This is done to guarantee that the
        # cross-module overloading does not affect functionality.
        orders = [
            (FloatOverloads,),
            (SymbolicOverloads,),
            (AutoDiffOverloads,),
            (AutoDiffOverloads, SymbolicOverloads),
            (SymbolicOverloads, AutoDiffOverloads),
        ]
        # At present, the `pybind11` modules appear not to be destroyed, even
        # when we try to completely dergister them and garbage collect.
        # To keep this test self-contained, we will just reinvoke this test
        # with the desired order so we can control which modules get imported.
        if len(sys.argv) == 1:
            # We have arrived here from a direct call. Call the specified
            # ordering.
            for i in range(len(orders)):
                args = [sys.executable, sys.argv[0], str(i)]
                subprocess.check_call(args)
        else:
            self.assertEquals(len(sys.argv), 2)
            i = int(sys.argv[1])
            self._check_overloads(orders[i])
