import copy
import unittest

import numpy as np

import numpy_dtypes_user_test_util as mut


class TestNumpyDtypesUser(unittest.TestCase):
    def test_scalar_meta(self):
        """Tests basic metadata."""
        self.assertTrue(issubclass(mut.Symbol, np.generic))
        self.assertIsInstance(np.dtype(mut.Symbol), np.dtype)

    def check_scalar(self, actual, expected):
        accepted = (mut.Symbol, mut.StrValueExplicit, mut.LengthValueImplicit)
        if isinstance(actual, accepted):
            self.assertEqual(actual.value(), expected)
        else:
            raise RuntimeError("Invalid scalar: {}".format(repr(expected)))

    def check_array(self, value, expected):
        expected = np.array(expected, dtype=np.object)
        self.assertEqual(value.shape, expected.shape)
        for a, b in zip(value.flat, expected.flat):
            self.check_scalar(a, b)

    def test_scalar_basics(self):
        """
        Tests basics for scalars.
        Important to do since we had to redo the instance registry to inherit
        from `np.generic` :(
        """
        # TODO(eric.cousineau): Consider using `pybind11`s `ConstructorStats`
        # to do instance tracking.
        c1 = mut.Symbol()
        c2 = mut.Symbol()
        self.assertIsNot(c1, c2)
        self.assertIs(c1, c1.self_reference())
        # Test functions.
        a = mut.Symbol("a")
        self.assertEqual(repr(a), "<Symbol 'a'>")
        self.assertEqual(str(a), "a")
        self.assertEqual(a.value(), "a")
        # Copying.
        # N.B. Normally, `pybind11` does not implicitly define copy semantics.
        # However, for these NumPy dtypes it is made implicit (relying on the
        # copy constructor).
        b = copy.copy(a)
        self.assertIsNot(a, b)
        self.assertEqual(a.value(), b.value())
        b = copy.deepcopy(a)
        self.assertIsNot(a, b)
        self.assertEqual(a.value(), b.value())

    def test_array_creation_basics(self):
        # Uniform creation.
        A = np.array([mut.Symbol("a")])
        self.assertEqual(A.dtype, mut.Symbol)
        self.assertEqual(A[0].value(), "a")

    def test_array_cast_explicit(self):
        # Check idempotent round-trip casts.
        A = np.array([mut.Symbol("a")])
        for dtype in (mut.Symbol, np.object, mut.StrValueExplicit):
            B = A.astype(dtype)
            self.assertEqual(B.dtype, dtype)
            C = B.astype(mut.Symbol)
            self.assertEqual(C.dtype, mut.Symbol)
            self.check_scalar(C[0], "a")
        # Check registered explicit casts.
        # - From.
        from_float = np.array([1.]).astype(mut.Symbol)
        self.check_array(from_float, ["float(1)"])
        from_str = np.array([mut.StrValueExplicit("abc")]).astype(mut.Symbol)
        self.check_array(from_str, ["abc"])
        from_length = np.array([mut.LengthValueImplicit(1)]).astype(mut.Symbol)
        self.check_array(from_length, ["length(1)"])
        # - To.
        # N.B. `np.int` may not be the same as `np.int32`; C++ uses `np.int32`.
        to_int = A.astype(np.int32)
        self.assertEqual(to_int[0], 1)
        to_str = A.astype(mut.StrValueExplicit)
        self.check_array(to_str, ["a"])
        to_length = A.astype(mut.LengthValueImplicit)
        self.check_array(to_length, [1])

    def test_array_cast_implicit(self):
        # By assignment.
        a = mut.Symbol("a")
        A = np.array([a])

        def reset():
            A[:] = a

        b_length = mut.LengthValueImplicit(1)
        # - Implicitly convertible types.
        A[0] = b_length
        self.check_array(A, ["length(1)"])
        A[:] = b_length
        self.check_array(A, ["length(1)"])
        # - Permitted as in place operation.
        reset()
        A += mut.LengthValueImplicit(1)
        self.check_array(A, ["(a) + (length(1))"])
        # Explicit: Scalar assignment not permitted.
        b_str = mut.StrValueExplicit("b")
        with self.assertRaises(TypeError):
            A[0] = b_str
        # N.B. For some reason, NumPy considers this explicit coercion...
        A[:] = b_str
        self.check_array(A, ["b"])
        # - Permitted as in place operation.
        reset()
        A += mut.StrValueExplicit("b")
        self.check_array(A, ["(a) + (b)"])
        reset()

    def test_array_creation_mixed(self):
        # Mixed creation with implicitly convertible types.
        with self.assertRaises(TypeError):
            # No type specified, NumPy gets confused.
            O_ = np.array([mut.Symbol(), mut.LengthValueImplicit(1)])
        A = np.array([
            mut.Symbol(), mut.LengthValueImplicit(1)], dtype=mut.Symbol)
        self.check_array(A, ["", "length(1)"])

        # Mixed creation without implicit casts, yields dtype=object.
        O_ = np.array([mut.Symbol(), 1.])
        self.assertEqual(O_.dtype, np.object)
        # - Explicit Cast.
        A = O_.astype(mut.Symbol)
        self.assertEqual(A.dtype, mut.Symbol)
        self.check_array(A, ["", "float(1)"])

        # Mixed creation with explicitly convertible types - does not work.
        with self.assertRaises(TypeError):
            A = np.array([
                mut.Symbol(), mut.StrValueExplicit("a")], dtype=mut.Symbol)

    def test_array_creation_constants(self):
        # Zeros: More so an `empty` array.
        Z = np.full((2,), mut.Symbol())
        self.assertEqual(Z.dtype, mut.Symbol)
        self.check_array(Z, 2 * [""])

        # Zeros: For making an "empty" array, but using float conversion.
        Z_from_float = np.zeros((2,)).astype(mut.Symbol)
        self.check_array(Z_from_float, 2 * ["float(0)"])

        # Ones: Uses float conversion.
        O_from_float = np.ones((2,)).astype(mut.Symbol)
        self.check_array(O_from_float, 2 * ["float(1)"])

        # Linear algebra.
        I_from_float = np.eye(2).astype(mut.Symbol)
        self.check_array(
            I_from_float,
            [["float(1)", "float(0)"], ["float(0)", "float(1)"]])
        self.check_array(np.diag(I_from_float), 2 * ["float(1)"])

    def test_array_creation_constants_bad(self):
        """
        WARNING: The following are all BAD. AVOID THEM (as of NumPy v1.15.2).
        """
        # BAD Memory: `np.empty` works with uninitialized memory.
        # Printing will most likely cause a segfault.
        E = np.empty((2,), dtype=mut.Symbol)
        self.assertEqual(E.dtype, mut.Symbol)
        # BAD Memory: `np.zeros` works by using `memzero`.
        # Printing will most likely cause a segfault.
        Z = np.zeros((2,), dtype=mut.Symbol)
        self.assertEqual(Z.dtype, mut.Symbol)
        # BAD Semantics: This requires that `np.long` be added as an implicit
        # conversion.
        # Could add implicit conversion, but that may wreak havoc.
        with self.assertRaises(ValueError):
            I_ = np.ones((2,), dtype=mut.Symbol)

    def test_array_ufunc(self):
        # - Symbol
        a = mut.Symbol("a")
        b = mut.Symbol("b")
        self.check_scalar(
            mut.custom_binary_ufunc(a, b), "custom-symbol(a, b)")
        A = [a, a]
        B = [b, b]
        self.check_array(
            mut.custom_binary_ufunc(A, B), ["custom-symbol(a, b)"] * 2)

        # Duplicating values for other tests.
        # - LengthValueImplicit
        x_length = mut.LengthValueImplicit(10)
        self.check_scalar(mut.custom_binary_ufunc(x_length, x_length), 20)
        X_length = [x_length, x_length]
        self.check_array(mut.custom_binary_ufunc(X_length, X_length), 2 * [20])
        # - StrValueExplicit
        x_str = mut.StrValueExplicit("x")
        self.check_scalar(
            mut.custom_binary_ufunc(x_str, x_str), "custom-str(x, x)")
        X_str = [x_str, x_str]
        self.check_array(
            mut.custom_binary_ufunc(X_str, X_str), 2 * ["custom-str(x, x)"])

        # - Mixing.
        # N.B. For UFuncs, order affects the resulting output when implicit or
        # explicit convesions are present.
        # - - Symbol + LengthValueImplicit
        self.check_scalar(
            mut.custom_binary_ufunc(x_length, a), 11)
        self.check_array(
            mut.custom_binary_ufunc(X_length, A), 2 * [11])
        self.check_scalar(
            mut.custom_binary_ufunc(a, x_length),
            "custom-symbol(a, length(10))")
        self.check_array(
            mut.custom_binary_ufunc(A, X_length),
            2 * ["custom-symbol(a, length(10))"])
        # - - Symbol + StrValueExplicit
        self.check_scalar(
            mut.custom_binary_ufunc(x_str, a), "custom-str(x, a)")
        self.check_array(
            mut.custom_binary_ufunc(X_str, A), 2 * ["custom-str(x, a)"])
        self.check_scalar(
            mut.custom_binary_ufunc(a, x_str),
            "custom-symbol(a, x)")
        self.check_array(
            mut.custom_binary_ufunc(A, X_str),
            2 * ["custom-symbol(a, x)"])
        # - - Symbol + OperandExplicit
        x_order = mut.OperandExplicit()
        X_order = [x_order, x_order]
        self.check_scalar(
            mut.custom_binary_ufunc(x_order, a), "custom-operand-lhs(a)")
        self.check_array(
            mut.custom_binary_ufunc(X_order, A), 2 * ["custom-operand-lhs(a)"])
        self.check_scalar(
            mut.custom_binary_ufunc(a, x_order), "custom-operand-rhs(a)")
        self.check_array(
            mut.custom_binary_ufunc(A, X_order), 2 * ["custom-operand-rhs(a)"])

    def test_eigen_aliases(self):
        a = mut.Symbol("a")
        A = np.array([a, a])
        mut.add_one(A)
        self.check_scalar(A[0], "(a) + (float(1))")
        # Check reference to live stuff.
        c = mut.SymbolContainer(2, 2)
        mut.add_one(c.symbols())
        self.check_array(c.symbols(), 2 * [2 * ["() + (float(1))"]])

    def check_binary(self, a, b, fop, value):
        """Checks a binary operator for both scalar and array cases."""
        self.check_scalar(fop(a, b), value)
        A = np.array([a, a])
        B = np.array([b, b])
        c1, c2 = fop(A, B)
        self.check_scalar(c1, value)
        self.check_scalar(c2, value)

    def check_binary_with_inplace(
            self, a, b, fop, fiop, value, inplace_same=True):
        """
        Args:
            a: Left-hand operand.
            b: Right-hand operand.
            fop: Binary operator function function (x, y) -> z.
            fiop: Binary operator inplace function (x, y). Must return `x`.
            value: Expected value.
            inplace_same:
                For the scalar case, expects that `a += b` will not implicitly
                create a new instance (per Python's math rules). If False, a
                new instance must be created.
        """
        # Scalar.
        self.check_scalar(fop(a, b), value)
        c = mut.Symbol(a)
        d = fiop(c, b)
        if inplace_same:
            self.assertIs(c, d)
        else:
            self.assertIsNot(c, d)
        self.check_scalar(d, value)

        # Array.
        A = np.array([a, a])
        B = np.array([b, b])
        c1, c2 = fop(A, B)
        self.check_scalar(c1, value)
        self.check_scalar(c2, value)
        C = np.array(A)
        D = fiop(C, B)
        # Regardless of the operation, numpy arrays should not generate
        # temporaries for inplace operations.
        self.assertIs(C, D)
        c1, c2 = C
        self.check_scalar(c1, value)
        self.check_scalar(c2, value)

    def test_algebra_closed(self):
        """Tests scalar and array algebra with implicit conversions."""
        a = mut.Symbol("a")
        b = mut.Symbol("b")

        # Operators.
        def fop(x, y): return x + y

        def fiop(x, y):
            x += y
            return x
        self.check_binary_with_inplace(a, a, fop, fiop, "(a) + (a)")
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) + (b)")

        def fop(x, y): return x - y

        def fiop(x, y):
            x -= y
            return x
        self.check_binary_with_inplace(a, a, fop, fiop, "(a) - (a)")
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) - (b)")

        def fop(x, y): return x * y

        def fiop(x, y):
            x *= y
            return x
        self.check_binary_with_inplace(a, a, fop, fiop, "(a) * (a)")
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) * (b)")

        def fop(x, y):
            return x / y

        def fiop(x, y):
            x /= y
            return x
        self.check_binary_with_inplace(a, a, fop, fiop, "(a) / (a)")
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) / (b)")

        def fop(x, y): return x & y

        def fiop(x, y):
            x &= y
            return x
        self.check_binary_with_inplace(a, a, fop, fiop, "(a) & (a)")
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) & (b)")

        def fop(x, y): return x | y

        def fiop(x, y):
            x |= y
            return x
        self.check_binary_with_inplace(a, a, fop, fiop, "(a) | (a)")
        self.check_binary_with_inplace(a, b, fop, fiop, "(a) | (b)")

        # Logical.
        def fop(x, y): return x == y
        self.check_binary(a, a, fop, "(a) == (a)")
        self.check_binary(a, b, fop, "(a) == (b)")

        def fop(x, y): return x != y
        self.check_binary(a, a, fop, "(a) != (a)")
        self.check_binary(a, b, fop, "(a) != (b)")

        def fop(x, y): return x < y
        self.check_binary(a, a, fop, "(a) < (a)")
        self.check_binary(a, b, fop, "(a) < (b)")

        def fop(x, y): return x <= y
        self.check_binary(a, a, fop, "(a) <= (a)")
        self.check_binary(a, b, fop, "(a) <= (b)")

        def fop(x, y): return x > y
        self.check_binary(a, a, fop, "(a) > (a)")
        self.check_binary(a, b, fop, "(a) > (b)")

        def fop(x, y): return x >= y
        self.check_binary(a, a, fop, "(a) >= (a)")
        self.check_binary(a, b, fop, "(a) >= (b)")

    def test_linear_algebra(self):
        a = mut.Symbol("a")
        b = mut.Symbol("b")
        L = np.array([a, b])
        R = np.array([b, a])
        self.check_scalar(np.dot(L, R), "(() + ((a) * (b))) + ((b) * (a))")
        # Vector.
        L.shape = (1, 2)
        R.shape = (2, 1)
        Y = np.dot(L, R)
        self.assertEqual(Y.shape, (1, 1))
        self.check_scalar(Y[0, 0], "(() + ((a) * (b))) + ((b) * (a))")

    def test_algebra_order_check(self):
        # By construction, `OperandExplicit` only interfaces with `Symbol` by
        # explicit operator overloads; no casting / construction is done.
        a = mut.Symbol("a")
        operand = mut.OperandExplicit()

        def fop(x, y): return x + y

        def fiop(x, y):
            x += y
            return x
        self.check_binary_with_inplace(a, operand, fop, fiop, "(a) + operand")
        self.check_binary(operand, a, fop, "operand + (a)")

    def test_algebra_implicit_casting(self):
        # N.B. Only tested on a single operator, `__add__` and `__iadd__`.
        a = mut.Symbol("a")

        def fop(x, y): return x + y

        def fiop(x, y):
            x += y
            return x

        # N.B. Implicitly convertible types will enable true in-place
        # operations. Explicitly convertible types requires a new value.
        b_length = mut.LengthValueImplicit(1)
        self.check_binary_with_inplace(
            a, b_length, fop, fiop, "(a) + (length(1))", inplace_same=True)

        b_str = mut.StrValueExplicit("b")
        self.check_binary_with_inplace(
            a, b_str, fop, fiop, "(a) + (b)", inplace_same=False)

    # TODO(eric.cousineau): Check trigonometric UFuncs.
