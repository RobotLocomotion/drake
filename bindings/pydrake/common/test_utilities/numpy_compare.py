"""
Provides consistent utilities for comparing NumPy matrices and scalars.

Prefer comparisons in the following order:

 - Methods from this module.
 - Methods from `np.testing.*`, if the dtypes are guaranteed to be NumPy
   builtins and the API definitely won't support other scalar types.
"""

# TODO(eric.cousineau): Make custom assert-vectorize which will output
# coordinates and stuff.

from contextlib import contextmanager
from collections import namedtuple
import functools
from itertools import product

import numpy as np

from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import (
    Expression, Formula, Monomial, Polynomial, Variable)
from pydrake.polynomial import Polynomial_ as RawPolynomial_


class _UnwantedEquality(AssertionError):
    pass


class _Registry:
    # Scalar comparator.
    # `assert_eq` will be vectorized; it should raise an assertion error upon
    # first inequality.
    # `assert_ne` will stay as scalar; this should raise `_UnwantedEquality` to
    # make intent explicit.
    # TODO(eric.cousineau): Add `assert_near` when it's necessary.
    AssertComparator = namedtuple(
        'AssertComparator', ['assert_eq', 'assert_ne', 'assert_allclose'])

    def __init__(self):
        self._comparators = {}
        self._to_float = {}

    def register_comparator(
        self,
        cls_a,
        cls_b,
        assert_eq,
        assert_ne=None,
        assert_allclose=None,
    ):
        key = (cls_a, cls_b)
        assert key not in self._comparators, key
        assert_eq = np.vectorize(assert_eq)
        self._comparators[key] = self.AssertComparator(
            assert_eq, assert_ne, assert_allclose
        )

    def get_comparator_from_arrays(self, a, b):
        # Ensure all types are homogeneous.
        key = (resolve_type(a), resolve_type(b))
        return self._comparators[key]

    def register_to_float(self, cls, func):
        assert cls not in self._to_float, cls
        self._to_float[cls] = func

    def get_to_float(self, cls):
        return self._to_float[cls]


@np.vectorize
def to_float(x):
    """Converts scalar or array to floats."""
    x = np.asarray(x)
    if x.dtype == object:
        x = x.item()
        cls = type(x)
        return _registry.get_to_float(cls)(x)
    else:
        return np.float64(x)


def assert_equal(a, b):
    """
    Compare scalars or arrays directly, requiring equality.

    For non-object dtypes, this has the same behavior as
    ``np.testing.assert_equal``, namely that two NaNs being in the same
    positions in an array implies equality, rather than NaN being compared as
    numbers.
    """
    a, b = map(np.asarray, (a, b))
    if a.size == 0 and b.size == 0:
        return
    if a.dtype != object and b.dtype != object:
        np.testing.assert_equal(a, b)
    else:
        assert_eq = _registry.get_comparator_from_arrays(a, b).assert_eq
        assert assert_eq is not None
        assert_eq(a, b)


def assert_allclose(a, b, atol=1e-15, rtol=0):
    """
    Same as `assert_equal`, but also checks for tolerances when possible.
    """
    a, b = map(np.asarray, (a, b))
    if a.size == 0 and b.size == 0:
        return
    if a.dtype != object and b.dtype != object:
        np.testing.assert_allclose(a, b, atol=atol, rtol=rtol)
    else:
        assert_allclose = (
            _registry.get_comparator_from_arrays(a, b).assert_allclose
        )
        assert assert_allclose is not None
        assert_allclose(a, b, atol=atol, rtol=rtol)


def _raw_eq(a, b):
    assert a == b, (a, b)


def _raw_ne(a, b):
    if a == b:
        raise _UnwantedEquality(str((a, b)))


def assert_not_equal(a, b):
    """Compare scalars or arrays directly, requiring inequality."""
    a, b = map(np.asarray, (a, b))
    assert not (a.size == 0 and b.size == 0)
    if a.dtype != object and b.dtype != object:
        assert_ne = _raw_ne
    else:
        assert_ne = _registry.get_comparator_from_arrays(a, b).assert_ne
    assert assert_ne is not None
    # For this to fail, all items must have failed.
    br = np.broadcast(a, b)
    errs = []
    for ai, bi in br:
        e = None
        try:
            assert_ne(ai, bi)
        except _UnwantedEquality as e:
            errs.append(str(e))
    all_equal = len(errs) == br.size
    if all_equal:
        raise AssertionError("Unwanted equality: {}".format(errs))


def assert_float_equal(a, bf):
    """Checks equality of `a` (non-float) and `bf` (float) by converting `a` to
    floats.

    See also: ``assert_equal``
    """
    assert np.asarray(bf).dtype == float, np.asarray(bf).dtype
    af = to_float(a)
    assert_equal(af, bf)


def assert_float_not_equal(a, bf):
    """Checks inequality of `a` (non-float) and `bf` (float) by converting `a`
    to floats."""
    assert np.asarray(bf).dtype == float, np.asarray(bf).dtype
    af = to_float(a)
    assert_not_equal(af, bf)


def assert_float_allclose(a, bf, atol=1e-15, rtol=0):
    """Checks nearness of `a` (non-float) to `bf` (float) by converting `a` to
    floats.

    Note:
        This uses default tolerances that are different from
        `np.testing.assert_allclose`.
    """
    assert np.asarray(bf).dtype == float
    af = to_float(a)
    np.testing.assert_allclose(af, bf, atol=atol, rtol=rtol)


def resolve_type(a):
    """Resolves the type of an array `a`; useful for dtype=object. This will
    ensure the entire array has homogeneous type, and will return the class of
    the first item. `a` cannot be empty.
    """
    a = np.asarray(a)
    assert a.size != 0, "Cannot be empty."
    cls_set = {type(np.asarray(x).item()) for x in a.flat}
    assert len(cls_set) == 1, (
        "Types must be homogeneous; got: {}".format(cls_set))
    cls, = cls_set
    return cls


def _str_eq(a, b):
    # b is a string, a is to be converted.
    a = str(a)
    assert a == b, (a, b)


def _str_ne(a, b):
    # b is a string, a is to be converted.
    a = str(a)
    if a == b:
        raise _UnwantedEquality(str((a, b)))


def _register_autodiff():

    def autodiff_eq(a, b):
        assert a.value() == b.value(), (a.value(), b.value())
        np.testing.assert_equal(a.derivatives(), b.derivatives())

    def autodiff_ne(a, b):
        if (a.value() == b.value()
                and (a.derivatives() == b.derivatives()).all()):
            raise _UnwantedEquality(str(a.value(), b.derivatives()))

    def autodiff_allclose(a, b, atol, rtol):
        # TODO(eric.cousineau): Figure out why `.item()` is necessary here, but
        # not above.
        a = a.item()
        b = b.item()
        np.testing.assert_allclose(a.value(), b.value(), atol=atol, rtol=rtol)
        np.testing.assert_allclose(
            a.derivatives(), b.derivatives(), atol=atol, rtol=rtol
        )

    _registry.register_to_float(AutoDiffXd, AutoDiffXd.value)
    _registry.register_comparator(
        AutoDiffXd, AutoDiffXd, autodiff_eq, autodiff_ne, autodiff_allclose)


def _register_symbolic():

    def sym_struct_eq(a, b):
        assert a.EqualTo(b), (a, b)

    def sym_struct_ne(a, b):
        assert not a.EqualTo(b), (a, b)

    def from_bool(x):
        assert isinstance(x, (bool, np.bool_)), type(x)
        if x:
            return Formula.True_()
        else:
            return Formula.False_()

    def formula_bool_eq(a, b):
        return sym_struct_eq(a, from_bool(b))

    def formula_bool_ne(a, b):
        return sym_struct_ne(a, from_bool(b))

    _registry.register_to_float(Expression, Expression.Evaluate)
    _registry.register_comparator(Formula, str, _str_eq, _str_ne)
    _registry.register_comparator(
        Formula, Formula, Formula.__eq__, Formula.__ne__)
    # Ensure that we can do simple boolean comparison, e.g. in lieu of
    # `unittest.TestCase.assertTrue`, use
    # `numpy_compare.assert_equal(f, True)`.
    _registry.register_comparator(
        Formula, bool, formula_bool_eq, formula_bool_ne)
    lhs_types = [Variable, Expression, Polynomial, Monomial]
    rhs_types = lhs_types + [float]
    for lhs_type in lhs_types:
        _registry.register_comparator(lhs_type, str, _str_eq, _str_ne)
    for lhs_type, rhs_type in product(lhs_types, rhs_types):
        _registry.register_comparator(
            lhs_type, rhs_type, sym_struct_eq, sym_struct_ne)


def _register_polynomial():
    _registry.register_comparator(
        RawPolynomial_[float], RawPolynomial_[float], _raw_eq, _raw_ne)
    _registry.register_comparator(
        RawPolynomial_[AutoDiffXd], RawPolynomial_[AutoDiffXd], _raw_eq,
        _raw_ne)
    _registry.register_comparator(
        RawPolynomial_[Expression], RawPolynomial_[Expression], _raw_eq,
        _raw_ne)


# Globals.
_registry = _Registry()
_register_autodiff()
_register_symbolic()
_register_polynomial()


def check_all_types(check_func):
    """Decorator to call a function multiple times with `T={type}`, where
    `type` covers all (common) scalar types for Drake."""

    @functools.wraps(check_func)
    def wrapper(*args, **kwargs):
        check_func(*args, T=float, **kwargs)
        check_func(*args, T=AutoDiffXd, **kwargs)
        check_func(*args, T=Expression, **kwargs)

    return wrapper


def check_nonsymbolic_types(check_func):
    """Decorator to call a function multiple types with `T={type}`, where
    `type` covers all (common) non-symbolic scalar types for Drake."""

    @functools.wraps(check_func)
    def wrapper(*args, **kwargs):
        check_func(*args, T=float, **kwargs)
        check_func(*args, T=AutoDiffXd, **kwargs)

    return wrapper


@contextmanager
def soft_sub_test(hint_for_error):
    """
    Prints out a message when an exception is raised.

    Useful for providing debug information for combinatorics tests.

    With unittest's ``--failfast`` option (at least on Python 3.6), an error in
    ``unittest.TestCase.subTest`` does not stop execution, making it difficult
    to digest errors. We use this workaround instead.
    """
    try:
        yield
    except Exception:
        print(f"soft_sub_test failure:\n  {hint_for_error}")
        raise
