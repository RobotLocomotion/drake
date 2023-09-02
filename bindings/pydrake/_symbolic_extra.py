# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import functools
import operator
import typing


def logical_and(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_and, formulas)


def logical_or(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_or, formulas)


def _reduce_add(*args):
    return functools.reduce(operator.add, args)


def _reduce_mul(*args):
    return functools.reduce(operator.mul, args)


# Drake's SymPy support is loaded lazily (on demand), so that Drake does not
# directly depend on SymPy. The implementation lives in `_symbolic_sympy.py`.
_symbolic_sympy = None


def to_sympy(x: typing.Union[float, bool, Variable, Expression, Formula],
             *,
             memo: typing.Dict = None) -> typing.Union[
                 float, bool, 'sympy.Expr']:
    """Converts a pydrake object to the corresponding SymPy Expr.

    Certain expressions are not supported and will raise NotImplementedError.

    Args:
        x: The pydrake object to be converted.
        memo: (Optional) Mapping between Drake variables and SymPy variables.
            Converting a pydrake.symbolic.Variable to SymPy will look up the
            Drake variable as a key in this dictionary. If a value is found,
            then it will be used as the SymPy atom for that Drake variable.
            Otherwise, a new SymPy variable will be created and used, and both
            mappings `{drake_var: sympy_var}` and `{sympy_var: drake_var}` will
            be inserted into `memo`.

    See also :meth:`pydrake.symbolic.from_sympy`.
    """
    global _symbolic_sympy
    if _symbolic_sympy is None:
        from . import _symbolic_sympy
    if memo is None:
        memo = dict()
    return _symbolic_sympy._to_sympy(x, memo=memo)


def from_sympy(x: typing.Union[float, bool, 'sympy.Expr'],
               *,
               memo: typing.Dict = None) -> typing.Union[
                   float, bool, Variable, Expression, Formula]:
    """Converts a SymPy Expr to the corresponding pydrake object.

    Certain expressions are not supported and will raise NotImplementedError.

    Args:
        x: The SymPy object to be converted.
        memo: (Optional) Mapping between SymPy variables and Drake variables.
            Converting a SymPy variable to a pydrake.symbolic.Variable will
            look up the SymPy variable as a key in this dictionary.
            If a value is found, then it will be used as the Drake variable.
            If no value is found, then (for now) raises an exception. In the
            future, we might support automatically creating variables, but
            that is not yet implemented.

    See also :meth:`pydrake.symbolic.to_sympy`.
    """
    global _symbolic_sympy
    if _symbolic_sympy is None:
        from . import _symbolic_sympy
    if memo is None:
        memo = dict()
    return _symbolic_sympy._from_sympy(x, memo=memo)
