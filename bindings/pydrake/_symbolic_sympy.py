"""This file contains the implementation of to_sympy and from_sympy as used by
`_symbolic_extra.py`. It is loaded is loaded lazily (on demand), so that Drake
does not # directly depend on SymPy.
"""

from typing import Dict, Union

import sympy

import pydrake.symbolic
from pydrake.symbolic import (
    Expression,
    Formula,
    ExpressionKind,
    Variable,
)
from pydrake.symbolic import _reduce_add, _reduce_mul


def _no_change(x):
    return x


_SYMPY_CONSTRUCTOR = {
    # Leave floats alone -- don't preemptively wrap them in sympy.Float.
    ExpressionKind.Constant: _no_change,
    # SymPy doesn't need any extra call to convert a variable to an expression.
    ExpressionKind.Var: _no_change,
    # The rest is all boring stuff.
    ExpressionKind.Add: sympy.Add,
    ExpressionKind.Mul: sympy.Mul,
}


# TODO(jwnimmer-tri) Unused.
_DRAKE_CONSTRUCTOR = {
    # Leave floats alone -- don't preemptively wrap them in an Expression.
    float: _no_change,
    sympy.Float: _no_change,
    # Leave variables alone -- don't wrap them in an Expression; they will be
    # promoted to Expression later if need be.
    sympy.Dummy: _no_change,
    # The rest is all boring stuff.
    sympy.Add: _reduce_add,
    sympy.Mul: _reduce_mul,
}


def _var_to_sympy(drake_var: Variable, *, memo: Dict):
    sympy_var = memo.get(drake_var)
    if sympy_var is None:
        sympy_var = sympy.Dummy(
            name=drake_var.get_name(),
            dummy_index=drake_var.get_id())
        memo[drake_var] = sympy_var
        memo[sympy_var] = drake_var
    return sympy_var


def _var_from_sympy(sympy_var: sympy.Dummy, *, memo: Dict):
    drake_var = memo.get(sympy_var)
    if drake_var is None:
        raise NotImplementedError()
    return drake_var


def _to_sympy(x: Union[float, Variable, Expression],
              *,
              memo: Dict) -> sympy.Expr:
    # TODO(jwnimmer-try) Also support Formula, Polynomial, Monomial, etc.
    if isinstance(x, float):
        return x
    if isinstance(x, Variable):
        return _var_to_sympy(drake_var=x, memo=memo)
    kind = x.get_kind()
    sympy_constructor = _SYMPY_CONSTRUCTOR.get(kind)
    if sympy_constructor is None:
        raise NotImplementedError(
            f"Cannot create a SymPy object from "
            f"the given pydrake {kind} object {x!r}")
    _, drake_args = x.Unapply()
    sympy_args = [_to_sympy(item, memo=memo) for item in drake_args]
    return sympy_constructor(*sympy_args)


# TODO(jwnimmer-tri) Probably we won't need do it this way.
# Keep around as a fallback for now?
def _unnecessary_recursive_from_sympy(x: Union[float, sympy.Expr],
                                      *,
                                      memo: Dict) -> Expression:
    if isinstance(x, float):
        return x
    drake_constructor = _DRAKE_CONSTRUCTOR.get(type(x))
    if drake_constructor is None:
        raise NotImplementedError(
            f"Cannot create a pydrake object from "
            f"the given SymPy {type(x)} object {x!r}")
    if x.is_number:
        drake_args = [x.evalf()]
    elif isinstance(x, sympy.Dummy):
        drake_args = [_var_from_sympy(x, memo=memo)]
    else:
        sympy_args = x.args
        drake_args = [_from_sympy(item, memo=memo) for item in sympy_args]
    return drake_constructor(*drake_args)


def _from_sympy(x: Union[float, sympy.Expr],
                *,
                memo: Dict) -> Expression:
    if isinstance(x, float):
        return x
    if x.is_number:
        return x.evalf()
    sympy_vars = []
    drake_vars = []
    for item in x.atoms():
        if item.is_number:
            continue
        if isinstance(item, sympy.Dummy):
            sympy_vars.append(item)
            drake_vars.append(_var_from_sympy(item, memo=memo))
            continue
        raise NotImplementedError("Unsupported atom {item!r}")
    return sympy.lambdify(sympy_vars, x, pydrake.symbolic)(*drake_vars)
