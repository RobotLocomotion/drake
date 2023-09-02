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
    # TODO(jwnimmer-tri) We really should use docstring_limit=0.
    return sympy.lambdify(args=sympy_vars, expr=x,
                          modules=[pydrake.symbolic])(*drake_vars)
