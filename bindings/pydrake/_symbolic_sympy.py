"""This file contains the implementation of to_sympy and from_sympy as used by
`_symbolic_extra.py`. It is loaded is loaded lazily (on demand), so that Drake
does not directly depend on SymPy.
"""

import operator
from typing import Dict, Union

import sympy

import pydrake.symbolic
from pydrake.symbolic import (
    Expression,
    ExpressionKind,
    Formula,
    FormulaKind,
    Variable,
)


def _no_change(x):
    return x


_SYMPY_CONSTRUCTOR = {
    # Leave floats alone -- don't preemptively wrap them in sympy.Float.
    ExpressionKind.Constant: _no_change,
    ExpressionKind.NaN: _no_change,
    # Use the SymPy preferred spelling of bools.
    FormulaKind.True_: lambda: sympy.true,
    FormulaKind.False_: lambda: sympy.false,
    # SymPy doesn't need any extra call to convert a variable to an expression.
    ExpressionKind.Var: _no_change,
    FormulaKind.Var: _no_change,
    # Not implemented yet:
    ExpressionKind.IfThenElse: None,
    ExpressionKind.UninterpretedFunction: None,
    # The rest is all boring stuff.
    ExpressionKind.Abs: sympy.Abs,
    ExpressionKind.Acos: sympy.acos,
    ExpressionKind.Add: sympy.Add,
    ExpressionKind.Asin: sympy.asin,
    ExpressionKind.Atan: sympy.atan,
    ExpressionKind.Atan2: sympy.atan2,
    ExpressionKind.Ceil: sympy.ceiling,
    ExpressionKind.Cos: sympy.cos,
    ExpressionKind.Cosh: sympy.cosh,
    ExpressionKind.Div: operator.truediv,
    ExpressionKind.Exp: sympy.exp,
    ExpressionKind.Floor: sympy.floor,
    ExpressionKind.Log: sympy.log,
    ExpressionKind.Max: sympy.Max,
    ExpressionKind.Min: sympy.Min,
    ExpressionKind.Mul: sympy.Mul,
    ExpressionKind.Pow: sympy.Pow,
    ExpressionKind.Sin: sympy.sin,
    ExpressionKind.Sinh: sympy.sinh,
    ExpressionKind.Sqrt: sympy.sqrt,
    ExpressionKind.Tan:  sympy.tan,
    ExpressionKind.Tanh: sympy.tanh,
    # Not implemented yet.
    FormulaKind.And: None,
    FormulaKind.Eq: None,
    FormulaKind.Forall: None,
    FormulaKind.Geq: None,
    FormulaKind.Gt: None,
    FormulaKind.Isnan: None,
    FormulaKind.Leq: None,
    FormulaKind.Lt: None,
    FormulaKind.Neq: None,
    FormulaKind.Not: None,
    FormulaKind.Or: None,
    FormulaKind.PositiveSemidefinite: None,
}


def _var_to_sympy(drake_var: Variable, *, memo: Dict):
    sympy_var = memo.get(drake_var)
    if sympy_var is None:
        # TODO(jwnimmer-tri) Use drake_type to fill in the assumptions.
        drake_type = drake_var.get_type()
        assumptions = {}
        sympy_var = sympy.Dummy(
            name=drake_var.get_name(),
            dummy_index=drake_var.get_id(),
            **assumptions)
        memo[drake_var] = sympy_var
        memo[sympy_var] = drake_var
    return sympy_var


def _var_from_sympy(sympy_var: sympy.Dummy, *, memo: Dict):
    drake_var = memo.get(sympy_var)
    if drake_var is None:
        raise NotImplementedError()
    return drake_var


def _to_sympy(x: Union[float, bool, Variable, Expression, Formula],
              *,
              memo: Dict) -> sympy.Expr:
    # TODO(jwnimmer-try) Also support Formula, Polynomial, Monomial, etc.
    if isinstance(x, (float, bool)):
        return x
    if isinstance(x, Variable):
        return _var_to_sympy(drake_var=x, memo=memo)
    try:
        kind = x.get_kind()
    except AttributeError as e:
        kind = None
    if kind is None:
        raise NotImplementedError(
            f"Cannot create a SymPy object from the given object {x!r}")
    sympy_constructor = _SYMPY_CONSTRUCTOR.get(kind)
    if sympy_constructor is None:
        raise NotImplementedError(
            f"Cannot create a SymPy object from "
            f"the given pydrake {kind} object {x!r}")
    _, drake_args = x.Unapply()
    sympy_args = [_to_sympy(item, memo=memo) for item in drake_args]
    return sympy_constructor(*sympy_args)


def _from_sympy(x: Union[float, bool, sympy.Expr],
                *,
                memo: Dict) -> Expression:
    if isinstance(x, (float, bool)):
        return x
    if x.is_number:
        return x.evalf()
    sympy_vars = []
    drake_vars = []
    for item in x.atoms():
        if item.is_number:
            continue
        if isinstance(item, sympy.logic.boolalg.BooleanAtom):
            continue
        if isinstance(item, sympy.Dummy):
            sympy_vars.append(item)
            drake_vars.append(_var_from_sympy(item, memo=memo))
            continue
        raise NotImplementedError(f"Unsupported atom {item!r} ({type(item)})")
    return sympy.lambdify(args=sympy_vars, expr=x, docstring_limit=0,
                          modules=[pydrake.symbolic])(*drake_vars)
