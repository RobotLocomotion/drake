"""This file contains the implementation of to_sympy and from_sympy as used by
`_symbolic_extra.py`. It is loaded is loaded lazily (on demand), so that Drake
does not directly depend on SymPy.
"""

import operator
from typing import Dict, Union

import sympy
from sympy.printing.pycode import MpmathPrinter

import pydrake.symbolic
from pydrake.symbolic import (
    Expression,
    ExpressionKind,
    Formula,
    FormulaKind,
    Variable,
)


def _no_change(x):
    """This function simply returns its argument. It is used below in the table
    of constructors to signify that no type-conversion is required.
    """
    return x


def _make_sympy_if_then_else(cond: sympy.Expr,
                             expr_then: sympy.Expr,
                             expr_else: sympy.Expr):
    """Returns the SymPy spelling of `{then_expr} if {cond} else {expr_else}`,
    also known as the "ternary conditional" operator.
    """
    # N.B. We can't use sympy.ITE -- that operates on three booleans; the Drake
    # if_then_else operates on (bool, float, float).
    return sympy.Piecewise((expr_then, cond), (expr_else, True))


# This table maps from ExpressionKind and FormulaKind to the SymPy constructor
# for that kind of term; this forms the core of `pydrake.symbolic.to_sympy()`.
_SYMPY_CONSTRUCTOR = {
    # Leave floats alone -- don't preemptively wrap them in sympy.Float.
    ExpressionKind.Constant: _no_change,
    # Like Drake's NaN, the SymPy NaN also rejects, e.g., comparison operators.
    # That's more like what we want than `math.nan`, which allows comparison.
    ExpressionKind.NaN: lambda _: sympy.nan,
    # Use the SymPy preferred spelling of bools.
    FormulaKind.True_: lambda: sympy.true,
    FormulaKind.False_: lambda: sympy.false,
    # SymPy doesn't need any extra call to convert a variable to an expression.
    ExpressionKind.Var: _no_change,
    FormulaKind.Var: _no_change,
    # The rest is all boring stuff, in alphabetical order.
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
    ExpressionKind.IfThenElse: _make_sympy_if_then_else,
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
    FormulaKind.And: sympy.And,
    FormulaKind.Eq: sympy.Equality,
    FormulaKind.Geq: sympy.GreaterThan,
    FormulaKind.Gt: sympy.StrictGreaterThan,
    FormulaKind.Leq: sympy.LessThan,
    FormulaKind.Lt: sympy.StrictLessThan,
    FormulaKind.Neq: sympy.Unequality,
    FormulaKind.Not: sympy.Not,
    FormulaKind.Or: sympy.Or,
    # We don't support converting these kinds of terms (yet).
    ExpressionKind.UninterpretedFunction: None,
    FormulaKind.Forall: None,
    FormulaKind.Isnan: None,
    FormulaKind.PositiveSemidefinite: None,
}


def _var_to_sympy(drake_var: Variable, *, memo: Dict):
    """Converts a Drake variable into a SymPy Variable.

    If the Drake variable was already in the `memo` dict, returns the value in
    the dict. Otherwise, creates and returns a new sympy.Dummy() variable.
    """
    sympy_var = memo.get(drake_var.get_id())
    if sympy_var is None:
        drake_type = drake_var.get_type()
        assumptions = {
            # TODO(jwnimmer-tri) Use drake_type to fill in the assumptions.
        }
        sympy_var = sympy.Dummy(
            name=drake_var.get_name(),
            dummy_index=drake_var.get_id(),
            **assumptions)
        memo[drake_var.get_id()] = sympy_var
        memo[sympy_var] = drake_var
    return sympy_var


def _var_from_sympy(sympy_var: sympy.Dummy, *, memo: Dict):
    """Converts a SymPy variable into a Drake Variable.

    At the moment, the *only* supported conversion is to look up the variable
    in `memo`, i.e., the user may pre-define the mapping.

    TODO(jwnimmer-tri) This could probably have a better fallback plan.
    """
    drake_var = memo.get(sympy_var)
    if drake_var is None:
        raise NotImplementedError(
            f"The SymPy variable {sympy_var} is missing from the `memo` dict.")
    return drake_var


def _to_sympy(
    x: Union[float, int, bool, Variable, Expression, Formula],
    *,
    memo: Dict = None
) -> Union[float, int, bool, sympy.Expr]:
    """This is the private implementation of pydrake.symbolic.to_sympy().
    Refer to that module-level function for the full docstring.

    TODO(jwnimmer-tri) Also support Polynomial, Monomial, etc.
    """
    if isinstance(x, (float, int, bool)):
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
    sympy_args = [_to_sympy(arg, memo=memo) for arg in drake_args]
    return sympy_constructor(*sympy_args)


class _DrakePrinter(MpmathPrinter):
    """A slightly customized Printer class that handles a few unique spellings
    necessary for writing Drake code.
    """

    def __init__(self):
        super().__init__({
            "fully_qualified_modules": False,
            "inline": True,
            "allow_unknown_functions": True,
            "user_functions": {},
        })

    def _print_drake_logical_op(self, expr, op):
        """Uses pydrake.symbolic.logical_{op} instead of the Python built-in
        boolean operators (`and`, `or`, and `not`).
        """
        args = [self._print(arg) for arg in expr.args]
        return f"logical_{op}({', '.join(args)})"

    def _print_And(self, expr):
        return self._print_drake_logical_op(expr, "and")

    def _print_Or(self, expr):
        return self._print_drake_logical_op(expr, "or")

    def _print_Not(self, expr):
        return self._print_drake_logical_op(expr, "not")

    def _print_Piecewise(self, expr):
        """Undo the effect of _make_sympy_if_then_else, by converting a list of
        `((cond1, expr1), (cond2, expr2), ...)` pairs into a nested chain of
        `if_then_else(cond1, expr1, if_then_else(cond2, expr2, ...))` calls.
        """
        assert len(expr.args) > 0
        if expr.args[-1].cond not in (True, sympy.true):
            raise NotImplementedError(
                "Piecewise functions must always have a value; the final "
                "condition must be the literal value `True`.")
        result = [self._print(expr.args[-1].expr)]
        for arg in reversed(expr.args[:-1]):
            arg_cond = self._print(arg.cond)
            arg_expr = self._print(arg.expr)
            result.insert(0, f"if_then_else({arg_cond}, {arg_expr}, ")
            result.append(")")
        return "".join(result)


_DRAKE_PRINTER = _DrakePrinter()


def _lambdify(*, expr, args):
    """Wraps sympy.lambdify with extra hard-coded arguments."""
    return sympy.lambdify(
        expr=expr,
        args=args,
        modules=pydrake.symbolic,
        printer=_DRAKE_PRINTER,
        use_imps=False,
        docstring_limit=0)


def _from_sympy(
    x: Union[float, int, bool, sympy.Expr],
    *,
    memo: Dict = None
) -> Union[float, int, bool, Variable, Expression, Formula]:
    """This is the private implementation of pydrake.symbolic.from_sympy().
    Refer to that module-level function for the full docstring.
    """
    # Return non-SymPy inputs as-is.
    if isinstance(x, (float, int, bool)):
        return x
    # Return constants quickly.
    if x.is_number:
        return float(x.evalf())
    # Find the SymPy variables in `x` and look up the matching Drake variables.
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
    # Convert the SymPy expression to a Python function of the variables.
    drake_func = _lambdify(expr=x, args=sympy_vars)
    # Call the Python function using the Drake variables; this returns the
    # Drake symbolic expression.
    return drake_func(*drake_vars)
