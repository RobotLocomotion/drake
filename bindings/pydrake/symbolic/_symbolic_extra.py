# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

# ruff: noqa: F821 (undefined-name). This file is only a fragment.

from collections.abc import Callable
import functools
import operator
import pickle
import sys
import typing

from numpy import empty


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


_IFTHENELSE_EXPR_ARG_NUM = 3
_NAN_EXPR_ARG_NUM = 0
_CONST_EXPR_ARG_NUM = 1
_PSD_FORMULA_ARG_NUM = 1
_FORALL_FORMULA_ARG_NUM = 2
_ISNAN_FORMULA_ARG_NUM = 1
_UNINTERPRETED_FUNCTION_ARG_NUM = 2
_BINARY_OPS_ARG_NUM = 2
_UNARY_OPS_ARG_NUM = 1
_EXPR_BINARY_OPS: dict[ExpressionKind, Callable] = {
    ExpressionKind.Pow: pow,
    ExpressionKind.Atan2: atan2,
    ExpressionKind.Div: operator.truediv,
    ExpressionKind.Max: max,
    ExpressionKind.Min: min,
}
_EXPR_UNARY_OPS: dict[ExpressionKind, Callable] = {
    ExpressionKind.Abs: abs,
    ExpressionKind.Acos: acos,
    ExpressionKind.Asin: asin,
    ExpressionKind.Atan: atan,
    ExpressionKind.Ceil: ceil,
    ExpressionKind.Cos: cos,
    ExpressionKind.Cosh: cosh,
    ExpressionKind.Exp: exp,
    ExpressionKind.Floor: floor,
    ExpressionKind.Log: log,
    ExpressionKind.Sin: sin,
    ExpressionKind.Sinh: sinh,
    ExpressionKind.Sqrt: sqrt,
    ExpressionKind.Tan: tan,
    ExpressionKind.Tanh: tanh,
}
_EXPR_ADD_MUL_OPS: dict[ExpressionKind, Callable] = {
    ExpressionKind.Add: _reduce_add,
    ExpressionKind.Mul: _reduce_mul,
}

_FORMULA_BINARY_OPS: dict[FormulaKind, Callable] = {
    FormulaKind.Eq: operator.eq,
    FormulaKind.Neq: operator.ne,
    FormulaKind.Gt: operator.gt,
    FormulaKind.Geq: operator.ge,
    FormulaKind.Lt: operator.lt,
    FormulaKind.Leq: operator.le,
}
_FORMULA_LOGICAL_OPS: dict[FormulaKind, Callable] = {
    FormulaKind.And: logical_and,
    FormulaKind.Or: logical_or,
    FormulaKind.Not: logical_not,
}


def _check_args_len(expected_len: int, args_len: int) -> None:
    if args_len != expected_len:
        raise ValueError(f"Expected {expected_len} arguments, got {args_len}")


def _deconstruct_variable(var: Variable) -> bytes:
    return pickle.dumps(var)


def _reconstruct_variable(pickled_var: bytes) -> Variable:
    return pickle.loads(pickled_var)


def _deconstruct_formula(f: Formula) -> tuple[FormulaKind, list]:
    args = list()
    match f.get_kind():
        case FormulaKind.Var:
            args = _deconstruct_variable(list(f.GetFreeVariables())[0])
        case (
            FormulaKind.Eq
            | FormulaKind.Neq
            | FormulaKind.Gt
            | FormulaKind.Geq
            | FormulaKind.Lt
            | FormulaKind.Leq
            | FormulaKind.Isnan
        ):
            _, exprs = f.Unapply()
            args = [_deconstruct_expression(expr) for expr in exprs]
        case FormulaKind.And | FormulaKind.Or | FormulaKind.Not:
            _, formulas = f.Unapply()
            args = [_deconstruct_formula(formula) for formula in formulas]
        case FormulaKind.Forall:
            _, forall_args = f.Unapply()
            vars: Variables = forall_args[0]
            formula: Formula = forall_args[1]
            if not isinstance(vars, Variables):
                raise TypeError(f"Expected Variables, got {type(vars)}")
            if not isinstance(formula, Formula):
                raise TypeError(f"Expected Formula, got {type(formula)}")
            args.append([_deconstruct_variable(var) for var in vars])
            args.append(_deconstruct_formula(formula))
        case FormulaKind.PositiveSemidefinite:
            _, matrix_list = f.Unapply()
            matrix = matrix_list[0]
            deconstructed_rows = [
                [_deconstruct_expression(expr) for expr in row]
                for row in matrix
            ]
            args.append(deconstructed_rows)
    return f.get_kind(), args


def _reconstruct_formula(formula_kind: FormulaKind, args: list) -> Formula:
    match formula_kind:
        case FormulaKind.Var:
            return Formula(_reconstruct_variable(args))
        case FormulaKind.Isnan:
            _check_args_len(_ISNAN_FORMULA_ARG_NUM, len(args))
            return isnan(_reconstruct_expression(*args[0]))
        case FormulaKind.Forall:
            _check_args_len(_FORALL_FORMULA_ARG_NUM, len(args))
            pickled_vars = args[0]
            vars = Variables()
            for pickled_var in pickled_vars:
                vars.insert(_reconstruct_variable(pickled_var))
            pickled_f = args[1]
            formula: Formula = _reconstruct_formula(*pickled_f)
            return forall(vars, formula)
        case FormulaKind.PositiveSemidefinite:
            _check_args_len(_PSD_FORMULA_ARG_NUM, len(args))
            deconstructed_m = args[0]
            rows = len(deconstructed_m)
            cols = len(deconstructed_m[0]) if rows > 0 else 0
            m = empty((rows, cols), dtype=object)
            for i in range(rows):
                for j in range(cols):
                    m[i][j] = _reconstruct_expression(*deconstructed_m[i][j])
            return positive_semidefinite(m)

    if formula_kind in _FORMULA_LOGICAL_OPS:
        formulas = [_reconstruct_formula(*pickled_f) for pickled_f in args]
        return _FORMULA_LOGICAL_OPS[formula_kind](*formulas)

    if formula_kind in _FORMULA_BINARY_OPS:
        _check_args_len(_BINARY_OPS_ARG_NUM, len(args))
        expr1, expr2 = [_reconstruct_expression(*arg) for arg in args]
        return _FORMULA_BINARY_OPS[formula_kind](expr1, expr2)
    raise ValueError(f"Unhandled FormulaKind: {formula_kind}")


def _deconstruct_expression(e: Expression) -> tuple[ExpressionKind, list]:
    args = list()
    match e.get_kind():
        case ExpressionKind.Constant:
            args.append(e.Evaluate())
        case ExpressionKind.Var:
            args = _deconstruct_variable(list(e.GetVariables())[0])
        case ExpressionKind.NaN:
            pass
        case ExpressionKind.Add | ExpressionKind.Mul:
            _, exprs = e.Unapply()
            if not isinstance(exprs[0], float):
                raise TypeError(f"Expected float, got {type(exprs[0])}")
            args = [exprs[0]] + [
                _deconstruct_expression(expr) for expr in exprs[1:]
            ]
        case (
            ExpressionKind.Pow
            | ExpressionKind.Atan2
            | ExpressionKind.Div
            | ExpressionKind.Max
            | ExpressionKind.Min
            | ExpressionKind.Abs
            | ExpressionKind.Acos
            | ExpressionKind.Asin
            | ExpressionKind.Atan
            | ExpressionKind.Ceil
            | ExpressionKind.Cos
            | ExpressionKind.Cosh
            | ExpressionKind.Exp
            | ExpressionKind.Floor
            | ExpressionKind.Log
            | ExpressionKind.Sin
            | ExpressionKind.Sinh
            | ExpressionKind.Sqrt
            | ExpressionKind.Tan
            | ExpressionKind.Tanh
        ):
            _, exprs = e.Unapply()
            args = [_deconstruct_expression(expr) for expr in exprs]
        case ExpressionKind.IfThenElse:
            _, symbolics = e.Unapply()
            args = [_deconstruct_formula(symbolics[0])] + [
                _deconstruct_expression(expr) for expr in symbolics[1:]
            ]
        case ExpressionKind.UninterpretedFunction:
            _, func_signature = e.Unapply()
            func_name: str = func_signature[0]
            func_args: list[Expression] = func_signature[1]
            args = [func_name] + [
                [_deconstruct_expression(expr) for expr in func_args]
            ]
    return e.get_kind(), args


def _reconstruct_expression(
    expr_kind: ExpressionKind,
    args: list,
) -> Expression:
    match expr_kind:
        case ExpressionKind.Constant:
            _check_args_len(_CONST_EXPR_ARG_NUM, len(args))
            return Expression(*args)
        case ExpressionKind.Var:
            return Expression(_reconstruct_variable(args))
        case ExpressionKind.NaN:
            _check_args_len(_NAN_EXPR_ARG_NUM, len(args))
            return Expression(float("nan"))
        case ExpressionKind.IfThenElse:
            _check_args_len(_IFTHENELSE_EXPR_ARG_NUM, len(args))
            formula = _reconstruct_formula(*args[0])
            expr_then, expr_else = [
                _reconstruct_expression(*arg) for arg in args[1:]
            ]
            return if_then_else(formula, expr_then, expr_else)
        case ExpressionKind.UninterpretedFunction:
            _check_args_len(_UNINTERPRETED_FUNCTION_ARG_NUM, len(args))
            return uninterpreted_function(
                name=args[0],
                arguments=[_reconstruct_expression(*arg) for arg in args[1]],
            )

    if expr_kind in _EXPR_UNARY_OPS:
        _check_args_len(_UNARY_OPS_ARG_NUM, len(args))
        expr = _reconstruct_expression(*args[0])
        return _EXPR_UNARY_OPS[expr_kind](expr)

    if expr_kind in _EXPR_BINARY_OPS:
        _check_args_len(_BINARY_OPS_ARG_NUM, len(args))
        expr1, expr2 = [_reconstruct_expression(*arg) for arg in args]
        return _EXPR_BINARY_OPS[expr_kind](expr1, expr2)

    if expr_kind in _EXPR_ADD_MUL_OPS:
        if not isinstance(args[0], float):
            raise TypeError(f"Expected float, got {type(args[0])}")
        exprs = [args[0]] + [_reconstruct_expression(*arg) for arg in args[1:]]
        return _EXPR_ADD_MUL_OPS[expr_kind](*exprs)
    raise ValueError(f"Unhandled ExpressionKind: {expr_kind}")


def _reduce_expression(self) -> tuple[Callable, tuple]:
    return (_reconstruct_expression, _deconstruct_expression(self))


Expression.__reduce__ = _reduce_expression

# Drake's SymPy support is loaded lazily (on demand), so that Drake does not
# directly depend on SymPy. The implementation lives in `_symbolic_sympy.py`.
_symbolic_sympy_defer = None


def to_sympy(
    x: float | int | bool | Variable | Expression | Formula,
    *,
    memo: dict = None,
) -> typing.Union[float, int, bool, "sympy.Expr"]:
    """Converts a pydrake object to the corresponding SymPy Expr.

    Certain expressions are not supported and will raise NotImplementedError.
    (Most acutely, note that `int` is not yet supported.)

    This function aims to support the latest contemporaneous version of SymPy
    as of Drake's release date.

    Args:
        x: The pydrake object to be converted.
        memo: (Optional) Mapping between Drake variables and SymPy variables.
            Converting a ``pydrake.symbolic.Variable`` to SymPy will look up
            the Drake variable's ``v.get_id()`` as a key in this dictionary.
            If a value is found, it will be used as the SymPy atom for that
            Drake variable.  Otherwise, a new SymPy variable will be created
            and used, and both mappings ``{drake_var.get_id(): sympy_var}``
            and ``{sympy_var: drake_var}`` will be inserted into ``memo``.

    See also :meth:`pydrake.symbolic.from_sympy`.
    """
    global _symbolic_sympy_defer
    if _symbolic_sympy_defer is None:
        from pydrake.symbolic import _symbolic_sympy as _symbolic_sympy_defer
    if memo is None:
        memo = dict()
    return _symbolic_sympy_defer._to_sympy(x, memo=memo)


def from_sympy(
    x: typing.Union[float, int, bool, "sympy.Expr"], *, memo: dict = None
) -> float | int | bool | Variable | Expression | Formula:
    """Converts a SymPy Expr to the corresponding pydrake object.

    Certain expressions are not supported and will raise NotImplementedError.

    This function aims to support the latest contemporaneous version of SymPy
    as of Drake's release date.

    Args:
        x: The SymPy object to be converted.
        memo: (Optional) Mapping between SymPy variables and Drake variables.
            Converting a SymPy variable to a ``pydrake.symbolic.Variable`` will
            look up the SymPy variable as a key in this dictionary.
            If a value is found, then it will be used as the Drake variable.
            If no value is found, then (for now) raises an exception. In the
            future, we might support automatically creating variables, but
            that is not yet implemented.

    See also :meth:`pydrake.symbolic.to_sympy`.
    """
    global _symbolic_sympy_defer
    if _symbolic_sympy_defer is None:
        from pydrake.symbolic import _symbolic_sympy as _symbolic_sympy_defer
    if memo is None:
        memo = dict()
    return _symbolic_sympy_defer._from_sympy(x, memo=memo)


# We must be able to do `from pydrake.symbolic import _symbolic_sympy` so we
# need `pydrake.symbolic` to be a Python package, not merely a module. (See
# https://docs.python.org/3/tutorial/modules.html for details.) The way to
# designate something as a package is to define its `__path__` attribute.
__path__ = [sys.modules["pydrake"].__path__[0] + "/symbolic"]
