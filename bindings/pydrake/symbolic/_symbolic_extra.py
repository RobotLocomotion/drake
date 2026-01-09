# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

# ruff: noqa: F821 (undefined-name). This file is only a fragment.

import functools
import operator
import sys
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


def _deconstruct_variable(var: Variable) -> list:
    return [var.get_id(), var.get_name(), var.get_type()]


var_map_t = dict[int, Variable]


def _reconstruct_variable(
    var_id: int, var_name: str, var_type: int, var_map: var_map_t
) -> Variable:
    if var_id not in var_map:
        var_map[var_id] = Variable(var_name, var_type)
    return var_map[var_id]


def _deconstruct_formula(f: Formula) -> tuple:
    result = list()
    args = list()
    result.append(f.get_kind())
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
            assert isinstance(vars, Variables) and isinstance(formula, Formula)
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
    result.append(args)
    return tuple(result)


def _reconstruct_formula(
    formula_kind: FormulaKind, args: list, var_map: var_map_t
) -> Formula:
    match formula_kind:
        case FormulaKind.Var:
            assert len(args) == 3
            return Formula(_reconstruct_variable(*args, var_map=var_map))
        case FormulaKind.Isnan:
            assert len(args) == 1
            return isnan(
                _recur_reconstruct_expression(*args[0], var_map=var_map)
            )
        case FormulaKind.Forall:
            assert len(args) == 2
            pickled_vars = args[0]
            vars = Variables()
            for pickled_var in pickled_vars:
                vars.insert(
                    _reconstruct_variable(*pickled_var, var_map=var_map)
                )
            pickled_f = args[1]
            formula: Formula = _reconstruct_formula(*pickled_f, var_map=var_map)
            return forall(vars, formula)
        case FormulaKind.PositiveSemidefinite:
            assert len(args) == 1
            deconstructed_m = args[0]
            rows = len(deconstructed_m)
            cols = len(deconstructed_m[0]) if rows > 0 else 0
            from numpy import empty

            m = empty((rows, cols), dtype=object)
            for i in range(rows):
                for j in range(cols):
                    m[i][j] = _recur_reconstruct_expression(
                        *deconstructed_m[i][j], var_map=var_map
                    )
            return positive_semidefinite(m)

    logical_ops = {
        FormulaKind.And: logical_and,
        FormulaKind.Or: logical_or,
        FormulaKind.Not: logical_not,
    }
    if formula_kind in logical_ops:
        formulas = [
            _reconstruct_formula(*pickled_f, var_map=var_map)
            for pickled_f in args
        ]
        return logical_ops[formula_kind](*formulas)
    binary_ops = {
        FormulaKind.Eq: operator.eq,
        FormulaKind.Neq: operator.ne,
        FormulaKind.Gt: operator.gt,
        FormulaKind.Geq: operator.ge,
        FormulaKind.Lt: operator.lt,
        FormulaKind.Leq: operator.le,
    }
    if formula_kind in binary_ops:
        assert len(args) == 2
        expr1, expr2 = [
            _recur_reconstruct_expression(*arg, var_map=var_map) for arg in args
        ]
        return binary_ops[formula_kind](expr1, expr2)


def _deconstruct_expression(e: Expression) -> tuple:
    result = list()
    args = list()
    result.append(e.get_kind())
    match e.get_kind():
        case ExpressionKind.Constant:
            args.append(e.Evaluate())
        case ExpressionKind.Var:
            args = _deconstruct_variable(list(e.GetVariables())[0])
        case ExpressionKind.NaN:
            pass
        case ExpressionKind.Add | ExpressionKind.Mul:
            _, exprs = e.Unapply()
            # exprs[0] is a number not an expression
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
    result.append(args)
    return tuple(result)


def _recur_reconstruct_expression(
    expr_kind: ExpressionKind, args: list, var_map: var_map_t
) -> Expression:
    match expr_kind:
        case ExpressionKind.Constant:
            assert len(args) == 1
            return Expression(*args)
        case ExpressionKind.Var:
            assert len(args) == 3
            return Expression(_reconstruct_variable(*args, var_map=var_map))
        case ExpressionKind.NaN:
            assert len(args) == 0
            return Expression(float("nan"))
        case ExpressionKind.IfThenElse:
            assert len(args) == 3
            formula = _reconstruct_formula(*args[0], var_map=var_map)
            expr_then, expr_else = [
                _recur_reconstruct_expression(*arg, var_map=var_map)
                for arg in args[1:]
            ]
            return if_then_else(formula, expr_then, expr_else)
        case ExpressionKind.UninterpretedFunction:
            assert len(args) == 2
            return uninterpreted_function(
                name=args[0],
                arguments=[
                    _recur_reconstruct_expression(*arg, var_map=var_map)
                    for arg in args[1]
                ],
            )

    unary_ops = {
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
    if expr_kind in unary_ops:
        assert len(args) == 1
        expr = _recur_reconstruct_expression(*args[0], var_map=var_map)
        return unary_ops[expr_kind](expr)

    binary_ops = {
        ExpressionKind.Pow: pow,
        ExpressionKind.Atan2: atan2,
        ExpressionKind.Div: operator.truediv,
        ExpressionKind.Max: max,
        ExpressionKind.Min: min,
    }
    if expr_kind in binary_ops:
        assert len(args) == 2
        expr1, expr2 = [
            _recur_reconstruct_expression(*arg, var_map=var_map) for arg in args
        ]
        return binary_ops[expr_kind](expr1, expr2)

    add_mul_ops = {
        ExpressionKind.Add: _reduce_add,
        ExpressionKind.Mul: _reduce_mul,
    }
    if expr_kind in add_mul_ops:
        assert isinstance(args[0], float)
        exprs = [args[0]] + [
            _recur_reconstruct_expression(*arg, var_map=var_map)
            for arg in args[1:]
        ]
        return add_mul_ops[expr_kind](*exprs)


def _reconstruct_expression(
    expr_kind: ExpressionKind, args: list
) -> Expression:
    var_map: var_map_t = dict()
    return _recur_reconstruct_expression(expr_kind, args, var_map)


def _reduce_expression(self):
    return (_reconstruct_expression, _deconstruct_expression(self))


Expression.__reduce__ = _reduce_expression

# Drake's SymPy support is loaded lazily (on demand), so that Drake does not
# directly depend on SymPy. The implementation lives in `_symbolic_sympy.py`.
_symbolic_sympy_defer = None


def to_sympy(
    x: typing.Union[float, int, bool, Variable, Expression, Formula],
    *,
    memo: typing.Dict = None,
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
    x: typing.Union[float, int, bool, "sympy.Expr"], *, memo: typing.Dict = None
) -> typing.Union[float, int, bool, Variable, Expression, Formula]:
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
