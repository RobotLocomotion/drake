#include "drake/bindings/pydrake/symbolic_py_unapply.h"

#include "fmt/format.h"
#include "fmt/ostream.h"
#include "pybind11/stl.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

using drake::symbolic::Expression;
using drake::symbolic::ExpressionKind;
using drake::symbolic::Formula;
using drake::symbolic::FormulaKind;

// Given the pydrake.symbolic module as "m" and an expression "e", returns
// the callable object (i.e., factory function or constructor) that would
// be able to re-construct the same expression, given appropriate arguments.
py::object MakeConstructor(py::module m, const Expression& e) {
  // This list of cases is in alphabetical order.
  switch (e.get_kind()) {
    case ExpressionKind::Abs:
      return m.attr("abs");
    case ExpressionKind::Acos:
      return m.attr("acos");
    case ExpressionKind::Add:
      return m.attr("_reduce_add");
    case ExpressionKind::Asin:
      return m.attr("asin");
    case ExpressionKind::Atan2:
      return m.attr("atan2");
    case ExpressionKind::Atan:
      return m.attr("atan");
    case ExpressionKind::Ceil:
      return m.attr("ceil");
    case ExpressionKind::Constant:
      return m.attr("Expression");
    case ExpressionKind::Cos:
      return m.attr("cos");
    case ExpressionKind::Cosh:
      return m.attr("cosh");
    case ExpressionKind::Div:
      return m.attr("operator").attr("truediv");
    case ExpressionKind::Exp:
      return m.attr("exp");
    case ExpressionKind::Floor:
      return m.attr("floor");
    case ExpressionKind::IfThenElse:
      return m.attr("if_then_else");
    case ExpressionKind::Log:
      return m.attr("log");
    case ExpressionKind::Max:
      return m.attr("max");
    case ExpressionKind::Min:
      return m.attr("min");
    case ExpressionKind::Mul:
      return m.attr("_reduce_mul");
    case ExpressionKind::NaN:
      return m.attr("Expression");
    case ExpressionKind::Pow:
      return m.attr("pow");
    case ExpressionKind::Sin:
      return m.attr("sin");
    case ExpressionKind::Sinh:
      return m.attr("sinh");
    case ExpressionKind::Sqrt:
      return m.attr("sqrt");
    case ExpressionKind::Tan:
      return m.attr("tan");
    case ExpressionKind::Tanh:
      return m.attr("tanh");
    case ExpressionKind::UninterpretedFunction:
      return m.attr("uninterpreted_function");
    case ExpressionKind::Var:
      return m.attr("Expression");
  }
  DRAKE_UNREACHABLE();
}

// Given the expression "e", returns an extracted list of arguments that would
// be able to re-construct the same expression, when passed to the result of
// MakeConstructor.
py::list MakeArgs(const Expression& e) {
  py::list result;
  switch (e.get_kind()) {
    // The only cases where the result is not a list of sub-Expressions are
    // constants and variables.
    case ExpressionKind::Constant: {
      result.append(get_constant_value(e));
      break;
    }
    case ExpressionKind::NaN: {
      result.append(NAN);
      break;
    }
    case ExpressionKind::Var: {
      result.append(get_variable(e));
      break;
    }
    // These are all UnaryExpressionCell.
    case ExpressionKind::Abs:
    case ExpressionKind::Acos:
    case ExpressionKind::Asin:
    case ExpressionKind::Atan:
    case ExpressionKind::Ceil:
    case ExpressionKind::Cos:
    case ExpressionKind::Cosh:
    case ExpressionKind::Exp:
    case ExpressionKind::Floor:
    case ExpressionKind::Log:
    case ExpressionKind::Sin:
    case ExpressionKind::Sinh:
    case ExpressionKind::Sqrt:
    case ExpressionKind::Tan:
    case ExpressionKind::Tanh: {
      result.append(get_argument(e));
      break;
    }
    // These are all BinaryExpressionCell.
    case ExpressionKind::Atan2:
    case ExpressionKind::Div:
    case ExpressionKind::Max:
    case ExpressionKind::Min:
    case ExpressionKind::Pow: {
      result.append(get_first_argument(e));
      result.append(get_second_argument(e));
      break;
    }
    // Add and Mul are reductions over lists of expressions.
    case ExpressionKind::Add: {
      result.append(get_constant_in_addition(e));
      for (const auto& [expr, coeff] : get_expr_to_coeff_map_in_addition(e)) {
        result.append(coeff * expr);
      }
      break;
    }
    case ExpressionKind::Mul: {
      result.append(get_constant_in_multiplication(e));
      for (const auto& [base, exp] :
          get_base_to_exponent_map_in_multiplication(e)) {
        result.append(symbolic::pow(base, exp));
      }
      break;
    }
    // Special forms.
    case ExpressionKind::IfThenElse: {
      result.append(get_conditional_formula(e));
      result.append(get_then_expression(e));
      result.append(get_else_expression(e));
      break;
    }
    case ExpressionKind::UninterpretedFunction: {
      py::list function_args;
      for (const auto& expr : get_uninterpreted_function_arguments(e)) {
        function_args.append(expr);
      }
      result.append(get_uninterpreted_function_name(e));
      result.append(function_args);
      break;
    }
  }
  return result;
}

// Given the pydrake.symbolic module as "m" and a formula "f", returns
// the callable object (i.e., factory function or constructor) that would
// be able to re-construct the same formula, given appropriate arguments.
py::object MakeConstructor(py::module m, const Formula& f) {
  switch (f.get_kind()) {
    case FormulaKind::False:
      return m.attr("Formula").attr("False_");
    case FormulaKind::True:
      return m.attr("Formula").attr("True_");
    case FormulaKind::Var:
      return m.attr("Formula");
    case FormulaKind::Eq:
      return m.attr("operator").attr("eq");
    case FormulaKind::Neq:
      return m.attr("operator").attr("ne");
    case FormulaKind::Gt:
      return m.attr("operator").attr("gt");
    case FormulaKind::Geq:
      return m.attr("operator").attr("ge");
    case FormulaKind::Lt:
      return m.attr("operator").attr("lt");
    case FormulaKind::Leq:
      return m.attr("operator").attr("le");
    case FormulaKind::And:
      return m.attr("logical_and");
    case FormulaKind::Or:
      return m.attr("logical_or");
    case FormulaKind::Not:
      return m.attr("logical_not");
    case FormulaKind::Forall:
      return m.attr("forall");
    case FormulaKind::Isnan:
      return m.attr("isnan");
    case FormulaKind::PositiveSemidefinite:
      return m.attr("positive_semidefinite");
  }
  DRAKE_UNREACHABLE();
}

// Given the formula "f", returns an extracted list of arguments that would
// be able to re-construct the same formula, when passed to the result of
// MakeConstructor.
py::list MakeArgs(const Formula& f) {
  py::list result;
  switch (f.get_kind()) {
    case FormulaKind::False:
    case FormulaKind::True: {
      break;
    }
    case FormulaKind::Var: {
      result.append(get_variable(f));
      break;
    }
    case FormulaKind::Eq:
    case FormulaKind::Neq:
    case FormulaKind::Gt:
    case FormulaKind::Geq:
    case FormulaKind::Lt:
    case FormulaKind::Leq: {
      result.append(get_lhs_expression(f));
      result.append(get_rhs_expression(f));
      break;
    }
    case FormulaKind::And:
    case FormulaKind::Or: {
      for (const Formula& operand : get_operands(f)) {
        result.append(operand);
      }
      break;
    }
    case FormulaKind::Not: {
      result.append(get_operand(f));
      break;
    }
    case FormulaKind::Forall: {
      result.append(get_quantified_variables(f));
      result.append(get_quantified_formula(f));
      break;
    }
    case FormulaKind::Isnan: {
      result.append(get_unary_expression(f));
      break;
    }
    case FormulaKind::PositiveSemidefinite: {
      result.append(get_matrix_in_positive_semidefinite(f));
      break;
    }
  }
  return result;
}

}  // namespace

py::object Unapply(py::module m, const Expression& e) {
  return py::make_tuple(MakeConstructor(m, e), MakeArgs(e));
}

py::object Unapply(py::module m, const symbolic::Formula& f) {
  return py::make_tuple(MakeConstructor(m, f), MakeArgs(f));
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
