#pragma once

#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"

namespace drake {
namespace symbolic {

/// Calls visitor object @p v with a polynomial symbolic-expression @p e, and
/// arguments @p args. Visitor object is expected to implement <tt>Result
/// operator()</tt> which takes a shared pointer of a class in ExpressionCell's
/// inheritance hierarchy.
///
/// @throws std::runtime_error if NaN is detected during a visit.
///
/// See the implementation of @c DegreeVisitor class and @c Degree function in
/// drake/common/monomial.cc as an example usage.
///
/// \pre{e.is_polynomial() is true.}
template <typename Result, typename Visitor, typename... Args>
Result VisitPolynomial(const Visitor& v, const Expression& e, Args&&... args) {
  DRAKE_ASSERT(e.is_polynomial());
  switch (e.get_kind()) {
    case ExpressionKind::Constant:
      return v(to_constant(e), std::forward<Args>(args)...);

    case ExpressionKind::Var:
      return v(to_variable(e), std::forward<Args>(args)...);

    case ExpressionKind::Add:
      return v(to_addition(e), std::forward<Args>(args)...);

    case ExpressionKind::Mul:
      return v(to_multiplication(e), std::forward<Args>(args)...);

    case ExpressionKind::Div:
      return v(to_division(e), std::forward<Args>(args)...);

    case ExpressionKind::Pow:
      return v(to_pow(e), std::forward<Args>(args)...);

    default:
      // Should be unreachable.
      DRAKE_ABORT();
  }
}

/// Calls visitor object @p v with a symbolic expression @p e, and arguments @p
/// args. Visitor object is expected to implement <tt>Result operator()</tt>
/// which takes a shared pointer of a class in ExpressionCell's inheritance
/// hierarchy.
///
/// @throws std::runtime_error if NaN is detected during a visit.
template <typename Result, typename Visitor, typename... Args>
Result VisitExpression(const Visitor& v, const Expression& e, Args&&... args) {
  switch (e.get_kind()) {
    case ExpressionKind::Constant:
      return v(to_constant(e), std::forward<Args>(args)...);

    case ExpressionKind::Var:
      return v(to_variable(e), std::forward<Args>(args)...);

    case ExpressionKind::Add:
      return v(to_addition(e), std::forward<Args>(args)...);

    case ExpressionKind::Mul:
      return v(to_multiplication(e), std::forward<Args>(args)...);

    case ExpressionKind::Div:
      return v(to_division(e), std::forward<Args>(args)...);

    case ExpressionKind::Log:
      return v(to_log(e), std::forward<Args>(args)...);

    case ExpressionKind::Abs:
      return v(to_abs(e), std::forward<Args>(args)...);

    case ExpressionKind::Exp:
      return v(to_exp(e), std::forward<Args>(args)...);

    case ExpressionKind::Sqrt:
      return v(to_sqrt(e), std::forward<Args>(args)...);

    case ExpressionKind::Pow:
      return v(to_pow(e), std::forward<Args>(args)...);

    case ExpressionKind::Sin:
      return v(to_sin(e), std::forward<Args>(args)...);

    case ExpressionKind::Cos:
      return v(to_cos(e), std::forward<Args>(args)...);

    case ExpressionKind::Tan:
      return v(to_tan(e), std::forward<Args>(args)...);

    case ExpressionKind::Asin:
      return v(to_asin(e), std::forward<Args>(args)...);

    case ExpressionKind::Acos:
      return v(to_acos(e), std::forward<Args>(args)...);

    case ExpressionKind::Atan:
      return v(to_atan(e), std::forward<Args>(args)...);

    case ExpressionKind::Atan2:
      return v(to_atan2(e), std::forward<Args>(args)...);

    case ExpressionKind::Sinh:
      return v(to_sinh(e), std::forward<Args>(args)...);

    case ExpressionKind::Cosh:
      return v(to_cosh(e), std::forward<Args>(args)...);

    case ExpressionKind::Tanh:
      return v(to_tanh(e), std::forward<Args>(args)...);

    case ExpressionKind::Min:
      return v(to_min(e), std::forward<Args>(args)...);

    case ExpressionKind::Max:
      return v(to_max(e), std::forward<Args>(args)...);

    case ExpressionKind::IfThenElse:
      return v(to_if_then_else(e), std::forward<Args>(args)...);

    case ExpressionKind::NaN:
      throw std::runtime_error("NaN is detected while visiting an expression.");

    case ExpressionKind::UninterpretedFunction:
      return v(to_uninterpreted_function(e), std::forward<Args>(args)...);
  }
}

}  // namespace symbolic
}  // namespace drake
