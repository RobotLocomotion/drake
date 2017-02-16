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

}  // namespace symbolic
}  // namespace drake
