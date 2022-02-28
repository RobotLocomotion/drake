#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

/** Represents a pair of Variables corresponding to sin(q) and cos(q). */
struct SinCos {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SinCos);

  SinCos(Variable s_var, Variable c_var)
      : s(std::move(s_var)), c(std::move(c_var)) {}

  /** sin variable. */
  Variable s{};
  /** cos variable. */
  Variable c{};
};

using SinCosSubstitution = std::unordered_map<Variable, SinCos>;

/** Given a substitution map q => {s, c}, substitutes instances of sin(q) and
 cos(q) in `e` with `s` and `c`, with partial support for trigonometric
 expansions. For instance,
 @verbatim
   Variable x{"x"}, y{"y"};
   Variable sx{"sx"}, cx{"cx"}, sy{"sy"}, cy{"cy"};
   SinCosSubstitution subs;
   subs.emplace(x, SinCos(sx, cx));
   subs.emplace(y, SinCos(sy, cy));
   Expression e = Substitute(x * sin(x + y), subs);
 @endverbatim
 will result in the expression `x * (sx*cy + cx*sy)`.

 @throws std::exception if a trigonometric function is not a trigonometric
 polynomial in `q` or if the `e` requires a trigonometric expansion that not
 supported yet.
 @pydrake_mkdoc_identifier{sincos}
*/
Expression Substitute(const Expression& e,
                      const SinCosSubstitution& subs);

/** Matrix version of sin/cos substitution.
 @pydrake_mkdoc_identifier{sincos_matrix} */
template <typename Derived>
Eigen::Matrix<Expression, Derived::RowsAtCompileTime,
              Derived::ColsAtCompileTime, 0, Derived::MaxRowsAtCompileTime,
              Derived::MaxColsAtCompileTime>
Substitute(const Eigen::MatrixBase<Derived>& m,
           const SinCosSubstitution& subs) {
  static_assert(std::is_same_v<typename Derived::Scalar, Expression>,
                "Substitute only accepts a matrix of symbolic::Expression.");
  // Note that the return type is written out explicitly to help gcc 5 (on
  // ubuntu).
  return m.unaryExpr(
      [&subs](const Expression& e) { return Substitute(e, subs); });
}

}  // namespace symbolic
}  // namespace drake
