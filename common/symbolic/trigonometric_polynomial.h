#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"

namespace drake {
namespace symbolic {

enum class SinCosSubstitutionType {
  /** Substitutes s <=> sin(q), c <=> cos(q). */
  kAngle,
  /** Substitutes s <=> sin(q/2), c <=> cos(q/2), and prefers sin when the
   choice is ambiguous; e.g. cos(q) => 1 - 2s². */
  kHalfAnglePreferSin,
  /** Subsitutes s <=> sin(q/2), c <=> cos(q/2), and prefers cos when the
   choice is ambiguous; e.g. cos(q) => 2c² - 1. */
  kHalfAnglePreferCos,
};

/** Represents a pair of Variables corresponding to sin(q) and cos(q). */
struct SinCos {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SinCos);

  SinCos(Variable _s, Variable _c,
         SinCosSubstitutionType _type = SinCosSubstitutionType::kAngle)
      : s(std::move(_s)), c(std::move(_c)), type(_type) {}

  /** sin variable. */
  Variable s{};
  /** cos variable. */
  Variable c{};

  /** Allows a user to specify non-default substitutions, such as using
   half-angle formulas. */
  SinCosSubstitutionType type{};
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

 @param half_angle If true, then the same workflow replaces instances of
 sin(q/2) and cos(q/2) in `e` will be replaced with `s`, and `c`.
 @default false.

 The half-angle representation is more natural in many analysis computations
 for robots, for instance:
 https://underactuated.csail.mit.edu/lyapunov.html#trig_quadratic

 @throws std::exception if a trigonometric function is not a trigonometric
 polynomial in `q` or if the `e` requires a trigonometric expansion that not
 supported yet.
 @pydrake_mkdoc_identifier{sincos}
*/
Expression Substitute(const Expression& e, const SinCosSubstitution& subs);

/** Matrix version of sin/cos substitution.
 @pydrake_mkdoc_identifier{sincos_matrix} */
template <typename Derived>
MatrixLikewise<Expression, Derived>
Substitute(const Eigen::MatrixBase<Derived>& m,
           const SinCosSubstitution& subs) {
  static_assert(std::is_same_v<typename Derived::Scalar, Expression>,
                "Substitute only accepts a matrix of symbolic::Expression.");
  // Note that the return type is written out explicitly to help gcc 5 (on
  // ubuntu).
  return m.unaryExpr([&subs](const Expression& e) {
    return Substitute(e, subs);
  });
}

}  // namespace symbolic
}  // namespace drake
