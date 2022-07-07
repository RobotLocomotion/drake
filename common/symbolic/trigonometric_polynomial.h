#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/rational_function.h"

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

/**
 * Substitutes the variables representing sine and cosine functions with their
 * stereographic projection.
 * If e is a multilinear polynomial of cosθ and sinθ,
 * and no cosθᵢ and sinθᵢ appear in the same monomial, then we replace cosθᵢ
 * with (1-tᵢ²)/(1+tᵢ²), and sinθᵢ with 2tᵢ/(1+tᵢ²), and get a rational
 * polynomial of t.
 * If the input expression doesn't contain the sine and cosine functions, then
 * the returned rational has denominator being 1.
 * @param e The symbolic polynomial to be substituted. Note that e has to be a
 * multilinear polynomial of variables in `sin_cos`, namely e.indeterminates()
 * is a subset of all the variables in `sin_cos`, and no sin_cos[i].s and
 * sin_cos[i].c appear in the same monomial. Namely that e can be sin_cos[0].s *
 * sin_cos[1].c + 1  or t[0]*sin_cos[0].s * sin_cos[1].c + 1 (note t can also
 * appear in e), but cannot be sin_cos[0].s * sin_cos[0].c (where sin_cos[0].s
 * and sin_cos[0].c appear in the same monomial) or pow(sin_cos[0].s, 2) (where
 * the degree or sin_cos[0].s is larger than 1).
 * @param sin_cos sin_cos(i) is the pair of variables (sᵢ, cᵢ), (where sᵢ=sinθᵢ,
 * cᵢ=cosθᵢ) as documented above.
 * @param t New variables to express cos and sin as rationals of t. tᵢ =
 * tan(θᵢ/2).
 * @pre t.rows() == sin_cos.size()
 * @return e_rational The rational polynomial of e after replacement. The
 * indeterminates of the polynomials are `t`.
 * Example
 * @verbatim
 * std::vector<SinCos> sin_cos;
 * sin_cos.emplace_back(symbolic::Variable("s0"), symbolic::Variable("c0"));
 * sin_cos.emplace_back(symbolic::Variable("s1"), symbolic::Variable("c1"));
 * Vector2<symbolic::Variable> t(symbolic::Variable("t0"),
 *                               symbolic::Variable("t1"));
 * const auto e_rational =
 * SubstituteStereographicProjection(t(0) * sin_cos[0].s*sin_cos[1].c + 1,
 *                                   sin_cos, t);
 * // e_rational should be
 * // (2*t0*t0*(1-t1*t1) + (1+t0*t0)*(1+t1*t1))
 * // --------------------------------------------
 * //        ((1+t0*t0)*(1+t1*t1))
 * @endverbatim
 */
[[nodiscard]] symbolic::RationalFunction SubstituteStereographicProjection(
    const symbolic::Polynomial& e, const std::vector<SinCos>& sin_cos,
    const VectorX<symbolic::Variable>& t);

/**
 * Substitutes sine and cosine function with their stereographic projection.
 * If e is a multilinear polynomial of cosθ and sinθ, and no
 * cosθᵢ and sinθᵢ appear in the same monomial, then we replace
 * cosθᵢ with (1-tᵢ²)/(1+tᵢ²), and sinθᵢ with 2tᵢ/(1+tᵢ²), and get a rational
 * polynomial of t.
 * If the input expression doesn't contain the sine and cosine functions, then
 * the returned rational has denominator being 1.
 * @param e The symbolic expression to be substituted. Note that e has to be a
 * multilinear polynomial of cosθ and sinθ, and no cosθᵢ and sinθᵢ appear in the
 * same monomial. Namely that e can be cosθ₁sinθ₂ + sinθ₂ or t₁cosθ₁sinθ₂ +
 * sinθ₂ (note that t can also appear in e), but cannot be sinθ₁cosθ₁ (where
 * sinθ₁ and cosθ₁ appear in the same monomial) or sin²θ₁ (where the degree or
 * sinθ₁ is larger than 1).
 * @param subs Correspond each θᵢ to tᵢ, we will replace cosθᵢ with
 * (1-tᵢ²)/(1+tᵢ²), and sinθᵢ with 2tᵢ/(1+tᵢ²)
 * @return e_rational The rational polynomial after replacing cosθᵢ with
 * (1-tᵢ²)/(1+tᵢ²), and sinθᵢ with 2tᵢ/(1+tᵢ²). The indeterminates of the
 * rational polynomials are t.
 *
 * Example
 * @verbatim
 * const symbolic::Variable theta1("theta1");
 * const symbolic::Variable theta2("theta1");
 * const symbolic::Variable t1{"t1");
 * const symbolic::Variable t2{"t2");
 * const auto e_rational = SubstituteStereographicProjection(t1 *
 *     sin(theta1)*cos(theta2) + 1, {{theta1, t1}, {theta2, t2}});
 * // The result of e_rational is
 * // (2t₁²(1−t₂²) + (1+t₁²)(1+t₂²)) / ((1+t₁²)(1+t₂²))
 * @endverbatim
 */
[[nodiscard]] symbolic::RationalFunction SubstituteStereographicProjection(
    const symbolic::Expression& e,
    const std::unordered_map<symbolic::Variable, symbolic::Variable>& subs);

}  // namespace symbolic
}  // namespace drake
