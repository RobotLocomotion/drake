#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
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

namespace internal {
/*
 This is the actual implementation of SubstituteStereographicProjection().
 We expose this function because repeated calls to
 SubstituteStereographicProjection() require re-constructing the polynomial
 1+t², 2t, 1-t²; it is more efficient to construct these polynomials
 beforehand and then call SubstituteStereographicProjectionImpl() repeatedly.
 @param e_poly The polynomial before substitution.
 @param sin_cos The sin and cos variables in e_poly to be replaced.
 @param t We will replace sin and cos functions as rational functions of t.
 @param t_set The set of variables @p t.
 @param one_plus_t_angles_squared 1+t²
 @param two_t_angles 2t
 @param one_minus_t_angles_squared 1-t²
 @retval e_rational The rational polynomial after replacing sin/cos in @p
 e_poly with rational functions of t.
 @pre sin_cos, t, t_set, one_plus_t_angles_squared, two_t_angles,
 one_minus_t_angles_squared all have the same size.
 @pre The indeterminates in `e_poly` are `sin_cos`.
 */
symbolic::RationalFunction SubstituteStereographicProjectionImpl(
    const symbolic::Polynomial& e_poly, const std::vector<SinCos>& sin_cos,
    const VectorX<symbolic::Variable>& t, const symbolic::Variables& t_set,
    const VectorX<symbolic::Polynomial>& one_plus_t_angles_squared,
    const VectorX<symbolic::Polynomial>& two_t_angles,
    const VectorX<symbolic::Polynomial>& one_minus_t_angles_squared);
}  // namespace internal

/**
 * Substitutes the variables representing sine and cosine functions with their
 * stereographic projection.
 * For the moment, we put the following requirement on `e`:
 * If the indeterminates of `e` include cosθ and sinθ, then e is a multilinear
 * polynomial of cosθ and sinθ, in which cosθᵢ and sinθᵢ do not appear in the
 * same monomial for any i.
 * When this requirement is satisfied, we replace cosθᵢ
 * with (1-tᵢ²)/(1+tᵢ²), and sinθᵢ with 2tᵢ/(1+tᵢ²), and get a rational
 * polynomial. The indeterminates of this rational polynomial are t together
 * with the indeterminates in `e` that are not cosθ or sinθ. If the input
 * expression doesn't contain the sine and cosine functions, then the returned
 * rational has denominator being 1. Notice that the indeterminates of `e` can
 * include variables other than cosθ and sinθ, and we impose no requirements on
 * these variables that are not cosθ or sinθ.
 *
 * For example,
 * e = x²ysinθ₁cosθ₂ + 3xsinθ₂cosθ₃ is a valid input.
 * e = sinθ₁cosθ₁ + 1 is not valid because both an sinθ₁ and cosθ₁ appear in the
 * same monomial.
 * e = sin²θ₁ is not a valid input because it is quadratic in sinθ₁.
 *
 * @param e The symbolic polynomial to be substituted. Note that e has to be a
 * multilinear polynomial of variables in `sin_cos`, namely inside each monomial
 * m, m.degree(sin_cos[i].c) <= 1, m.degree(sin_cos[i].s) <= 1, and no
 * sin_cos[i].s and sin_cos[i].c appear in the same monomial. Namely that e can
 * be sin_cos[0].s * sin_cos[1].c + 1  or t[0]*sin_cos[0].s * sin_cos[1].c + 1
 * (note t can also appear in e), but cannot be sin_cos[0].s * sin_cos[0].c
 * (where sin_cos[0].s and sin_cos[0].c appear in the same monomial) or
 * pow(sin_cos[0].s, 2) (where the degree or sin_cos[0].s is larger than 1).
 * @param sin_cos sin_cos(i) is the pair of variables (sᵢ, cᵢ), (where sᵢ=sinθᵢ,
 * cᵢ=cosθᵢ) as documented above.
 * @param t New variables to express cos and sin as rationals of t. tᵢ =
 * tan(θᵢ/2).
 * @pre t.rows() == sin_cos.size()
 * @return e_rational The rational polynomial of e after replacement. The
 * indeterminates of the polynomials are `t` together with the indeterminates in
 * `e` that are not cosθ or sinθ. Example
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
// TODO(hongkai.dai): relax the requirements on `e`, such that we can handle any
// `e` being a polynomial function of cosθ an sinθ.
[[nodiscard]] symbolic::RationalFunction SubstituteStereographicProjection(
    const symbolic::Polynomial& e, const std::vector<SinCos>& sin_cos,
    const VectorX<symbolic::Variable>& t);

DRAKE_DEPRECATED("2023-02-01",
                 "Use the other SubstituteStereographicProjection which passes "
                 "e as a symbolic::Polynomial")
[[nodiscard]] symbolic::RationalFunction SubstituteStereographicProjection(
    const symbolic::Expression& e,
    const std::unordered_map<symbolic::Variable, symbolic::Variable>& subs);

}  // namespace symbolic
}  // namespace drake
