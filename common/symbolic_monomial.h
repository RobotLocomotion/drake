#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <cstddef>
#include <map>
#include <ostream>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {

namespace symbolic {

/** Represents a monomial, a product of powers of variables with non-negative
 * integer exponents. Note that it does not include the coefficient part of a
 * monomial.
 */
class Monomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Monomial)

  /** Constructs a monomial equal to 1. Namely the total degree is zero. */
  Monomial() = default;

  /** Constructs a default value.  This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit Monomial(std::nullptr_t) : Monomial() {}

  /** Constructs a Monomial from @p powers.
   * @throws std::exception if `powers` includes a negative exponent.
   */
  explicit Monomial(const std::map<Variable, int>& powers);

  /** Constructs a Monomial from a vector of variables `vars` and their
   * corresponding integer exponents `exponents`.
   * For example, `Monomial([x, y, z], [2, 0, 1])` constructs a Monomial `x²z`.
   *
   * @pre The size of `vars` should be the same as the size of `exponents`.
   * @throws std::exception if `exponents` includes a negative integer.
   */
  Monomial(const Eigen::Ref<const VectorX<Variable>>& vars,
           const Eigen::Ref<const Eigen::VectorXi>& exponents);

  /**
   * Converts an expression to a monomial if the expression is written as
   * ∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.
   * @pre is_polynomial(e) should be true.
   */
  explicit Monomial(const Expression& e);

  /** Constructs a Monomial from @p var. */
  explicit Monomial(const Variable& var);

  /** Constructs a Monomial from @p var and @p exponent. */
  Monomial(const Variable& var, int exponent);

  /** Returns the degree of this Monomial in a variable @p v. */
  int degree(const Variable& v) const;

  /** Returns the total degree of this Monomial. */
  int total_degree() const { return total_degree_; }

  /** Returns the set of variables in this monomial. */
  Variables GetVariables() const;

  /** Returns the internal representation of Monomial, the map from a base
   * (Variable) to its exponent (int).*/
  const std::map<Variable, int>& get_powers() const { return powers_; }

  /** Evaluates under a given environment @p env.
   *
   * @throws std::exception if there is a variable in this monomial
   * whose assignment is not provided by @p env.
   */
  double Evaluate(const Environment& env) const;

  /** Partially evaluates using a given environment @p env. The evaluation
   * result is of type pair<double, Monomial>. The first component (: double)
   * represents the coefficient part while the second component represents the
   * remaining parts of the Monomial which was not evaluated.
   *
   * Example 1. Evaluate with a fully-specified environment
   *     (x³*y²).EvaluatePartial({{x, 2}, {y, 3}})
   *   = (2³ * 3² = 8 * 9 = 72, Monomial{} = 1).
   *
   * Example 2. Evaluate with a partial environment
   *     (x³*y²).EvaluatePartial({{x, 2}})
   *   = (2³ = 8, y²).
   */
  std::pair<double, Monomial> EvaluatePartial(const Environment& env) const;

  /** Returns a symbolic expression representing this monomial. */
  Expression ToExpression() const;

  /** Checks if this monomial and @p m represent the same monomial. Two
   * monomials are equal iff they contain the same variable raised to the same
   * exponent. */
  bool operator==(const Monomial& m) const;

  /** Checks if this monomial and @p m do not represent the same monomial. */
  bool operator!=(const Monomial& m) const;

  /** Returns this monomial multiplied by @p m. */
  Monomial& operator*=(const Monomial& m);

  /** Returns this monomial raised to @p p.
   * @throws std::exception if @p p is negative.
   */
  Monomial& pow_in_place(int p);

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher, const Monomial& item) noexcept {
    using drake::hash_append;
    // We do not send total_degree_ to the hasher, because it is already fully
    // represented by powers_ -- it is just a cached tally of the exponents.
    hash_append(hasher, item.powers_);
  }

 private:
  int total_degree_{0};
  std::map<Variable, int> powers_;
  friend std::ostream& operator<<(std::ostream& out, const Monomial& m);
};

std::ostream& operator<<(std::ostream& out, const Monomial& m);

/** Returns a multiplication of two monomials, @p m1 and @p m2. */
Monomial operator*(Monomial m1, const Monomial& m2);

/** Returns @p m1 raised to @p p.
 * @throws std::exception if @p p is negative.
 */
Monomial pow(Monomial m, int p);
}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Monomial>. */
template <>
struct hash<drake::symbolic::Monomial>
    : public drake::DefaultHash {};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Monomial>.
template <>
struct NumTraits<drake::symbolic::Monomial>
    : GenericNumTraits<drake::symbolic::Monomial> {
  static inline int digits10() { return 0; }
};

namespace internal {
// Informs Eigen how to cast drake::symbolic::Monomial to
// drake::symbolic::Expression.
template <>
EIGEN_DEVICE_FUNC inline drake::symbolic::Expression cast(
    const drake::symbolic::Monomial& m) {
  return m.ToExpression();
}
}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
