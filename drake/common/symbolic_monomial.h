#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
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

  /** Constructs a Monomial from @p powers. */
  explicit Monomial(const std::map<Variable, int>& powers);

  /**
   * Converts an expression to a monomial if the expression is written as
   * ∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.
   * @pre is_polynomial(e) should be true.
   */
  explicit Monomial(const Expression& e);

  /** Constructs a Monomial from @p var. */
  explicit Monomial(const Variable& var);

  /** Constructs a Monomial from @p var and @exponent. */
  Monomial(const Variable& var, int exponent);

  /** Returns the degree of this Monomial in a variable @p v. */
  int degree(const Variable& v) const;

  /** Returns the total degree of this Monomial. */
  int total_degree() const { return total_degree_; }

  /** Returns hash value. */
  size_t GetHash() const;

  /** Returns the set of variables in this monomial. */
  Variables GetVariables() const;

  /** Returns the internal representation of Monomial, the map from a base
   * (Variable) to its exponent (int).*/
  const std::map<Variable, int>& get_powers() const { return powers_; }

  /** Evaluates under a given environment @p env.
   *
   * @throws std::out_of_range exception if there is a variable in this monomial
   * whose assignment is not provided by @p env.
   */
  double Evaluate(const Environment& env) const;

  /** Substitutes using a given environment @p env. The substitution result is
   * of type pair<double, Monomial>. The first component (: double) represents
   * the coefficient part while the second component represents the remaining
   * parts of the Monomial which was not substituted. Note that users are
   * allowed to provide a partial environment.
   *
   * Example 1. Substitution with a fully-specified environment
   *     (x^3*y^2).Substitute({{x, 2}, {y, 3}})
   *   = (2^3 * 3^2 = 8 * 9 = 72, Monomial{} = 1).
   *
   * Example 1. Substitution with a partial environment
   *     (x^3*y^2).Substitute({{x, 2}})
   *   = (2^3 = 8, y^2).
   */
  std::pair<double, Monomial> Substitute(const Environment& env) const;

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
   * @throws std::runtime_error if @p p is negative.
   */
  Monomial& pow_in_place(int p);

 private:
  int total_degree_{0};
  std::map<Variable, int> powers_;
  friend std::ostream& operator<<(std::ostream& out, const Monomial& m);
};

std::ostream& operator<<(std::ostream& out, const Monomial& m);

/** Returns a multiplication of two monomials, @p m1 and @p m2. */
Monomial operator*(Monomial m1, const Monomial& m2);

/** Returns @p m1 raised to @p p.
 * @throws std::runtime_error if @p p is negative.
 */
Monomial pow(Monomial m, int p);
}  // namespace symbolic

/** Computes the hash value of a Monomial. */
template <>
struct hash_value<symbolic::Monomial> {
  size_t operator()(const symbolic::Monomial& m) const { return m.GetHash(); }
};

}  // namespace drake

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
