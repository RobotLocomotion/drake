#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic/expression.h"

// Remove with deprecation 2026-05-01.
#include <ostream>

// Some of our Eigen template specializations live in polynomial.h, so we
// must only have been included from that file.  This helps prevent us from
// triggering undefined behavior due to the order of template specializations.
#ifndef DRAKE_COMMON_SYMBOLIC_POLYNOMIAL_H
// NOLINTNEXTLINE(whitespace/line_length)
#error Do not directly include this file. Use "drake/common/symbolic/polynomial.h".
#endif

namespace drake {

namespace symbolic {

/** Represents a monomial, a product of powers of variables with non-negative
integer exponents. Note that it does not include the coefficient part of a
monomial. */
class Monomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Monomial);

  /** Constructs a monomial equal to 1. Namely the total degree is zero. */
  Monomial() = default;

  /** Constructs a default value.  This overload is used by Eigen when
  EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled. */
  explicit Monomial(std::nullptr_t) : Monomial() {}

  /** Constructs a monomial from `powers`.
  @throws std::exception if `powers` includes a negative exponent. */
  explicit Monomial(const std::map<Variable, int>& powers);

  /** Constructs a monomial from a vector of variables `vars` and their
  corresponding integer exponents `exponents`.
  For example, `%Monomial([x, y, z], [2, 0, 1])` constructs a monomial `x²z`.
  @pre The size of `vars` should be the same as the size of `exponents`.
  @throws std::exception if `exponents` includes a negative integer. */
  Monomial(const Eigen::Ref<const VectorX<Variable>>& vars,
           const Eigen::Ref<const Eigen::VectorXi>& exponents);

  /** Converts an expression to a monomial if the expression is written as
  ∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.
  @pre is_polynomial(e) should be true. */
  explicit Monomial(const Expression& e);

  /** Constructs a monomial from `var`. */
  explicit Monomial(const Variable& var);

  /** Constructs a monomial from `var` and `exponent`. */
  Monomial(const Variable& var, int exponent);

  /** Returns the degree of this monomial in a variable `v`. */
  int degree(const Variable& v) const;

  /** Returns the total degree of this monomial. */
  int total_degree() const { return total_degree_; }

  /** Returns the set of variables in this monomial. */
  Variables GetVariables() const;

  /** Returns the internal representation of %Monomial, the map from a base
  (Variable) to its exponent (int). */
  const std::map<Variable, int>& get_powers() const { return powers_; }

  /** Evaluates under a given environment `env`.
  @throws std::exception if there is a variable in this monomial whose
  assignment is not provided by `env`. */
  double Evaluate(const Environment& env) const;

  /** Evaluates the monomial for a batch of data.
  We return monomial_vals such that monomial_vals(j) is obtained by substituting
  `vars(i)` with `vars_values(i, j)`, note that `vars_values.rows() ==
  vars.rows()` and `vars_values.cols() == monomial_vals.rows()`.
  @param vars The variables whose value will be substituted. `vars` must contain
  all variables in this->GetVariables(). Also `vars` cannot contain any
  duplicate variables, namely vars(i) != vars(j) if i != j.
  @param vars_values The i'th column of `vars_values` is the i'th data for
  `vars`.
  @throw std::exception if `vars` doesn't contain all the variables in
  `this->GetVariables()`. */
  Eigen::VectorXd Evaluate(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
      const Eigen::Ref<const Eigen::MatrixXd>& vars_values) const;

  /** Partially evaluates using a given environment `env`. The evaluation result
  is of type pair<double, Monomial>. The first component (a double) represents
  the coefficient part while the second component represents the remaining parts
  of the monomial which was not evaluated.

  Example 1. Evaluate with a fully-specified environment
      (x³*y²).EvaluatePartial({{x, 2}, {y, 3}})
    = (2³ * 3² = 8 * 9 = 72, %Monomial{} = 1).

  Example 2. Evaluate with a partial environment
      (x³*y²).EvaluatePartial({{x, 2}})
    = (2³ = 8, y²). */
  std::pair<double, Monomial> EvaluatePartial(const Environment& env) const;

  /** Returns a symbolic expression representing this monomial. */
  Expression ToExpression() const;

  /** Checks if this monomial and `m` represent the same monomial. Two
  monomials are equal iff they contain the same variable raised to the same
  exponent. */
  bool operator==(const Monomial& m) const;

  /** Checks if this monomial and `m` do not represent the same monomial. */
  bool operator!=(const Monomial& m) const;

  /** Returns this monomial multiplied by `m`. */
  Monomial& operator*=(const Monomial& m);

  /** Returns this monomial raised to `p`.
  @throws std::exception if `p` is negative. */
  Monomial& pow_in_place(int p);

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  // NOLINTNEXTLINE(runtime/references) Per hash_append convention.
  friend void hash_append(HashAlgorithm& hasher,
                          const Monomial& item) noexcept {
    using drake::hash_append;
    // We do not send total_degree_ to the hasher, because it is already fully
    // represented by powers_ -- it is just a cached tally of the exponents.
    hash_append(hasher, item.powers_);
  }

  /** Returns the string representation of this monomial. */
  std::string to_string() const;

 private:
  int total_degree_{0};
  std::map<Variable, int> powers_;
};

DRAKE_DEPRECATED(
    "2026-05-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream& out, const Monomial& m);

/** Returns a multiplication of two monomials, `m1` and `m2`. */
Monomial operator*(Monomial m1, const Monomial& m2);

/** Returns `m` raised to `p`.
@throws std::exception if `p` is negative. */
Monomial pow(Monomial m, int p);
}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Monomial>. */
template <>
struct hash<drake::symbolic::Monomial> : public drake::DefaultHash {};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
/* Eigen scalar type traits for Matrix<drake::symbolic::Monomial>. */
template <>
struct NumTraits<drake::symbolic::Monomial>
    : GenericNumTraits<drake::symbolic::Monomial> {
  constexpr static int digits() { return 0; }
  constexpr static int digits10() { return 0; }
  constexpr static int max_digits10() { return 0; }
};

namespace internal {
/* Informs Eigen how to cast drake::symbolic::Monomial to
drake::symbolic::Expression. */
template <>
EIGEN_DEVICE_FUNC inline drake::symbolic::Expression cast(
    const drake::symbolic::Monomial& m) {
  return m.ToExpression();
}
}  // namespace internal
}  // namespace Eigen

DRAKE_FORMATTER_AS(, drake::symbolic, Monomial, x, x.to_string())
#endif  // !defined(DRAKE_DOXYGEN_CXX)
