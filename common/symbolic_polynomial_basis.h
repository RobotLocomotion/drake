#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Each polynomial p(x) can be written as a linear combination of its basis
 * p(x) = ∑ᵢ cᵢ * ϕᵢ(x), where ϕᵢ(x) is the i'th basis, cᵢ is the coefficient
 * of that basis. The most commonly used basis is monomials. For example
 * in polynomial p(x) = 2x₀²x₁ + 3x₀x₁ + 2, x₀²x₁, x₀x₁ and 1 are all its basis.
 * Likewise, a polynomial can be written using other basis, such as Chebyshev
 * polynomials, Legendre polynomials, etc. For a polynomial written with
 * Chebyshev polynomial basis p(x) = 2T₂(x₀)T₁(x₁) + 3T₁(x₁) + 2T₂(x₀),
 * T₂(x₀)T₁(x₁),T₁(x₁), and T₂(x₀) are all its basis. This PolynomialBasis class
 * represents a basis ϕᵢ(x). We can think of a polynomial basis as a mapping
 * from the variable to its degree. So for monomial basis x₀²x₁, it can be
 * thought of as a mapping {x₀ -> 2, x₁ -> 1}. For a Chebyshev basis
 * T₂(x₀)T₁(x₁), it can be thought of as a mapping {x₀ -> 2, x₁ -> 1}.
 *
 * Each of the derived class, `Derived`, should implement the following
 * functions
 *
 * - std::map<Derived, double> operator*(const Derived& A, const Derived&B)
 * - std::map<Derived, double> Derived::Differentiate(const Variable& var)
 * const;
 * - bool Derived::operator<(const Derived& other) const;
 */
class PolynomialBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolynomialBasis)

  /**
   * Constructs a polynomial basis given the variable and the degree of that
   * variable.
   * @throw std::logic_error if any of the degree is negative.
   * @note we will ignore the variable with degree 0.
   */
  explicit PolynomialBasis(const std::map<Variable, int>& var_to_degree_map);

  virtual ~PolynomialBasis() = default;

  const std::map<Variable, int>& var_to_degree_map() const {
    return var_to_degree_map_;
  }

  /** Returns the total degree of a polynomial basis. This is the summation of
   * the degree for each variable. */
  int total_degree() const { return total_degree_; }

  Variables GetVariables() const;

  /** Evaluates under a given environment @p env.
   *
   * @throws std::invalid_argument exception if there is a variable in this
   * monomial whose assignment is not provided by @p env.
   */
  double Evaluate(const Environment& env) const;

  bool operator==(const PolynomialBasis& other) const;

  bool operator!=(const PolynomialBasis& other) const;

 protected:
  /**
   * Compares two PolynomialBasis using lexicographical order. This function
   * is meant to be called by the derived class, to compare two polynomial basis
   * of the same derived class.
   */
  bool lexicographical_compare(const PolynomialBasis& other) const;

 protected:
  virtual bool EqualTo(const PolynomialBasis& other) const;

 private:
  // This function evaluates the polynomial basis for a univariate polynomial at
  // a given degree. For example, for a monomial basis, this evaluates xⁿ where
  // x is the variable value and n is the degree; for a Chebyshev basis, this
  // evaluats the Chebyshev polynomial Tₙ(x).
  virtual double DoEvaluate(double variable_val, int degree) const = 0;

  // Internally, the polynomial basis is represented as a mapping from a
  // variable to its degree.
  std::map<Variable, int> var_to_degree_map_;
  int total_degree_{};
};

}  // namespace symbolic
}  // namespace drake
