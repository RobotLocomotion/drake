#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Each polynomial p(x) can be written as a linear combination of its basis
 * elements p(x) = ∑ᵢ cᵢ * ϕᵢ(x), where ϕᵢ(x) is the i'th element in the basis,
 * cᵢ is the coefficient of that element. The most commonly used basis is
 * monomials. For example in polynomial p(x) = 2x₀²x₁ + 3x₀x₁ + 2, x₀²x₁, x₀x₁
 * and 1 are all elements of monomial basis. Likewise, a polynomial can be
 * written using other basis, such as Chebyshev polynomials, Legendre
 * polynomials, etc. For a polynomial written with Chebyshev polynomial basis
 * p(x) = 2T₂(x₀)T₁(x₁) + 3T₁(x₁) + 2T₂(x₀), T₂(x₀)T₁(x₁),T₁(x₁), and T₂(x₀) are
 * all elements of Chebyshev basis. This PolynomialBasisElement class represents
 * an element ϕᵢ(x) in the basis. We can think of an element of polynomial basis
 * as a mapping from the variable to its degree. So for monomial basis element
 * x₀²x₁, it can be thought of as a mapping {x₀ -> 2, x₁ -> 1}. For a Chebyshev
 * basis element T₂(x₀)T₁(x₁), it can be thought of as a mapping {x₀ -> 2, x₁ ->
 * 1}.
 *
 * Each of the derived class, `Derived`, should implement the following
 * functions
 *
 * - std::map<Derived, double> operator*(const Derived& A, const Derived&B)
 * - std::map<Derived, double> Derived::Differentiate(const Variable& var)
 * const;
 * - bool Derived::operator<(const Derived& other) const;
 * - std::pair<double, Derived> EvaluatePartial(const Environment& e) const;
 *
 * The function lexicographical_compare can be used when implementing operator<.
 * The function DoEvaluatePartial can be used when implementing EvaluatePartial
 */
class PolynomialBasisElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolynomialBasisElement)

  /**
   * Constructs a polynomial basis with empty var_to_degree map. This element
   * should be interpreted as 1.
   */
  PolynomialBasisElement();

  /**
   * Constructs a polynomial basis given the variable and the degree of that
   * variable.
   * @throw std::logic_error if any of the degree is negative.
   * @note we will ignore the variable with degree 0.
   */
  explicit PolynomialBasisElement(
      const std::map<Variable, int>& var_to_degree_map);

  PolynomialBasisElement(const Eigen::Ref<const VectorX<Variable>>& vars,
                         const Eigen::Ref<const Eigen::VectorXi>& degrees);

  virtual ~PolynomialBasisElement() = default;

  const std::map<Variable, int>& var_to_degree_map() const {
    return var_to_degree_map_;
  }

  /**
   * Returns variable to degree map.
   * TODO(hongkai.dai): this function is added because Monomial class has
   * get_powers() function. We will remove this get_powers() function when
   * Monomial class is deprecated.
   */
  const std::map<Variable, int>& get_powers() const {
    return var_to_degree_map_;
  }

  /** Returns the total degree of a polynomial basis. This is the summation of
   * the degree for each variable. */
  int total_degree() const { return total_degree_; }

  /** Returns the degree of this PolynomialBasisElement in a variable @p v. If
   * @p v is not a variable in this PolynomialBasisElement, then returns 0.*/
  int degree(const Variable& v) const;

  Variables GetVariables() const;

  /** Evaluates under a given environment @p env.
   *
   * @throws std::invalid_argument exception if there is a variable in this
   * monomial whose assignment is not provided by @p env.
   */
  double Evaluate(const Environment& env) const;

  bool operator==(const PolynomialBasisElement& other) const;

  bool operator!=(const PolynomialBasisElement& other) const;

  symbolic::Expression ToExpression() const;

 protected:
  /**
   * Compares two PolynomialBasisElement using lexicographical order. This
   * function is meant to be called by the derived class, to compare two
   * polynomial basis of the same derived class.
   */
  bool lexicographical_compare(const PolynomialBasisElement& other) const;

  virtual bool EqualTo(const PolynomialBasisElement& other) const;

  // Partially evaluate a polynomial basis element, where @p e doesn't contain
  // all the variables in this basis element. The evaluation result is coeff *
  // new_basis_element
  void DoEvaluatePartial(const Environment& e, double* coeff,
                         std::map<Variable, int>* new_basis_element) const;

  int* get_mutable_total_degree() { return &total_degree_; }

  std::map<Variable, int>* get_mutable_var_to_degree_map() {
    return &var_to_degree_map_;
  }

 private:
  // This function evaluates the polynomial basis for a univariate polynomial at
  // a given degree. For example, for a monomial basis, this evaluates xⁿ where
  // x is the variable value and n is the degree; for a Chebyshev basis, this
  // evaluats the Chebyshev polynomial Tₙ(x).
  virtual double DoEvaluate(double variable_val, int degree) const = 0;

  virtual Expression DoToExpression() const = 0;

  // Internally, the polynomial basis is represented as a mapping from a
  // variable to its degree.
  std::map<Variable, int> var_to_degree_map_;
  int total_degree_{};
};

}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
namespace internal {
// Informs Eigen how to cast drake::symbolic::PolynomialBasisElement to
// drake::symbolic::Expression.
template <>
EIGEN_DEVICE_FUNC inline drake::symbolic::Expression cast(
    const drake::symbolic::PolynomialBasisElement& m) {
  return m.ToExpression();
}
}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
