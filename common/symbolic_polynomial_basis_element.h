#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
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
 * - std::map<Derived, double> Derived::Integrate(const Variable& var) const;
 * - bool Derived::operator<(const Derived& other) const;
 * - std::pair<double, Derived> EvaluatePartial(const Environment& e) const;
 * - void MergeBasisElementInPlace(const Derived& other)
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
  PolynomialBasisElement() = default;

  /**
   * Constructs a polynomial basis given the variable and the degree of that
   * variable.
   * @throws std::exception if any of the degree is negative.
   * @note we will ignore the variable with degree 0.
   */
  explicit PolynomialBasisElement(
      const std::map<Variable, int>& var_to_degree_map);

  /**
   * Constructs a polynomial basis, such that it contains the variable-to-degree
   * map vars(i)→degrees(i).
   * @throws std::exception if @p vars contains repeated variables.
   * @throws std::exception if any degree is negative.
   */
  PolynomialBasisElement(const Eigen::Ref<const VectorX<Variable>>& vars,
                         const Eigen::Ref<const Eigen::VectorXi>& degrees);

  virtual ~PolynomialBasisElement() = default;

  [[nodiscard]] const std::map<Variable, int>& var_to_degree_map() const {
    return var_to_degree_map_;
  }

  /**
   * Returns variable to degree map.
   * TODO(hongkai.dai): this function is added because Monomial class has
   * get_powers() function. We will remove this get_powers() function when
   * Monomial class is deprecated.
   */
  [[nodiscard]] const std::map<Variable, int>& get_powers() const {
    return var_to_degree_map_;
  }

  /** Returns the total degree of a polynomial basis. This is the summation of
   * the degree for each variable. */
  [[nodiscard]] int total_degree() const { return total_degree_; }

  /** Returns the degree of this PolynomialBasisElement in a variable @p v. If
   * @p v is not a variable in this PolynomialBasisElement, then returns 0.*/
  [[nodiscard]] int degree(const Variable& v) const;

  [[nodiscard]] Variables GetVariables() const;

  /** Evaluates under a given environment @p env.
   *
   * @throws std::exception exception if there is a variable in this
   * monomial whose assignment is not provided by @p env.
   */
  [[nodiscard]] double Evaluate(const Environment& env) const;

  bool operator==(const PolynomialBasisElement& other) const;

  bool operator!=(const PolynomialBasisElement& other) const;

  [[nodiscard]] Expression ToExpression() const;

 protected:
  /**
   * Compares two PolynomialBasisElement using lexicographical order. This
   * function is meant to be called by the derived class, to compare two
   * polynomial basis of the same derived class.
   */
  [[nodiscard]] bool lexicographical_compare(
      const PolynomialBasisElement& other) const;

  [[nodiscard]] virtual bool EqualTo(const PolynomialBasisElement& other) const;

  // Partially evaluate a polynomial basis element, where @p e does not
  // necessarily contain all the variables in this basis element. The
  // evaluation result is coeff * new_basis_element.
  void DoEvaluatePartial(const Environment& e, double* coeff,
                         std::map<Variable, int>* new_basis_element) const;

  int* get_mutable_total_degree() { return &total_degree_; }

  std::map<Variable, int>* get_mutable_var_to_degree_map() {
    return &var_to_degree_map_;
  }

  /** Merge this basis element with another basis element by merging their
   * var_to_degree_map. After merging, the degree of each variable is raised to
   * the sum of the degree in each basis element (if a variable does not show up
   * in either one of the basis element, we regard its degree to be 0).
   */
  void DoMergeBasisElementInPlace(const PolynomialBasisElement& other);

 private:
  // This function evaluates the polynomial basis for a univariate polynomial at
  // a given degree. For example, for a monomial basis, this evaluates xⁿ where
  // x is the variable value and n is the degree; for a Chebyshev basis, this
  // evaluats the Chebyshev polynomial Tₙ(x).
  [[nodiscard]] virtual double DoEvaluate(double variable_val,
                                          int degree) const = 0;

  [[nodiscard]] virtual Expression DoToExpression() const = 0;

  // Internally, the polynomial basis is represented as a mapping from a
  // variable to its degree.
  std::map<Variable, int> var_to_degree_map_;
  int total_degree_{};
};

/** Implements Graded reverse lexicographic order.
 *
 * @tparam VariableOrder VariableOrder{}(v1, v2) is true if v1 < v2.
 * @tparam BasisElement A derived class of PolynomialBasisElement.
 *
 * We first compare the total degree of the PolynomialBasisElement; if there is
 * a tie, then we use the graded reverse lexicographical order as the tie
 * breaker.
 *
 * Take monomials with variables {x, y, z} and total degree<=2 as an
 * example, with the order x > y > z. To get the graded reverse lexicographical
 * order, we take the following steps:
 *
 * First find all the monomials using the total degree. The monomials with
 * degree 2 are {x², y², z², xy, xz, yz}. The monomials with degree 1 are {x,
 * y, z}, and the monomials with degree 0 is {1}. To break the tie between
 * monomials with the same total degree, first sort them in the reverse
 * lexicographical order, namely x < y < z. The lexicographical order compares
 * two monomials by first comparing the exponent of the largest variable, if
 * there is a tie then go forth to the second largest variable. Thus z² > zy >zx
 * > y² > yx > x². Finally reverse the order as x² > xy > y² > xz > yz > z² > x
 * > y > z.
 *
 * There is an introduction to monomial order in
 * https://en.wikipedia.org/wiki/Monomial_order, and an introduction to graded
 * reverse lexicographical order in
 * https://en.wikipedia.org/wiki/Monomial_order#Graded_reverse_lexicographic_order
 */
template <typename VariableOrder, typename BasisElement>
struct BasisElementGradedReverseLexOrder {
  /** Returns true if m1 < m2 under the Graded reverse lexicographic order. */
  bool operator()(const BasisElement& m1, const BasisElement& m2) const {
    const int d1{m1.total_degree()};
    const int d2{m2.total_degree()};
    if (d1 > d2) {
      return false;
    }
    if (d2 > d1) {
      return true;
    }
    // d1 == d2
    if (d1 == 0) {
      // Because both of them are 1.
      return false;
    }
    const std::map<Variable, int>& powers1{m1.get_powers()};
    const std::map<Variable, int>& powers2{m2.get_powers()};
    std::map<Variable, int>::const_iterator it1{powers1.cbegin()};
    std::map<Variable, int>::const_iterator it2{powers2.cbegin()};
    while (it1 != powers1.cend() && it2 != powers2.cend()) {
      const Variable& var1{it1->first};
      const Variable& var2{it2->first};
      const int degree1{it1->second};
      const int degree2{it2->second};
      if (variable_order_(var2, var1)) {
        return false;
      } else if (variable_order_(var1, var2)) {
        return true;
      } else {
        // var1 == var2
        if (degree1 == degree2) {
          ++it1;
          ++it2;
        } else {
          return degree1 > degree2;
        }
      }
    }
    // When m1 and m2 are identical.
    return false;
  }

 private:
  VariableOrder variable_order_;
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
