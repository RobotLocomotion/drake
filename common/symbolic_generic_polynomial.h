#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>
#include <ostream>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents symbolic generic polynomials using a given basis (for example,
 * monomial basis, Chebyshev basis, etc). A generic symbolic polynomial keeps a
 * mapping from a basis element of indeterminates to its coefficient in a
 * symbolic expression. A generic polynomial `p` has to satisfy an invariant
 * such that `p.decision_variables() ∩ p.indeterminates() = ∅`. We have
 * CheckInvariant() method to check the invariant.
 *
 * Note that arithmetic operations (+,-,*) between a Polynomial and a Variable
 * are not provided. The problem is that Variable class has no intrinsic
 * information if a variable is a decision variable or an indeterminate while we
 * need this information to perform arithmetic operations over Polynomials.
 * We provide two instantiations of this template
 * - BasisElement = MonomialBasisElement
 * - BasisElement = ChebyshevBasisElement
 * @tparam BasisElement Must be a subclass of PolynomialBasisElement.
 */
template <typename BasisElement>
class GenericPolynomial {
 public:
  static_assert(
      std::is_base_of<PolynomialBasisElement, BasisElement>::value,
      "BasisElement should be a derived class of PolynomialBasisElement");
  /** Type of mapping from basis element to coefficient */
  using MapType = std::map<BasisElement, Expression>;

  /** Constructs a zero polynomial. */
  GenericPolynomial() = default;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericPolynomial)

  /** Constructs a default value. This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit GenericPolynomial(std::nullptr_t)
      : GenericPolynomial<BasisElement>() {}

  /** Constructs a generic polynomial from a map, basis_element → coefficient.
   * For example
   * GenericPolynomial<MonomialBasiElement>({{MonomialBasisElement(x, 2), a},
   * {MonomialBasisElement(x, 3), a+b}}) constructs a polynomial ax²+(a+b)x³.*/
  explicit GenericPolynomial(MapType init);

  /** Constructs a generic polynomial from a single basis element @p m. Note
   * that all variables in `m` are considered as indeterminates. Namely the
   * constructed generic polynomial contains the map with a single key `m`, with
   * the coefficient being 1.
   */
  // Note that this implicit conversion is desirable to have a dot product of
  // two Eigen::Vector<BasisElement>s return a GenericPolynomial<BasisElement>.
  // NOLINTNEXTLINE(runtime/explicit)
  GenericPolynomial(const BasisElement& m);

  /** Constructs a polynomial from an expression @p e. Note that all variables
   * in `e` are considered as indeterminates.
   *
   * @throws std::runtime_error if @p e is not a polynomial.
   */
  explicit GenericPolynomial(const Expression& e);

  /** Constructs a polynomial from an expression @p e by decomposing it with
   * respect to @p indeterminates.
   *
   * @note The indeterminates for the polynomial are @p indeterminates. Even if
   * a variable in @p indeterminates does not show up in @p e, that variable is
   * still registered as an indeterminate in this polynomial, as
   * this->indeterminates() be the same as @p indeterminates.
   *
   * @throws std::runtime_error if @p e is not a polynomial in @p
   * indeterminates.
   */
  GenericPolynomial(const Expression& e, Variables indeterminates);

  /** Returns the indeterminates of this generic polynomial. */
  [[nodiscard]] const Variables& indeterminates() const {
    return indeterminates_;
  }

  /** Returns the decision variables of this generic polynomial. */
  [[nodiscard]] const Variables& decision_variables() const {
    return decision_variables_;
  }

  /** Returns the map from each basis element to its coefficient. */
  [[nodiscard]] const MapType& basis_element_to_coefficient_map() const {
    return basis_element_to_coefficient_map_;
  }

  /** Returns the highest degree of this generic polynomial in an indeterminate
   * @p v. */
  [[nodiscard]] int Degree(const Variable& v) const;

  /** Returns the total degree of this generic polynomial. */
  [[nodiscard]] int TotalDegree() const;

  /** Returns an equivalent symbolic expression of this generic polynomial.*/
  [[nodiscard]] Expression ToExpression() const;

  /**
   * Differentiates this generic polynomial with respect to the variable @p x.
   * Note that a variable @p x can be either a decision variable or an
   * indeterminate.
   */
  [[nodiscard]] GenericPolynomial<BasisElement> Differentiate(
      const Variable& x) const;

  /** Computes the Jacobian matrix J of the generic polynomial with respect to
   * @p vars. J(0,i) contains ∂f/∂vars(i). @p vars should be an Eigen column
   * vector of symbolic variables.
   */
  template <typename Derived>
  Eigen::Matrix<GenericPolynomial<BasisElement>, 1, Derived::RowsAtCompileTime>
  Jacobian(const Eigen::MatrixBase<Derived>& vars) const {
    static_assert(
        std::is_same_v<typename Derived::Scalar, Variable> &&
            (Derived::ColsAtCompileTime == 1),
        "The argument of GenericPolynomial::Jacobian() should be a vector of "
        "symbolic variables.");
    const VectorX<Expression>::Index n{vars.size()};
    Eigen::Matrix<GenericPolynomial<BasisElement>, 1,
                  Derived::RowsAtCompileTime>
        J{n};
    for (VectorX<Expression>::Index i = 0; i < n; ++i) {
      J(i) = this->Differentiate(vars(i));
    }
    return J;
  }

  /**
   * Evaluates this generic polynomial under a given environment @p env.
   *
   * @throws std::invalid_argument if there is a variable in this generic
   * polynomial whose assignment is not provided by @p env.
   */
  [[nodiscard]] double Evaluate(const Environment& env) const;

  /** Partially evaluates this generic polynomial using an environment @p env.

   * @throws std::runtime_error if NaN is detected during evaluation.
   */
  [[nodiscard]] GenericPolynomial<BasisElement> EvaluatePartial(
      const Environment& env) const;

  /** Partially evaluates this generic polynomial by substituting @p var with @p
   * c.

   * @throws std::runtime_error if NaN is detected at any point during
   * evaluation.
   */
  [[nodiscard]] GenericPolynomial<BasisElement> EvaluatePartial(
      const Variable& var, double c) const;

  /** Adds @p coeff * @p m to this generic polynomial. */
  GenericPolynomial<BasisElement> AddProduct(const Expression& coeff,
                                             const BasisElement& m);

  /** Removes the terms whose absolute value of the coefficients are smaller
   * than or equal to @p coefficient_tol.
   * For example, if the generic polynomial is 2x² + 3xy + 10⁻⁴x - 10⁻⁵,
   * then after calling RemoveTermsWithSmallCoefficients(1e-3), the returned
   * polynomial becomes 2x² + 3xy.
   * @param coefficient_tol A positive scalar.
   * @retval polynomial_cleaned A generic polynomial whose terms with small
   * coefficients are removed.
   */
  [[nodiscard]] GenericPolynomial<BasisElement>
  RemoveTermsWithSmallCoefficients(double coefficient_tol) const;

  /** Returns true if this generic polynomial and @p p are structurally equal.
   */
  [[nodiscard]] bool EqualTo(const GenericPolynomial<BasisElement>& p) const;

  /** Returns true if this generic polynomial and @p p are equal after expanding
   * the coefficients. */
  [[nodiscard]] bool EqualToAfterExpansion(
      const GenericPolynomial<BasisElement>& p) const;

 private:
  // Throws std::runtime_error if there is a variable appeared in both of
  // decision_variables() and indeterminates().
  void CheckInvariant() const;

  MapType basis_element_to_coefficient_map_;
  Variables indeterminates_;
  Variables decision_variables_;
};

template <typename BasisElement>
std::ostream& operator<<(std::ostream& os,
                         const GenericPolynomial<BasisElement>& p) {
  const typename GenericPolynomial<BasisElement>::MapType& map{
      p.basis_element_to_coefficient_map()};
  if (map.empty()) {
    return os << 0;
  }
  auto it = map.begin();
  os << it->second << "*" << it->first;
  for (++it; it != map.end(); ++it) {
    os << " + " << it->second << "*" << it->first;
  }
  return os;
}

extern template class GenericPolynomial<MonomialBasisElement>;
extern template class GenericPolynomial<ChebyshevBasisElement>;
}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Defines Eigen traits needed for Matrix<drake::symbolic::Polynomial>.
template <>
struct NumTraits<
    drake::symbolic::GenericPolynomial<drake::symbolic::MonomialBasisElement>>
    : GenericNumTraits<drake::symbolic::GenericPolynomial<
          drake::symbolic::MonomialBasisElement>> {
  static inline int digits10() { return 0; }
};

template <>
struct NumTraits<
    drake::symbolic::GenericPolynomial<drake::symbolic::ChebyshevBasisElement>>
    : GenericNumTraits<drake::symbolic::GenericPolynomial<
          drake::symbolic::ChebyshevBasisElement>> {
  static inline int digits10() { return 0; }
};
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
