#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * MonomialBasisElement represents a monomial, a product of powers of variables
 * with non-negative integer exponents. Note that it doesn't not include the
 * coefficient part of a monomial. So x, x³y, xy²z are all valid
 * MonomialBasisElement instances, but 1+x or 2xy²z are not.
 * TODO(hongkai.dai): deprecate Monomial class and replace Monomial class with
 * MonomialBasisElement class.
 * For more information regarding the motivation of this class, please see
 * Drake github issue #13602 and #13803.
 */
class MonomialBasisElement : public PolynomialBasisElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MonomialBasisElement)

  /** Constructs a monomial equal to 1. Namely the toal degree is zero. */
  MonomialBasisElement();

  /** Constructs a default value.  This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit MonomialBasisElement(std::nullptr_t) : MonomialBasisElement() {}

  /**
   * Constructs a MonomialBasisElement from variable to degree map.
   */
  explicit MonomialBasisElement(
      const std::map<Variable, int>& var_to_degree_map);

  /**
   * Converts an expression to a monomial if the expression is written as
   * ∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.
   * @pre is_polynomial(e) should be true.
   */
  explicit MonomialBasisElement(const Expression& e);

  /** Constructs a Monomial from a vector of variables `vars` and their
   * corresponding integer degrees `degrees`.
   * For example, `MonomialBasisElement([x, y, z], [2, 0, 1])` constructs a
   * MonomialBasisElement `x²z`.
   *
   * @pre The size of `vars` should be the same as the size of `degrees`.
   * @throws std::exception if `degrees` includes a negative integer.
   */
  MonomialBasisElement(const Eigen::Ref<const VectorX<Variable>>& vars,
                       const Eigen::Ref<const Eigen::VectorXi>& degrees);

  /**
   * Constructs a monomial basis element with only one variable, and the degree
   * is 1.
   */
  explicit MonomialBasisElement(const Variable& var);

  /**
   * Constructs a monomial basis element with only one variable, and the degree
   * of that variable is given by @p degree.
   */
  MonomialBasisElement(const Variable& var, int degree);

  /** Partially evaluates using a given environment @p env. The evaluation
   * result is of type pair<double, MonomialBasisElement>. The first component
   * (: double) represents the coefficient part while the second component
   * represents the remaining parts of the MonomialBasisElement which was not
   * evaluated.
   *
   * Example 1. Evaluate with a fully-specified environment
   *     (x³*y²).EvaluatePartial({{x, 2}, {y, 3}})
   *   = (2³ * 3² = 8 * 9 = 72, MonomialBasisElement{} = 1).
   *
   * Example 2. Evaluate with a partial environment
   *     (x³*y²).EvaluatePartial({{x, 2}})
   *   = (2³ = 8, y²).
   */
  [[nodiscard]] std::pair<double, MonomialBasisElement> EvaluatePartial(
      const Environment& env) const;

  /** Returns this monomial raised to @p p.
   * @throws std::exception if @p p is negative.
   */
  MonomialBasisElement& pow_in_place(int p);

  /**
   * Compares two MonomialBasisElement in lexicographic order.
   */
  bool operator<(const MonomialBasisElement& other) const;

  /**
   * Differentiates this MonomialBasisElement.
   * Since dxⁿ/dx = nxⁿ⁻¹, we return the map from the MonomialBasisElement to
   * its coefficient. So if this MonomialBasisElement is x³y², then
   * differentiate with x will return (x²y² → 3) as dx³y²/dx = 3x²y²
   * If @p var is not a variable in MonomialBasisElement, then returns an empty
   * map.
   */
  [[nodiscard]] std::map<MonomialBasisElement, double> Differentiate(
      const Variable& var) const;

  /**
   * Integrates this MonomialBasisElement on a variable.
   * Since ∫ xⁿ dx = 1 / (n+1) xⁿ⁺¹, we return the map from the
   * MonomialBasisElement to its coefficient in the integration result. So if
   * this MonomialBasisElement is x³y², then we return (x⁴y² → 1/4) as ∫ x³y²dx
   * = 1/4 x⁴y². If @p var is not a variable in this MonomialBasisElement, for
   * example ∫ x³y²dz = x³y²z, then we return (x³y²z → 1)
   */
  [[nodiscard]] std::map<MonomialBasisElement, double> Integrate(
      const Variable& var) const;

  /** Merges this basis element with another basis element @p other by merging
   * their var_to_degree_map. This is equivalent to multiplying this monomial
   * basis element in place with monomial basis element @p other.
   */
  void MergeBasisElementInPlace(const MonomialBasisElement& other);

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const MonomialBasisElement& item) noexcept {
    using drake::hash_append;
    // We do not send total_degree_ to the hasher, because it is already fully
    // represented by var_to_degree_map_ -- it is just a cached tally of the
    // exponents.
    hash_append(hasher, item.var_to_degree_map());
  }

  /**
   * Converts this monomial to Chebyshev polynomial basis. For example,
   *
   *  - For x², it returns 0.5T₂(x) + 0.5T₀(x).
   *  - For x²y³, it returns 1/8T₂(x)T₃(y) + 3/8T₂(x)T₁(y) + 1/8T₀(x)T₃(y) +
   *    3/8T₀(x)T₁(y).
   *
   * We return the map from each ChebyshevBasisElement to its coefficient.
   * For example, when this = x², it returns {[T₂(x)⇒0.5], [T₀(x)⇒0.5]}.
   * When this = x²y³, it returns {[T₂(x)T₃(y)⇒1/8], [T₂(x)T₁(y)⇒3/8],
   * [T₀(x)T₃(y)⇒1/8], [T₀(x)T₁(y)⇒3/8]}.
   */
  [[nodiscard]] std::map<ChebyshevBasisElement, double> ToChebyshevBasis()
      const;

  /**
   * Converts this monomial to a weighted sum of basis elements of type
   * BasisElement. We return the map from each BasisElement to its coefficient.
   * For example, if BasisElement=ChebyshevBasisElement, then when this = x²y³,
   * it returns {[T₂(x)T₃(y)⇒1/8], [T₂(x)T₁(y)⇒3/8], [T₀(x)T₃(y)⇒1/8],
   * [T₀(x)T₁(y)⇒3/8]}.
   * @note Currently we only support @tparam BasisElement being
   * MonomialBasisElement and ChebyshevBasisElement.
   */
  template <typename BasisElement>
  std::map<BasisElement, double> ToBasis() const {
    static_assert(std::is_same_v<BasisElement, MonomialBasisElement> ||
                      std::is_same_v<BasisElement, ChebyshevBasisElement>,
                  "MonomialBasisElement::ToBasis() does not support this "
                  "BasisElement type.");
    if constexpr (std::is_same_v<BasisElement, MonomialBasisElement>) {
      return {{*this, 1.}};
    }
    return ToChebyshevBasis();
  }

 private:
  [[nodiscard]] double DoEvaluate(double variable_val,
                                  int degree) const override;
  [[nodiscard]] Expression DoToExpression() const override;
};

std::ostream& operator<<(std::ostream& out, const MonomialBasisElement& m);

/** Returns a multiplication of two monomials, @p m1 and @p m2.
 * @note that we return a map from the monomial product to its coefficient. This
 * map has size 1, and the coefficient is also 1. We return a map instead of the
 * MonomialBasisElement directly, because we want operator* to have the same
 * return signature as other PolynomialBasisElement. For example, the product
 * between two ChebyshevBasisElement objects is a weighted sum of
 * ChebyshevBasisElement objects.
 * @note we do not provide operator*= function for this class, since operator*=
 * would return MonomialBasisElement, which is different from operator*.
 */
std::map<MonomialBasisElement, double> operator*(
    const MonomialBasisElement& m1, const MonomialBasisElement& m2);

/** Returns @p m raised to @p p.
 * @note that we return a map from the monomial power to its coefficient. This
 * map has size 1, and the coefficient is also 1. We return a map instead of the
 * MonomialBasisElement directly, because we want pow() to have the same
 * return signature as other PolynomialBasisElement. For example, the power
 * of a ChebyshevBasisElement object is a weighted sum of ChebyshevBasisElement
 * objects.
 * @throws std::exception if @p p is negative.
 */
std::map<MonomialBasisElement, double> pow(MonomialBasisElement m, int p);
}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::MonomialBasisElement>. */
template <>
struct hash<drake::symbolic::MonomialBasisElement> : public drake::DefaultHash {
};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::MonomialBasisElement>.
template <>
struct NumTraits<drake::symbolic::MonomialBasisElement>
    : GenericNumTraits<drake::symbolic::MonomialBasisElement> {
  static inline int digits10() { return 0; }
};

namespace internal {
// Informs Eigen how to cast drake::symbolic::MonomialBasisElement to
// drake::symbolic::Expression.
template <>
EIGEN_DEVICE_FUNC inline drake::symbolic::Expression cast(
    const drake::symbolic::MonomialBasisElement& m) {
  return m.ToExpression();
}
}  // namespace internal
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
