#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * ChebyshevBasisElement represents an element of Chebyshev polynomial basis,
 * written as the product of Chebyshev polynomials, in the form
 * Tₚ₀(x₀)Tₚ₁(x₁)...Tₚₙ(xₙ), where each Tₚᵢ(xᵢ) is a (univariate) Chebyshev
 * polynomial of degree pᵢ.
 */
class ChebyshevBasisElement : public PolynomialBasisElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ChebyshevBasisElement)

  /** Constructs a ChebyshevBasisElement equals to 1. */
  ChebyshevBasisElement();

  explicit ChebyshevBasisElement(
      const std::map<Variable, int>& var_to_degree_map);

  /** Constructs a Chebyshev polynomial T₁(var). */
  explicit ChebyshevBasisElement(const Variable& var);

  /** Constructs a Chebyshev polynomial Tₙ(var) where n = degree. */
  ChebyshevBasisElement(const Variable& var, int degree);

  /** Constructs a default value 1.  This overload is used by Eigen when
   * EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.
   */
  explicit ChebyshevBasisElement(std::nullptr_t);

  ChebyshevBasisElement(const Eigen::Ref<const VectorX<Variable>>& vars,
                        const Eigen::Ref<const Eigen::VectorXi>& degrees);

  ~ChebyshevBasisElement() override = default;

  /**
   * Compares two ChebyshevBasisElement in lexicographic order.
   */
  bool operator<(const ChebyshevBasisElement& other) const;

  /**
   * Differentiates the ChebyshevBasisElement with respect to a variable.
   * We use the fact that
   * - If n is even dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x), j is odd  and 1 <= j <= n-1
   * - If n is odd dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x) - n, j is even and 0 <= j <= n-1
   * We return `result`, a map from ChebyshevBasisElement to double, such that
   * sum(result.key() * result[key]) is the differentiation of `this` w.r.t the
   * variable.
   * For example if n is even, dTₙ(x)Tₘ(y)/dx = 2n∑ⱼ Tⱼ(x)Tₘ(y), j is odd and
   * 1 <= j <= n-1, then the returned result is {T₁(x)Tₘ(y), 2n}, {T₃(x)Tₘ(y),
   * 2n}, ..., {T₂ₙ₋₁(x)Tₘ(y), 2n}. A special case is that @p var is not a
   * variable in `this`, then we return an empty map.
   * @param var A variable to differentiate with.
   */
  [[nodiscard]] std::map<ChebyshevBasisElement, double> Differentiate(
      const Variable& var) const;

  /**
   * Integrates a ChebyshevBasisElement for a variable.
   * We use the fact that
   * ∫ Tₙ(x)dx = 1/(2n+2)Tₙ₊₁(x) − 1/(2n−2)Tₙ₋₁(x)
   * A special case is ∫ T₀(x)dx = T₁(x)
   * @param var The variable to integrate. If @param var is not a variable in
   * this ChebyshevBasisElement, then the integration result is *this * T₁(var).
   * @retval result sum(key * result[key]) is the integration result. For
   * example, ∫ T₂(x)T₃(y)dx = 1/6*T₃(x)T₃(y) − 1/2 * T₁(x)T₃(y), then the
   * result is the map containing {T₃(x)T₃(y), 1/6} and {T₁(x)T₃(y), -1/2}.
   */
  [[nodiscard]] std::map<ChebyshevBasisElement, double> Integrate(
      const Variable& var) const;

  /** Merges this Chebyshev basis element with another Chebyshev basis element
   * @p other by merging their var_to_degree_map. After merging, the degree of
   * each variable is raised to the sum of the degree in each basis element (if
   * a variable does not show up in either one of the basis element, we regard
   * its degree to be 0). For example, merging T₁(x)T₃(y) and T₂(x)T₄(z) gets
   * T₃(x)T₃(y)T₄(z).
   */
  void MergeBasisElementInPlace(const ChebyshevBasisElement& other);

  /** Partially evaluates using a given environment @p env. The evaluation
   * result is of type pair<double, ChebyshevBasisElement>. The first component
   * (: double) represents the coefficient part while the second component
   * represents the remaining parts of the ChebyshevBasisElement which was not
   * evaluated, the product of the first and the second component is the result
   * of the partial evaluation. For example, if this ChebyshevBasisElement is
   * T₂(x)T₃(y)T₁(z), and @p env stores x→ 3, y→ 2, then the partial evaluation
   * is T₂(3)*T₃(2)*T₁(z) = 17 * 26 * T₁(z) = 442*T₁(z), then we return the pair
   * (442, T₁(z)).
   */
  [[nodiscard]] std::pair<double, ChebyshevBasisElement> EvaluatePartial(
      const Environment& env) const;

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const ChebyshevBasisElement& item) noexcept {
    using drake::hash_append;
    // We do not send total_degree_ to the hasher, because it is already fully
    // represented by var_to_degree_map_ -- it is just a cached tally of the
    // exponents.
    hash_append(hasher, item.var_to_degree_map());
  }

 private:
  [[nodiscard]] double DoEvaluate(double variable_val,
                                  int degree) const override;
  [[nodiscard]] Expression DoToExpression() const override;
};

/**
 * Returns the product of two Chebyshev basis elements.
 * Since Tₘ(x) * Tₙ(x) = 0.5 (Tₘ₊ₙ(x) + Tₘ₋ₙ(x)) if m >= n, the product of
 * Chebyshev basis elements is the weighted sum of several Chebyshev basis
 * elements. For example T₁(x)T₂(y) * T₃(x)T₁(y) = 0.25*(T₄(x)T₃(y) + T₂(x)T₃(y)
 * + T₄(x)T₁(y) + T₂(x)T₁(y))
 * @return the result of the product, from each ChebyshevBasisElement to its
 * coefficient. In the example above, it returns (T₄(x)T₃(y) -> 0.25),
 * (T₂(x)T₃(y) -> 0.25), (T₄(x)T₁(y) -> 0.25) and (T₂(x)T₁(y) -> 0.25)
 */
std::map<ChebyshevBasisElement, double> operator*(
    const ChebyshevBasisElement& a, const ChebyshevBasisElement& b);

std::ostream& operator<<(std::ostream& out, const ChebyshevBasisElement& m);
}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::ChebyshevBasisElement>. */
template <>
struct hash<drake::symbolic::ChebyshevBasisElement>
    : public drake::DefaultHash {};
}  // namespace std

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::ChebyshevBasisElement>.
template <>
struct NumTraits<drake::symbolic::ChebyshevBasisElement>
    : GenericNumTraits<drake::symbolic::ChebyshevBasisElement> {
  static inline int digits10() { return 0; }
};
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
