#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>

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

  ~ChebyshevBasisElement() = default;

  /**
   * Compares two ChebyshevBasisElement in lexicographic order.
   */
  bool operator<(const ChebyshevBasisElement& other) const;

 private:
  double DoEvaluate(double variable_val, int degree) const override;
  Expression DoToExpression() const override;
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
