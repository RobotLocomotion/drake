#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <map>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * ChebyshevBasis represents a polynomial basis written as the product of
 * Chebyshev polynomials, in the form Tₚ₀(x₀)Tₚ₁(x₁)...Tₚₙ(xₙ), where each
 * Tₚᵢ(xᵢ) is a (univariate) Chebyshev polynomial of degree pᵢ.
 */
class ChebyshevBasis : public PolynomialBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ChebyshevBasis)

  explicit ChebyshevBasis(const std::map<Variable, int>& var_to_degree_map);

  ~ChebyshevBasis();

  /**
   * Compare two ChebyshevBasis in lexicographic order.
   */
  bool operator<(const ChebyshevBasis& other) const;

 private:
  double DoEvaluate(double variable_val, int degree) const override;
};

/**
 * Return the product of two Chebyshev basis.
 * Since Tₘ(x) * Tₙ(x) = 0.5 (Tₘ₊ₙ(x) + Tₘ₋ₙ(x)) if m >= n, the product of
 * Chebyshev basis is the weighted sum of several Chebyshev basises. For example
 * T₁(x)T₂(y) * T₃(x)T₁(y) = 0.25*(T₄(x)T₃(y) + T₂(x)T₃(y) + T₄(x)T₁(y) +
 * T₂(x)T₁(y))
 * @return the result of the product, from each ChebyshevBasis to its
 * coefficient. In the example above, it returns (T₄(x)T₃(y) -> 0.25),
 * (T₂(x)T₃(y) -> 0.25), (T₄(x)T₁(y) -> 0.25) and (T₂(x)T₁(y) -> 0.25)
 */
std::map<ChebyshevBasis, double> operator*(const ChebyshevBasis& a,
                                           const ChebyshevBasis& b);
}  // namespace symbolic
}  // namespace drake
