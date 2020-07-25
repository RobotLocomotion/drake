#pragma once
#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <cstddef>
#include <map>
#include <ostream>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents the Chebyshev polynomial of the first kind Tₙ(x).
 * One definition of Chebyshev polynomial of first kind is
 * Tₙ(cos(θ)) = cos(nθ)
 * It can also be defined recursively as
 *
 *     T₀(x) = 1
 *     T₁(x) = x
 *     Tₙ₊₁(x) = 2xTₙ(x) − Tₙ₋₁(x)
 */
class ChebyshevPolynomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ChebyshevPolynomial)

  /**
   * Construct a Chebyshev polynomial Tₙ(x)
   * @param var The variable x
   * @param degree The Chebyshev polynomial is of degree n. @pre degree >= 0.
   */
  ChebyshevPolynomial(const symbolic::Variable& var, int degree);

  const symbolic::Variable& var() const { return var_; }

  int degree() const { return degree_; }

  /**
   * Convert this Chebyshev polynomial to a polynomial with monomial basis.
   */
  symbolic::Polynomial ToPolynomial() const;

  /**
   * Evaluate this Chebyshev polynomial at @p var_val.
   */
  double Evaluate(double var_val) const;

  /**
   * Checks if this and @p other represent the same Chebyshev polynomial. Two
   * Chebyshev polynomials are equal iff their variable and degree are the same,
   * or they both have degree 0.
   * @note T₀(x) = T₀(y) = 1
   */
  bool operator==(const ChebyshevPolynomial& other) const;

  /** Checks if this and @p other do not represent the same Chebyshev
   * polynomial. */
  bool operator!=(const ChebyshevPolynomial& other) const;

  /**
   * Computes the differentiate of a Chebyshev polynomial
   * dTₙ(x)/dx = nUₙ₋₁(x)
   * If n is even
   * dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x), j is odd  and j <= n-1
   * If n is odd
   * dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x) - n, j is even and j <= n-1
   * A special case is that dT₀(x)/dx = 0.
   * @return chebyshev_coeff_pairs. sum(chebyshev_coeff_pairs[j].first *
   * chebyshev_coeff_pairs[j].second) is the differetiation dTₙ(x)/dx. If n is
   * even, then chebyshev_coeff_pairs[j] = (T₂ⱼ₋₁(x), 2n). If n is odd, then
   * chebyshev_coeff_pairs[j] = (T₂ⱼ(x), 2n) for j >= 1, and
   * chebyshev_coeff_pairs[0] = (T₀(x), n).
   * For the special case when degree() == 0, we return an empty vector.
   */
  std::vector<std::pair<ChebyshevPolynomial, double>> Differentiate() const;

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const ChebyshevPolynomial& item) noexcept {
    if (item.degree() == 0) {
      hash_append(hasher, item.degree());
    } else {
      hash_append(hasher, std::make_pair(item.var().get_id(), item.degree()));
    }
  }

 private:
  symbolic::Variable var_;
  int degree_;
};

std::ostream& operator<<(std::ostream& out, const ChebyshevPolynomial& p);

/**
 * Compare two Chebyshev polynomials, returns True if lhs is regarded as less
 * than rhs, otherwise returns false.
 *
 *    If lhs.var() < rhs.var(), return True.
 *    If lhs.var() > rhs.var(), return False.
 *    If lhs.var() == rhs.var(), then return lhs.degree() < rhs.degree().
 */
struct CompareChebyshevPolynomial {
  bool operator()(const ChebyshevPolynomial& lhs,
                  const ChebyshevPolynomial& rhs);
};

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::ChebyshevPolynomial>. */
template <>
struct hash<drake::symbolic::ChebyshevPolynomial> : public drake::DefaultHash {
};
}  // namespace std
