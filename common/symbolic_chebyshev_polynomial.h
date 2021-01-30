#pragma once
#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

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
 * One definition of Chebyshev polynomial of the first kind is
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
   * Constructs a Chebyshev polynomial Tₙ(x)
   * @param var The variable x
   * @param degree The Chebyshev polynomial is of degree n.
   * @pre degree >= 0.
   */
  ChebyshevPolynomial(Variable var, int degree);

  /** Getter for the variable. */
  [[nodiscard]] const Variable& var() const { return var_; }

  /** Getter for the degree of the Chebyshev polynomial. */
  [[nodiscard]] int degree() const { return degree_; }

  /**
   * Converts this Chebyshev polynomial to a polynomial with monomial basis.
   */
  [[nodiscard]] Polynomial ToPolynomial() const;

  /**
   * Evaluates this Chebyshev polynomial at @p var_val.
   */
  [[nodiscard]] double Evaluate(double var_val) const;

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
   * Compare this to another Chebyshev polynomial, returns True if this is
   * regarded as less than the other, otherwise returns false.
   *
   *    If this.var() < other.var(), return True.
   *    If this.var() > other.var(), return False.
   *    If this.var() == other.var(), then return this.degree() <
   * other.degree().
   *
   * A special case is when this.degree() == 0 or other.degree()
   * == 0. In this case the variable doesn't matter, and we return this.degree()
   * < other.degree().
   */
  bool operator<(const ChebyshevPolynomial& other) const;

  /**
   * Computes the differentiation of a Chebyshev polynomial
   * dTₙ(x)/dx = nUₙ₋₁(x)
   * where Uₙ₋₁(x) is a Chebyshev polynomial of the second kind. Uₙ₋₁(x) can
   * be written as a summation of Chebyshev polynomials of the first kind
   * with lower degrees.
   * - If n is even dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x), j is odd  and j <= n-1
   * - If n is odd dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x) - n, j is even and j <= n-1
   * - A special case is that dT₀(x)/dx = 0.
   * @retval chebyshev_coeff_pairs. sum(chebyshev_coeff_pairs[j].first *
   * chebyshev_coeff_pairs[j].second) is the differentiation dTₙ(x)/dx. If n is
   * even, then chebyshev_coeff_pairs[j] = (T₂ⱼ₋₁(x), 2n). If n is odd, then
   * chebyshev_coeff_pairs[j] = (T₂ⱼ(x), 2n) for j >= 1, and
   * chebyshev_coeff_pairs[0] = (T₀(x), n).
   * For the special case when degree() == 0, we return an empty vector.
   */
  [[nodiscard]] std::vector<std::pair<ChebyshevPolynomial, double>>
  Differentiate() const;

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
  Variable var_{};
  int degree_{};
};

std::ostream& operator<<(std::ostream& out, const ChebyshevPolynomial& p);

/**
 * Evaluates a Chebyshev polynomial at a given value.
 * @param var_val The value of the variable.
 * @param degree The degree of the Chebyshev polynomial.
 */
double EvaluateChebyshevPolynomial(double var_val, int degree);
}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::ChebyshevPolynomial>. */
template <>
struct hash<drake::symbolic::ChebyshevPolynomial> : public drake::DefaultHash {
};
}  // namespace std
