#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <ostream>
#include <set>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {

namespace symbolic {

/** Represents a monomial, a product of powers of variables with non-negative
 * integer exponents. Note that it does not include the coefficient part of a
 * monomial.
 */
class Monomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Monomial)

  /** Constructs a monomial equal to 1. Namely the total degree is zero. */
  Monomial() = default;

  /** Constructs a Monomial from @p powers. */
  explicit Monomial(const std::map<Variable, int>& powers);

  /**
   * Converts an expression to a monomial if the expression is written as
   * ∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.
   * @pre is_polynomial(e) should be true.
   */
  explicit Monomial(const Expression& e);

  /** Constructs a Monomial from @p var. */
  explicit Monomial(const Variable& var);

  /** Constructs a Monomial from @p var and @exponent. */
  Monomial(const Variable& var, int exponent);

  /** Returns the total degree of this Monomial. */
  int total_degree() const { return total_degree_; }

  /** Returns hash value. */
  size_t GetHash() const;

  /** Returns the set of variables in this monomial. */
  Variables GetVariables() const;

  /** Returns the internal representation of Monomial, the map from a base
   * (Variable) to its exponent (int).*/
  const std::map<Variable, int>& get_powers() const { return powers_; }

  /** Evaluates under a given environment @p env.
   *
   * @throws std::out_of_range exception if there is a variable in this monomial
   * whose assignment is not provided by @p env.
   */
  double Evaluate(const Environment& env) const;

  /** Substitutes using a given environment @p env. The substitution result is
   * of type pair<double, Monomial>. The first component (: double) represents
   * the coefficient part while the second component represents the remaining
   * parts of the Monomial which was not substituted. Note that users are
   * allowed to provide a partial environment.
   *
   * Example 1. Substitution with a fully-specified environment
   *     (x^3*y^2).Substitute({{x, 2}, {y, 3}})
   *   = (2^3 * 3^2 = 8 * 9 = 72, Monomial{} = 1).
   *
   * Example 1. Substitution with a partial environment
   *     (x^3*y^2).Substitute({{x, 2}})
   *   = (2^3 = 8, y^2).
   */
  std::pair<double, Monomial> Substitute(const Environment& env) const;

  /** Returns a symbolic expression representing this monomial. */
  Expression ToExpression() const;

  /** Checks if this monomial and @p m represent the same monomial. Two
   * monomials are equal iff they contain the same variable raised to the same
   * exponent. */
  bool operator==(const Monomial& m) const;

  /** Checks if this monomial and @p m do not represent the same monomial. */
  bool operator!=(const Monomial& m) const;

  /** Returns this monomial multiplied by @p m. */
  Monomial& operator*=(const Monomial& m);

  /** Returns this monomial raised to @p p.
   * @throws std::runtime_error if @p p is negative.
   */
  Monomial& pow_in_place(int p);

 private:
  int total_degree_{0};
  std::map<Variable, int> powers_;
  friend std::ostream& operator<<(std::ostream& out, const Monomial& m);
};

std::ostream& operator<<(std::ostream& out, const Monomial& m);

/** Returns a multiplication of two monomials, @p m1 and @p m2. */
Monomial operator*(Monomial m1, const Monomial& m2);

/** Returns @p m1 raised to @p p.
 * @throws std::runtime_error if @p p is negative.
 */
Monomial pow(Monomial m, int p);

/** Returns a monomial of the form x^2*y^3, it does not have the constant
 * factor. To generate a monomial x^2*y^3, @p map_var_to_exponent contains the
 * pair (x, 2) and (y, 3).
 *
 * @pre All exponents in @p map_var_to_exponent are positive integers.
 */
Expression GetMonomial(
    const std::unordered_map<Variable, int, hash_value<Variable>>&
        map_var_to_exponent);

typedef std::unordered_map<Expression, Expression, hash_value<Expression>>
    MonomialAsExpressionToCoefficientMap;
/**
 * Decomposes a polynomial `e` into monomials, with respect to a specified set
 * of variables `vars`.
 * A polynomial can be represented as
 * ∑ᵢ c(i) * m(i)
 * where m(i) is a monomial in the specified set of variables, and c(i) is the
 * corresponding coefficient.
 * Note the coefficient will include any constants and symbols not in the set of
 * variables.
 * <pre>
 * Example:
 * For polynomial e1 = 2x²y + 3xy²z + 4z
 * Decompose(e1, {x,y,z}) will return the map
 * map[x²y] = 2
 * map[xy²z] = 3
 * map[z] = 4
 * on the other hand, Decompose(e1, {x,y}) (notice z is not included in the
 * input argument) will return the map
 * map[x²y] = 2
 * map[xy²] = 3z
 * map[1] = 4z
 * </pre>
 * @pre{e.is_polynomial() returns true}
 * @param e The polynomial to be decomposed. Throw a runtime error if `e` is not
 * a polynomial.
 * @param vars The variables whose monomials will be considered in the
 * decomposition.
 * @retval monomial_to_coeff_map Map the monomial to the coefficient in each
 * term of the polynomial.
 */
MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
    const Expression& e, const Variables& vars);

/**
 * Decomposes a polynomial as the summation of coefficients multiply monomials,
 * w.r.t all variables in the polynomial.
 * For polynomial e1 = 2x²y + 3xy²z + 4z
 * Decompose(e1, {x,y,z}) will return the map
 * map[x²y] = 2
 * map[xy²z] = 3
 * map[z] = 4
 * @param e A polynomial. Throws a runtime error if `e` is not a polynomial.
 * @pre{e.is_polynomial() returns true.}
 * @return map. The key of the map is the monomial, with the value being the
 * coefficient.
 */
MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
    const Expression& e);
}  // namespace symbolic

/** Computes the hash value of a Monomial. */
template <>
struct hash_value<symbolic::Monomial> {
  size_t operator()(const symbolic::Monomial& m) const { return m.GetHash(); }
};

namespace symbolic {
/**
 * Maps a monomial to a coefficient. This map can be used to represent a
 * polynomial, such that the polynomial is
 *   ∑ map[key] * key
 * Compared to MonomialAsExpressionToCoefficientMap, using Monomial as the key
 * type should be faster than using the Expression as the key type.
 */
typedef std::unordered_map<Monomial, Expression, hash_value<Monomial>>
    MonomialToCoefficientMap;

/**
 * Decomposes a polynomial into monomial and its coefficient. Throws a runtime
 * error if the expression is not a polynomial.
 * @see DecomposePolynomialIntoExpression();
 * Using MonomialToCoefficientMap is faster and more specific than using
 * MonomialAsExpressionToCoefficientMap, so prefer
 * DecomposePolynomialIntoMonomial to DecomposePolynomialIntoExpression when
 * speed is a concern.
 */
MonomialToCoefficientMap DecomposePolynomialIntoMonomial(const Expression& e,
                                                         const Variables& vars);

}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {
// Eigen scalar type traits for Matrix<drake::symbolic::Monomial>.
template <>
struct NumTraits<drake::symbolic::Monomial>
    : GenericNumTraits<drake::symbolic::Monomial> {
  static inline int digits10() { return 0; }
};

}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
