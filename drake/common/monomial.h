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

// Computes "n choose k", the number of ways, disregarding order, that k objects
// can be chosen from among n objects.
constexpr int NChooseK(int n, int k) {
  return (k == 0) ? 1 : (n * NChooseK(n - 1, k - 1)) / k;
}

/** Represents a monomial, a product of powers of variables with non-negative
 * integer exponents. Note that it does not include the coefficient part of a
 * monomial. Internally, it is represented by a map from a variable ID to its
 * integer exponent. */
class Monomial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Monomial)

  /** Constructs a monomial equal to 1. Namely the total degree is zero. */
  Monomial();

  /** Constructs a Monomial from @p powers. */
  explicit Monomial(const std::map<Variable::Id, int>& powers);

  /** Constructs a Monomial from @p var and @exponent. */
  Monomial(const Variable& var, int exponent);

  /**
   * Converts an expression to a monomial, if the expression is written as
   * ∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.
   * @pre{is_polynomial(e) should be true.}
   */
  explicit Monomial(const Expression& e);

  /** Returns the total degree of this Monomial. */
  int total_degree() const { return total_degree_; }

  /** Returns hash value. */
  size_t GetHash() const;

  /** Returns the internal representation of Monomial, the map from a base
   * (Variable ID) to its exponent (int).*/
  const std::map<Variable::Id, int>& get_powers() const { return powers_; }

  /** Evaluates under a given environment. */
  double Evaluate(const std::unordered_map<Variable::Id, double>& env) const;

  /** Substitutes using a given environment. */
  std::pair<double, Monomial> Substitute(
      const std::unordered_map<Variable::Id, double>& env) const;

  /** Returns a symbolic expression representing this monomial. Since, this
   * class only includes the ID of a variable, not a variable itself, we need
   * @id_to_var_map, a map from a variable ID to a variable as an argument of
   * this method to build an expression. */
  Expression ToExpression(
      const std::unordered_map<Variable::Id, Variable>& id_to_var_map) const;

  /** Checks if this monomial and @p m represent the same monomial.
   * Two monomials are equal iff they contain the same variable ID
   * raised to the same exponent. */
  bool operator==(const Monomial& m) const;

 private:
  // Computes the total degree of a monomial. This method is used in a
  // constructor of Monomial to set its total degree at construction.
  static int TotalDegree(const std::map<Variable::Id, int>& powers);

  int total_degree_{0};
  std::map<Variable::Id, int> powers_;
  friend std::ostream& operator<<(std::ostream& out, const Monomial& m);
};

std::ostream& operator<<(std::ostream& out, const Monomial& m);

/** Returns a multiplication of two monomials, m1 and m2. */
Monomial operator*(const Monomial& m1, const Monomial& m2);

/** Implements Graded reverse lexicographic order.
 *
 * @tparam VariableOrder VariableOrder{}(v1, v2) is true if v1 < v2.
 *
 * We first compare the total degree of the monomial; if there is a tie, then we
 * use the lexicographical order as the tie breaker, but a monomial with higher
 * order in lexicographical order is considered lower order in graded reverse
 * lexicographical order.
 *
 * Take the example MonomialBasis({x, y, z}, 2) as an example, with the order x
 * > y > z. To get the graded reverse lexicographical order, we take the
 * following steps:
 *
 * First find all the monomials using the total degree. The monomials with
 * degree 2 are {x^2, y^2, z^2, xy, xz, yz}. The monomials with degree 1 are {x,
 * y, z}, and the monomials with degree 0 is {1}. To break the tie between
 * monomials with the same total degree, first sort them in the reverse
 * lexicographical order, namely x < y < z in the reverse lexicographical
 * order. The lexicographical order compares two monomial by first comparing the
 * exponent of the largest variable, if there is a tie then go forth to the
 * second largest variable. Thus z^2 > zy >zx > y^2 > yx > x^2. Finally reverse
 * the order as x^2 > xy > y^2 > xz > yz > z^2.
 *
 * There is an introduction to monomial order in
 * https://en.wikipedia.org/wiki/Monomial_order, and an introduction to graded
 * reverse lexicographical order in
 * https://en.wikipedia.org/wiki/Monomial_order#Graded_reverse_lexicographic_order
 */
template <typename VariableOrder>
struct GradedReverseLexOrder {
  /** Returns true if m1 > m2 under the Graded reverse lexicographic order. */
  bool operator()(const Monomial& m1, const Monomial& m2) {
    const int d1{m1.total_degree()};
    const int d2{m2.total_degree()};
    if (d1 > d2) {
      return true;
    }
    if (d2 > d1) {
      return false;
    }
    // d1 == d2
    if (d1 == 0) {
      // Because both of them are 1.
      return false;
    }
    const std::map<Variable::Id, int>& powers1{m1.get_powers()};
    const std::map<Variable::Id, int>& powers2{m2.get_powers()};
    std::map<Variable::Id, int>::const_iterator it1{powers1.cbegin()};
    std::map<Variable::Id, int>::const_iterator it2{powers2.cbegin()};
    while (it1 != powers1.cend() && it2 != powers2.cend()) {
      const Variable::Id var1{it1->first};
      const Variable::Id var2{it2->first};
      const int exponent1{it1->second};
      const int exponent2{it2->second};
      if (variable_order_(var2, var1)) {
        return true;
      } else if (variable_order_(var1, var2)) {
        return false;
      } else {
        // var1 == var2
        if (exponent1 == exponent2) {
          ++it1;
          ++it2;
        } else {
          return exponent2 > exponent1;
        }
      }
    }
    // When m1 and m2 are identical.
    return false;
  }

 private:
  VariableOrder variable_order_;
};

/** Generates [b * m for m in MonomialBasis(vars, degree)] and push them to
 * bin. Used as a helper function to implement MonomialBasis.
 *
 * @tparam MonomialOrder provides a monomial ordering.
 */
template <typename MonomialOrder>
void AddMonomialsOfDegreeN(const Variables& vars, int degree, const Monomial& b,
                           std::set<Monomial, MonomialOrder>* const bin) {
  DRAKE_ASSERT(vars.size() > 0);
  if (degree == 0) {
    bin->insert(b);
    return;
  }
  const Variable& var{*vars.cbegin()};
  bin->insert(b * Monomial{var, degree});
  if (vars.size() == 1) {
    return;
  }
  for (int i{degree - 1}; i >= 0; --i) {
    AddMonomialsOfDegreeN(vars - var, degree - i, b * Monomial{var, i}, bin);
  }
  return;
}

/** Returns all monomials up to a given degree under the graded reverse
 * lexicographic order. This is called by MonomialBasis functions defined below
 * outside of internal namespace.
 *
 * @tparam rows Number of rows or Dynamic
 */
template <int rows>
Eigen::Matrix<Expression, rows, 1> ComputeMonomialBasis(const Variables& vars,
                                                        int degree) {
  DRAKE_DEMAND(vars.size() > 0);
  DRAKE_DEMAND(degree >= 0);
  // 1. Collect monomials.
  std::set<Monomial, GradedReverseLexOrder<std::less<Variable::Id>>> monomials;
  for (int i{degree}; i >= 0; --i) {
    AddMonomialsOfDegreeN(vars, i, Monomial{}, &monomials);
  }
  // 2. Build id_to_var_map (used in step 3).
  std::unordered_map<Variable::Id, Variable> id_to_var_map;
  for (const Variable& var : vars) {
    id_to_var_map.emplace(var.get_id(), var);
  }
  // 3. Prepare the return value, basis.
  DRAKE_DEMAND((rows == Eigen::Dynamic) ||
               (static_cast<size_t>(rows) == monomials.size()));
  Eigen::Matrix<Expression, rows, 1> basis(monomials.size());
  size_t i{0};
  for (const auto& m : monomials) {
    basis[i] = m.ToExpression(id_to_var_map);
    i++;
  }
  return basis;
}

/**
 * Returns the total degrees of the polynomial @p e w.r.t the variables in @p
 * vars. For example, the total degree of
 * e = x^2*y + 2 * x*y*z^3 + x * z^2
 * w.r.t (x, y) is 3 (from x^2 * y)
 * w.r.t (x, z) is 4 (from x*y*z^3)
 * w.r.t (z)    is 3 (from x*y*z^3)
 * Throws a runtime error if e.is_polynomial() is false.
 * @param vars A set of variables.
 * @return The total degree.
 */
int Degree(const Expression& e, const Variables& var);

/**
 * Returns the total degress of all the variables in the polynomial @p e.
 * For example, the total degree of
 * x^2*y + 2*x*y*z^3 + x*z^2
 * is 5, from x*y*z^3
 * Throws a runtime error is e.is_polynomial() is false.
 * @return The total degree.
 */
int Degree(const Expression& e);

/** Returns a monomial of the form x^2*y^3, it does not have the constant
 * factor. To generate a monomial x^2*y^3, @p map_var_to_exponent contains the
 * pair (x, 2) and (y, 3).
 *
 * \pre{All exponents in @p map_var_to_exponent are positive integers.}
 */
Expression GetMonomial(
    const std::unordered_map<Variable, int, hash_value<Variable>>&
        map_var_to_exponent);

/** Returns all monomials up to a given degree under the graded reverse
 * lexicographic order. Note that graded reverse lexicographic order uses the
 * total order among Variable which is based on a variable's unique ID. For
 * example, for a given variable ordering x > y > z, <tt>MonomialBasis({x, y,
 * z}, 2)</tt> returns a column vector <tt>[x^2, xy, y^2, xz, yz, z^2, x, y, z,
 * 1]</tt>.
 *
 * \pre{@p vars is a non-empty set.}
 * \pre{@p degree is a non-negative integer.}
 */
Eigen::Matrix<Expression, Eigen::Dynamic, 1> MonomialBasis(
    const Variables& vars, int degree);

/** Returns all monomials up to a given degree under the graded reverse
 * lexicographic order.
 *
 * @tparam n      number of variables
 * @tparam degree maximum total degree of monomials to compute
 *
 * \pre{@p vars is a non-empty set.}
 * \pre{<tt>vars.size()</tt> == @p.}
 */
template <int n, int degree>
Eigen::Matrix<Expression, NChooseK(n + degree, degree), 1> MonomialBasis(
    const Variables& vars) {
  static_assert(n > 0, "n should be a positive integer.");
  static_assert(degree >= 0, "degree should be a non-negative integer.");
  DRAKE_ASSERT(vars.size() == n);
  return ComputeMonomialBasis<NChooseK(n + degree, degree)>(vars, degree);
}

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
