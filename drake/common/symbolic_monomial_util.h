#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_monomial.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

/** Implements Graded reverse lexicographic order.
 *
 * @tparam VariableOrder VariableOrder{}(v1, v2) is true if v1 < v2.
 *
 * We first compare the total degree of the monomial; if there is a tie, then we
 * use the lexicographical order as the tie breaker, but a monomial with higher
 * order in lexicographical order is considered lower order in graded reverse
 * lexicographical order.
 *
 * Take MonomialBasis({x, y, z}, 2) as an example, with the order x > y > z. To
 * get the graded reverse lexicographical order, we take the following steps:
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
    const std::map<Variable, int>& powers1{m1.get_powers()};
    const std::map<Variable, int>& powers2{m2.get_powers()};
    std::map<Variable, int>::const_iterator it1{powers1.cbegin()};
    std::map<Variable, int>::const_iterator it2{powers2.cbegin()};
    while (it1 != powers1.cend() && it2 != powers2.cend()) {
      const Variable& var1{it1->first};
      const Variable& var2{it2->first};
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

namespace internal {
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
 * lexicographic order. This is called by MonomialBasis functions defined below.
 *
 * @tparam rows Number of rows or Dynamic
 */
template <int rows>
Eigen::Matrix<Monomial, rows, 1> ComputeMonomialBasis(const Variables& vars,
                                                      int degree) {
  DRAKE_DEMAND(vars.size() > 0);
  DRAKE_DEMAND(degree >= 0);
  // 1. Collect monomials.
  std::set<Monomial, GradedReverseLexOrder<std::less<Variable>>> monomials;
  for (int i{degree}; i >= 0; --i) {
    AddMonomialsOfDegreeN(vars, i, Monomial{}, &monomials);
  }
  // 2. Prepare the return value, basis.
  DRAKE_DEMAND((rows == Eigen::Dynamic) ||
               (static_cast<size_t>(rows) == monomials.size()));
  Eigen::Matrix<Monomial, rows, 1> basis(monomials.size());
  size_t i{0};
  for (const auto& m : monomials) {
    basis[i] = m;
    i++;
  }
  return basis;
}
}  // namespace internal

/** Returns all monomials up to a given degree under the graded reverse
 * lexicographic order. Note that graded reverse lexicographic order uses the
 * total order among Variable which is based on a variable's unique ID. For
 * example, for a given variable ordering x > y > z, `MonomialBasis({x, y, z},
 * 2)` returns a column vector `[x^2, xy, y^2, xz, yz, z^2, x, y, z, 1]`.
 *
 * @pre @p vars is a non-empty set.
 * @pre @p degree is a non-negative integer.
 */
Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         int degree);

// Computes "n choose k", the number of ways, disregarding order, that k objects
// can be chosen from among n objects. It is used in the following MonomialBasis
// function.
constexpr int NChooseK(int n, int k) {
  return (k == 0) ? 1 : (n * NChooseK(n - 1, k - 1)) / k;
}

/** Returns all monomials up to a given degree under the graded reverse
 * lexicographic order.
 *
 * @tparam n      number of variables.
 * @tparam degree maximum total degree of monomials to compute.
 *
 * @pre @p vars is a non-empty set.
 * @pre vars.size() == @p n.
 */
template <int n, int degree>
Eigen::Matrix<Monomial, NChooseK(n + degree, degree), 1> MonomialBasis(
    const Variables& vars) {
  static_assert(n > 0, "n should be a positive integer.");
  static_assert(degree >= 0, "degree should be a non-negative integer.");
  DRAKE_ASSERT(vars.size() == n);
  return internal::ComputeMonomialBasis<NChooseK(n + degree, degree)>(vars,
                                                                      degree);
}

}  // namespace symbolic
}  // namespace drake
