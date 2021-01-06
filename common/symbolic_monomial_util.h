#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

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
  bool operator()(const Monomial& m1, const Monomial& m2) const {
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
 * TODO(hongkai.dai): Remove this method and use
 * AddPolynomialBasisElementsOfDegreeN in symbolic_polynomial_basis.h instead
 * when we deprecate Monomial class.
 */
template <typename MonomialOrder>
void AddMonomialsOfDegreeN(const Variables& vars, int degree, const Monomial& b,
                           std::set<Monomial, MonomialOrder>* const bin) {
  DRAKE_ASSERT(!vars.empty());
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
}

enum class DegreeType {
  kEven,  ///< Even degree
  kOdd,   ///< Odd degree
  kAny,   ///< Any degree
};

/** Returns all monomials up to a given degree under the graded reverse
 * lexicographic order. This is called by MonomialBasis functions defined below.
 *
 * @tparam rows Number of rows or Dynamic
 * @param degree_type If degree_type is kAny, then the monomials' degrees are no
 * larger than @p degree. If degree_type is kEven, then the monomial's degrees
 * are even numbers no larger than @p degree. If degree_type is kOdd, then the
 * monomial degrees are odd numbers no larger than @p degree.
 */
template <int rows>
Eigen::Matrix<Monomial, rows, 1> ComputeMonomialBasis(
    const Variables& vars, int degree,
    DegreeType degree_type = DegreeType::kAny) {
  DRAKE_DEMAND(!vars.empty());
  DRAKE_DEMAND(degree >= 0);
  // 1. Collect monomials.
  std::set<Monomial, GradedReverseLexOrder<std::less<Variable>>> monomials;
  int start_degree = 0;
  int degree_stride = 1;
  switch (degree_type) {
    case DegreeType::kAny: {
      start_degree = 0;
      degree_stride = 1;
      break;
    }
    case DegreeType::kEven: {
      start_degree = 0;
      degree_stride = 2;
      break;
    }
    case DegreeType::kOdd: {
      start_degree = 1;
      degree_stride = 2;
    }
  }
  for (int i = start_degree; i <= degree; i += degree_stride) {
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

/** Returns all even degree monomials up to a given degree under the graded
 * reverse lexicographic order. A monomial has an even degree if its total
 * degree is even. So xy is an even degree monomial (degree 2) while x²y is not
 * (degree 3). Note that graded reverse lexicographic order uses the total order
 * among Variable which is based on a variable's unique ID. For example, for a
 * given variable ordering x > y > z, `EvenDegreeMonomialBasis({x, y, z}, 2)`
 * returns a column vector `[x², xy, y², xz, yz, z², 1]`.
 *
 * @pre @p vars is a non-empty set.
 * @pre @p degree is a non-negative integer.
 */
Eigen::Matrix<Monomial, Eigen::Dynamic, 1> EvenDegreeMonomialBasis(
    const Variables& vars, int degree);

/** Returns all odd degree monomials up to a given degree under the graded
 * reverse lexicographic order. A monomial has an odd degree if its total
 * degree is odd. So x²y is an odd degree monomial (degree 3) while xy is not
 * (degree 2). Note that graded reverse lexicographic order uses the total order
 * among Variable which is based on a variable's unique ID. For example, for a
 * given variable ordering x > y > z, `OddDegreeMonomialBasis({x, y, z}, 3)`
 * returns a column vector `[x³, x²y, xy², y³, x²z, xyz, y²z, xz², yz², z³, x,
 * y, z]`
 *
 * @pre @p vars is a non-empty set.
 * @pre @p degree is a non-negative integer.
 */
Eigen::Matrix<Monomial, Eigen::Dynamic, 1> OddDegreeMonomialBasis(
    const Variables& vars, int degree);

}  // namespace symbolic
}  // namespace drake
