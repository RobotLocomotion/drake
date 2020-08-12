#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <functional>
#include <map>
#include <set>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace internal {
/* Adds the set {RaisePower(b, m) for m in Basis(vars, degree)} to `bin`.
 `Basis` is the polynomial basis associated with `BasisElement` and
 `Basis(vars, degree)` returns all BasisElement objects with variables `vars`
 and total degree = `degree`. RaisePower(m, b) is the operation that "merges"
 two `BasisElement`s such that the merged `BasisElement`:

    - Is a product of all variables present in either m or b.
    - The power of variable v is the sum of the degree of that variable in m and
      b. If a variable is missing in a BasisElement, it has degree 0.

 As a concrete example, assume that BasisElement is actually
 MonomialBasisElement with
 vars = {x, y}, degree = 2, and b = x²yz³. MonomialBasis({x, y}, 2) would
 produce the set of MonomialBasisElements: x², xy, y². And applying
 RaisePower(b, m) to each m would produce:

  - RaisePower(x²yz³, x²) --> x⁴yz³
  - RaisePower(x²yz³, xy) --> x³y²z³
  - RaisePower(x²yz³, y²) --> x²y³z³

 @tparam BasisElementOrder a total order of PolynomialBasisElement. An example
         is BasisElementGradedReverseLexOrder.
 @tparam BasisElement A derived class of PolynomialBasisElement.
 @pre vars must contain at least one variable.
 */
template <typename BasisElementOrder, typename BasisElement>
void AddPolynomialBasisElementsOfDegreeN(
    const Variables& vars, int degree, const BasisElement& b,
    std::set<BasisElement, BasisElementOrder>* const bin) {
  // To explain this algorithm, here I use the running example that BasisElement
  // is MonomialBasisElement, vars = {x, y, z}, degree = 3, b = x²zw. We want to
  // add the monomials of the form x²zw * xⁱyʲzᵏ, i+j+k=degree=3 to @p bin. We
  // do that by first taking var = vars.cbegin() = x, and add the monomial b
  // * power(var, degree) = x²zw * x³ = x⁵zw to bin. Then we consider to add the
  // case when i < degree, namely i = 0, 1, 2. In each case, this is equivalent
  // to adding the monomials of the form x²⁺ⁱzw * yʲzᵏ to bin, where
  // j+k=degree-i, namely we can set new_b = x²⁺ⁱzw, and call the function
  // AddPolynomialBasisElementOfDegreeN({y, z}, degree-m, new_b, bin) to add the
  // monomials x²⁺ⁱz * yʲzᵏw to @p bin.
  static_assert(
      std::is_base_of<PolynomialBasisElement, BasisElement>::value ||
          std::is_same<BasisElement, Monomial>::value,
      "BasisElement should be a derived class of PolynomialBasisElement");
  DRAKE_ASSERT(vars.size() > 0);
  if (degree == 0) {
    bin->insert(b);
    return;
  }
  const Variable& var{*vars.cbegin()};
  // Compute RaisePower(b, ElementBasis(var, degree)).
  std::map<Variable, int> new_var_to_degree_map = b.get_powers();
  auto it = new_var_to_degree_map.find(var);
  if (it != new_var_to_degree_map.end()) {
    it->second += degree;
  } else {
    new_var_to_degree_map.emplace_hint(it, var, degree);
  }
  bin->insert(BasisElement(new_var_to_degree_map));
  if (vars.size() == 1) {
    return;
  }

  for (int i{degree - 1}; i >= 0; --i) {
    // Compute RaisePower(b, ElementBasis(var, i))
    std::map<Variable, int> new_b_var_to_degree_map = b.get_powers();
    auto it2 = new_b_var_to_degree_map.find(var);
    if (it2 != new_b_var_to_degree_map.end()) {
      it2->second += i;
    } else {
      new_b_var_to_degree_map.emplace_hint(it2, var, i);
    }
    const BasisElement new_b(new_b_var_to_degree_map);
    AddPolynomialBasisElementsOfDegreeN(vars - var, degree - i, new_b, bin);
  }
}
}  // namespace internal

/**
 * Returns all polynomial basis elements up to a given degree under the graded
 * reverse lexicographic order.
 * @tparam rows Number of rows or Eigen::Dynamic.
 * @tparam BasisElement A derived class of PolynomialBasisElement.
 * @param vars The variables appearing in the polynomial basis.
 * @param degree The highest total degree of the polynomial basis elements.
 * @param degree_type If degree_type is kAny, then the polynomial basis
 * elements' degrees are no larger than @p degree. If degree_type is kEven, then
 * the elements' degrees are even numbers no larger than @p degree. If
 * degree_type is kOdd, then the elements' degrees are odd numbers no larger
 * than @p degree.
 * TODO(hongkai.dai): this will replace ComputeMonomialBasis in
 * symbolic_monomial_util.h
 */
template <int rows, typename BasisElement>
Eigen::Matrix<BasisElement, rows, 1> ComputePolynomialBasisUpToDegree(
    const Variables& vars, int degree, internal::DegreeType degree_type) {
  DRAKE_DEMAND(vars.size() > 0);
  DRAKE_DEMAND(degree >= 0);
  // 1. Collect elements.
  std::set<BasisElement,
           BasisElementGradedReverseLexOrder<std::less<Variable>, BasisElement>>
      basis_elements_set;
  int start_degree = 0;
  int degree_stride = 1;
  switch (degree_type) {
    case internal::DegreeType::kAny: {
      start_degree = 0;
      degree_stride = 1;
      break;
    }
    case internal::DegreeType::kEven: {
      start_degree = 0;
      degree_stride = 2;
      break;
    }
    case internal::DegreeType::kOdd: {
      start_degree = 1;
      degree_stride = 2;
    }
  }
  for (int i = start_degree; i <= degree; i += degree_stride) {
    internal::AddPolynomialBasisElementsOfDegreeN(vars, i, BasisElement{},
                                                  &basis_elements_set);
  }
  // 2. Prepare the return value, basis.
  DRAKE_DEMAND((rows == Eigen::Dynamic) ||
               (static_cast<size_t>(rows) == basis_elements_set.size()));
  Eigen::Matrix<BasisElement, rows, 1> basis(basis_elements_set.size());
  int i{0};
  for (const auto& m : basis_elements_set) {
    basis[i] = m;
    i++;
  }
  return basis;
}

}  // namespace symbolic
}  // namespace drake
