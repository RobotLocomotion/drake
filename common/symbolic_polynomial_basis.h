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
/**
 * Generates [RaisePower(b, m) for m in PolynomialBasis(vars, degree)], where
 * PolynomialBasis(vars, degree) returns all the PolynomialBasisElement objects
 * with variables `vars` and total degree = `degree`. RaisePower is the
 * operation that merges the two map b.var_to_degree_map() and
 * m.var_to_degree_map(), When there is a shared key `v` in both maps
 * `b` and `m`, then the returned PolynomialBasisElement contains `v` to degree
 * `b.var_to_degree_map().at(v)+m.var_to_degree_map().at(v)`. For example, if
 * both b and m are MonomialBasisElements, b = x²y, m=xy²z, then RaisePower(b,
 * m) = x³y³z.
 * TODO(hongkai.dai): this will replace AddMonomialsOfDegreeN in
 * symbolic_monomial_util.h
 * @tparam BasisElementOrder a total order of PolynomialBasisElement. An example
 * is BasisElementGradedReverseLexOrder.
 * @tparam BasisElement A derived class of PolynomialBasisElement.
 */
template <typename BasisElementOrder, typename BasisElement>
void AddPolynomialBasisElementsOfDegreeN(
    const Variables& vars, int degree, const BasisElement& b,
    std::set<BasisElement, BasisElementOrder>* const bin) {
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
 * @param vars The variables appeared in the polynomial basis.
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
  size_t i{0};
  for (const auto& m : basis_elements_set) {
    basis[i] = m;
    i++;
  }
  return basis;
}

}  // namespace symbolic
}  // namespace drake
