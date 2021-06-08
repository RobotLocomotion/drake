#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
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
 @note A non-zero degree has no effect if vars is empty. If vars is empty
 and degree is zero, then {RaisePower(b, m) for m in Basis(vars, degree)} = {b}.
 */
template <typename BasisElementOrder, typename BasisElement>
void AddPolynomialBasisElementsOfDegreeN(
    const Variables& vars, int degree, const BasisElement& b,
    std::set<BasisElement, BasisElementOrder>* const bin) {
  /* Iterating through Basis(vars, degree) is tricky due to the fact that the
   number of variables is determined at runtime. For example, assume the
   MonomialBasisElement with vars = {x, y, z} and degree = 3, then
   Basis({x, y, z}, 3) would produce the basis elements: x³, x²y, xy², xyz, y³,
   y²z, yz², z³. Conceptually, this matches well to nested for loops. Each
   for loop "consumes" a portion of the target degree, leaving the remaining
   part to the nested loops.

   for (int p_x = degree; p_x >= 0; --p_x) {
     for (int p_y = degree - p_x; p_y >= 0; --p_y) {
       for (int p_z = degree - p_x - p_y; p_z >= 0; --p_z) {
         Monomial m = x ** p_x * y ** p_y * z ** p_z;  // Ok, not real code.
         bin->insert(RaisePower(m, b));
       }
     }
   }

   However, if we add another variable, we have to add another nested for loop.
   We can't do that dynamically. So, we use recursion to create the *effect*
   of a dynamically-determined number of nested for loops. Each level of the
   recursion selects the "first" variable in the set and iteratively consumes
   some portion of the target degree. The recursive call is made on the
   remaining variables with the remaining available degree. The consumed portion
   is preserved by pre-multiplying it with the `b` value and passing on this
   accumulated b to the recursive call. RaisePower(m, b) is simply m * b. We
   can refactor the product: RaisePower(m, b) = RaisePower(m/xⁱ, b * xⁱ) and
   this relationship is the recursive call. */
  static_assert(
      std::is_base_of_v<PolynomialBasisElement, BasisElement> ||
          std::is_same_v<BasisElement, Monomial>,
      "BasisElement should be a derived class of PolynomialBasisElement");
  if (degree == 0) {
    bin->insert(b);
    return;
  }
  if (vars.empty()) {
    return;
  }
  const Variable& var{*vars.cbegin()};
  for (int var_degree = degree; var_degree >= 0; --var_degree) {
    std::map<Variable, int> new_var_to_degree_map = b.get_powers();
    auto it = new_var_to_degree_map.find(var);
    if (it != new_var_to_degree_map.end()) {
      it->second += var_degree;
    } else {
      new_var_to_degree_map.emplace_hint(it, var, var_degree);
    }
    const BasisElement new_b(new_var_to_degree_map);
    AddPolynomialBasisElementsOfDegreeN(vars - var, degree - var_degree, new_b,
                                        bin);
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
  DRAKE_DEMAND(!vars.empty());
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
