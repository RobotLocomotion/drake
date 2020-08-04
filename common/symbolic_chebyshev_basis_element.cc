// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <cmath>
#include <vector>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
ChebyshevBasisElement::ChebyshevBasisElement(
    const std::map<Variable, int>& var_to_degree_map)
    : PolynomialBasisElement(var_to_degree_map) {}

bool ChebyshevBasisElement::operator<(
    const ChebyshevBasisElement& other) const {
  return this->lexicographical_compare(other);
}

double ChebyshevBasisElement::DoEvaluate(double variable_val,
                                         int degree) const {
  return EvaluateChebyshevPolynomial(variable_val, degree);
}

namespace {
// This function is used in operator*, it appends the pair (var, degree) to each
// entry of chebyshev_basis_all.
void AppendVariableAndDegree(
    const Variable& var, int degree,
    std::vector<std::map<Variable, int>>* chebyshev_basis_all) {
  for (auto& cheby_basis : *chebyshev_basis_all) {
    cheby_basis.emplace(var, degree);
  }
}

int power_of_2(int degree) {
  if (degree < 0) {
    throw std::invalid_argument("power of 2 underflow");
  }
  if (degree > static_cast<int>((sizeof(int) * CHAR_BIT - 2))) {
    throw std::invalid_argument("power of 2 overflow");
  }
  return 1 << degree;
}
}  // namespace

std::map<ChebyshevBasisElement, double> operator*(
    const ChebyshevBasisElement& a, const ChebyshevBasisElement& b) {
  // If variable x shows up in both ChebyshevBasisElement a and b, we know that
  // Tₘ(x) * Tₙ(x) = 0.5 (Tₘ₊ₙ(x) + Tₘ₋ₙ(x)), namely we will expand the product
  // to the sum of two new Chebyshev polynomials. Hence the number of terms in
  // the product of a * b is 2 to the power of # common variables showing up in
  // both a and b.

  // Number of variables that show up in both ChebyshevBasisElement a and b.
  // I first count the nummber of common variables, so as to do memory
  // allocation for the product result.
  int num_common_variables = 0;
  auto it_a = a.var_to_degree_map().begin();
  auto it_b = b.var_to_degree_map().begin();
  // Since the keys in var_to_degree_map are sorted, we can loop through
  // a.var_to_degree_map and b.var_to_degree_map jointly to find the common
  // variables.
  while (it_a != a.var_to_degree_map().end() &&
         it_b != b.var_to_degree_map().end()) {
    const Variable& var_a = it_a->first;
    const Variable& var_b = it_b->first;
    if (var_a.less(var_b)) {
      // var_a is less than var_b, and hence var_a less than all variables in b
      // after var_b. We can increment it_a.
      it_a++;
    } else if (var_b.less(var_a)) {
      it_b++;
    } else {
      num_common_variables++;
      it_a++;
      it_b++;
    }
  }
  // The number of ChebyshevBasisElement in the product result is
  // 2^num_common_variables.
  std::vector<std::map<Variable, int>> chebyshev_basis_all(
      power_of_2(num_common_variables));
  // I will go through the (variable, degree) pair of both a and b. If the
  // variable shows up in only a or b, then each term in the product a * b
  // contains that variable and its degree. If a has term Tₘ(x) and b has term
  // Tₙ(x), where x is the common variable, then half of the basis in a * b
  // contains term Tₘ₊ₙ(x), and the other half contains Tₘ₋ₙ(x).
  int common_variables_count = 0;
  it_a = a.var_to_degree_map().begin();
  it_b = b.var_to_degree_map().begin();

  // Note that var_to_degree_map() has its keys sorted in an increasing order.
  while (it_a != a.var_to_degree_map().end() &&
         it_b != b.var_to_degree_map().end()) {
    const Variable& var_a = it_a->first;
    const int degree_a = it_a->second;
    const Variable& var_b = it_b->first;
    const int degree_b = it_b->second;
    if (var_a.less(var_b)) {
      // var_a is not a common variable, add (var_a, degree_a) to each element
      // in chebyshev_basis_all.
      AppendVariableAndDegree(var_a, degree_a, &chebyshev_basis_all);
      it_a++;
    } else if (var_b.less(var_a)) {
      // var_b is not a common variable, add (var_b, degree_b) to each element
      // in chebyshev_basis_all.
      AppendVariableAndDegree(var_b, degree_b, &chebyshev_basis_all);
      it_b++;
    } else {
      // Add (var_common, degree_a + degree_b) to half of the elements in
      // chebyshev_basis_all, and (var_common, |degree_a - degree_b| to the
      // other half of the elements in chebyshev_basis_all.
      // If we denote 2 ^ common_variables_count = n, then we add (var_common,
      // degree_a + degree_b) to the chebyshev_basis_all[i], if i is in the
      // interval of [j * 2n, j * 2n + n) for some j, and we add (var_common,
      // |degree_a - degree_b|) to chebyshev_basis_all[i] if i is in the
      // interval [j * 2n + n, (j+1) * 2n).
      const int degree_sum = degree_a + degree_b;
      const int degree_diff = std::abs(degree_a - degree_b);
      const int n = power_of_2(common_variables_count);
      for (int j = 0;
           j < power_of_2(num_common_variables - common_variables_count - 1);
           ++j) {
        for (int i = j * 2 * n; i < j * 2 * n + n; ++i) {
          chebyshev_basis_all[i].emplace(var_a, degree_sum);
        }
        for (int i = j * 2 * n + n; i < (j + 1) * 2 * n; ++i) {
          chebyshev_basis_all[i].emplace(var_a, degree_diff);
        }
      }
      it_a++;
      it_b++;
      common_variables_count++;
    }
  }
  // Now either or both it_a or it_b is at the end of var_to_degree_map.end(),
  // if it_a != a.var_to_degree_map().end(), then the remaining variables after
  // it_a in a.var_to_degree_map cannot be a common variable. Append the rest of
  // the (variable, degree) to the Chebyshev basis.
  for (; it_a != a.var_to_degree_map().end(); ++it_a) {
    AppendVariableAndDegree(it_a->first, it_a->second, &chebyshev_basis_all);
  }
  for (; it_b != b.var_to_degree_map().end(); ++it_b) {
    AppendVariableAndDegree(it_b->first, it_b->second, &chebyshev_basis_all);
  }
  const double coeff = 1.0 / power_of_2(num_common_variables);
  std::map<ChebyshevBasisElement, double> result;
  for (const auto& var_to_degree_map : chebyshev_basis_all) {
    result.emplace(ChebyshevBasisElement(var_to_degree_map), coeff);
  }
  return result;
}
}  // namespace symbolic
}  // namespace drake
