#include "drake/common/symbolic/monomial_util.h"

#include <algorithm>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace symbolic {
Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         const int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(
      vars, degree, internal::DegreeType::kAny);
}

Eigen::Matrix<Monomial, Eigen::Dynamic, 1> EvenDegreeMonomialBasis(
    const Variables& vars, int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(
      vars, degree, internal::DegreeType::kEven);
}

Eigen::Matrix<Monomial, Eigen::Dynamic, 1> OddDegreeMonomialBasis(
    const Variables& vars, int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(
      vars, degree, internal::DegreeType::kOdd);
}

namespace {
// Generates all the monomials with degree up to 1 for each variable, for all
// the variables in x[start], x[start+1], ..., x.back(). Append these monomials
// to `monomial_basis`.
// @param[in/out] monomial_basis We assume that as the input, monomial_basis
// doesn't contain x[start] in its variables. As an output, the new monomials
// that contain x[start] will be appended to monomial_basis.
void CalcMonomialBasisOrderUpToOneHelper(
    const std::vector<Variable>& x, int start,
    std::vector<Monomial>* monomial_basis) {
  DRAKE_DEMAND(start >= 0 && start < static_cast<int>(x.size()));
  if (start == static_cast<int>(x.size() - 1)) {
    // Generate the monomial for a single variable, which is 1 and the variable
    // itself.
    monomial_basis->emplace_back();
    monomial_basis->emplace_back(x[start], 1);
    return;
  } else {
    // First generate the monomials using all the variables x[start + 1] ...
    // x[x.size() - 1]
    CalcMonomialBasisOrderUpToOneHelper(x, start + 1, monomial_basis);
    const int monomial_count = monomial_basis->size();
    // Construct the monomial x[start].
    const Monomial x_start(x[start], 1);
    // Multiply x_start to each monomial already in monomial_set, and append the
    // multiplication result to monomial_set.
    for (int i = 0; i < monomial_count; ++i) {
      monomial_basis->push_back(x_start * (*monomial_basis)[i]);
    }
    return;
  }
}
}  // namespace

VectorX<Monomial> CalcMonomialBasisOrderUpToOne(const Variables& x,
                                                bool sort_monomial) {
  std::vector<Variable> x_vec;
  x_vec.reserve(x.size());
  for (const auto& var : x) {
    x_vec.push_back(var);
  }
  std::vector<Monomial> monomial_basis;
  // There will be pow(2, x.size()) monomials in the basis.
  monomial_basis.reserve(1 << static_cast<int>(x.size()));
  CalcMonomialBasisOrderUpToOneHelper(x_vec, 0, &monomial_basis);
  VectorX<Monomial> ret(monomial_basis.size());
  if (sort_monomial) {
    std::sort(monomial_basis.begin(), monomial_basis.end(),
              GradedReverseLexOrder<std::less<Variable>>());
  }
  for (int i = 0; i < ret.rows(); ++i) {
    ret(i) = monomial_basis[i];
  }
  return ret;
}
}  // namespace symbolic
}  // namespace drake
