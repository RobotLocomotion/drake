#pragma once
#include <vector>

#include "drake/solvers/binding.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace solvers {
namespace internal {
// Given many quadratic costs, aggregate them into
//
//     0.5 xᵀQx + bᵀ*x + c,
// @param quadratic_costs the quadratic costs to be aggregated.
// @param Q_lower[out] contains the lower_triangular matrix of Q.
// @param linear_coeff[out] b in the documentation above.
// @param vars[out] x in the documentation above.
// @param constant_cost[out] c in the documentation above.
void AggregateQuadraticCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* vars, double* constant_cost);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
