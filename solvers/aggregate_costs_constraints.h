#pragma once
#include <vector>

#include "drake/solvers/binding.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace solvers {
/** Given many linear costs, aggregate them into
 *
 *     aᵀ*x + b,
 * @param linear_costs the linear costs to be aggregated.
 * @param linear_coeff[out] a in the documentation above.
 * @param vars[out] x in the documentation above.
 * @param constant_cost[out] b in the documentation above.
 */
void AggregateLinearCosts(const std::vector<Binding<LinearCost>>& linear_costs,
                          Eigen::SparseVector<double>* linear_coeff,
                          VectorX<symbolic::Variable>* vars,
                          double* constant_cost);

/** Given many linear and quadratic costs, aggregate them into
 *
 *     0.5*x₁ᵀQx₁ + bᵀx₂ + c
 * where x₁ and x₂ don't need to be the same.
 * @param quadratic_costs The quadratic costs to be aggregated.
 * @param linear_costs The linear costs to be aggregated.
 * @param Q_lower[out] The lower triangular part of the matrix Q.
 * @param quadratic_vars[out] x₁ in the documentation above.
 * @param linear_coeff[out] b in the documentation above.
 * @param linear_vars[out] x₂ in the documentation above.
 * @param constant_cost[out] c in the documentation above.
 */
void AggregateQuadraticAndLinearCosts(
    const std::vector<Binding<QuadraticCost>>& quadratic_costs,
    const std::vector<Binding<LinearCost>>& linear_costs,
    Eigen::SparseMatrix<double>* Q_lower,
    VectorX<symbolic::Variable>* quadratic_vars,
    Eigen::SparseVector<double>* linear_coeff,
    VectorX<symbolic::Variable>* linear_vars, double* constant_cost);
}  // namespace solvers
}  // namespace drake
