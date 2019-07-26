#include "drake/systems/trajectory_optimization/integration_constraint.h"

#include <utility>
#include <vector>

namespace drake {
namespace systems {
namespace trajectory_optimization {

MidPointIntegrationConstraint::MidPointIntegrationConstraint(int dim)
    : solvers::Constraint(dim, 4 * dim + 1, Eigen::VectorXd::Zero(dim),
                          Eigen::VectorXd::Zero(dim),
                          "midpoint_integration_constraint"),
      dim_{dim} {
  // Set the sparsity pattern of the constraint gradient. The i'th row of the
  // constraint only depends on variable x_r(i), x_l(i), xdot_r(i), xdot_l(i)
  // and dt.
  std::vector<std::pair<int, int>> gradient_sparsity_pattern;
  gradient_sparsity_pattern.reserve(5 * dim);
  for (int i = 0; i < dim_; ++i) {
    gradient_sparsity_pattern.emplace_back(i, i);            // x_r(i)
    gradient_sparsity_pattern.emplace_back(i, i + dim);      // x_l(i)
    gradient_sparsity_pattern.emplace_back(i, i + 2 * dim);  // xdot_r(i)
    gradient_sparsity_pattern.emplace_back(i, i + 3 * dim);  // xdot_r(i)
    gradient_sparsity_pattern.emplace_back(i, 4 * dim);      // dt
  }
  SetGradientSparsityPattern(gradient_sparsity_pattern);
}

template <typename T>
void MidPointIntegrationConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  VectorX<T> x_r, x_l, xdot_r, xdot_l;
  T dt;
  DecomposeX<T>(x, &x_r, &x_l, &xdot_r, &xdot_l, &dt);
  *y = x_r - x_l - dt / 2 * (xdot_r + xdot_l);
}

void MidPointIntegrationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void MidPointIntegrationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y);
}

void MidPointIntegrationConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
