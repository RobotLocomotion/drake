#include "drake/systems/trajectory_optimization/integration_constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

MidPointIntegrationConstraint::MidPointIntegrationConstraint(int dim)
    : solvers::Constraint(dim, 4 * dim + 1, Eigen::VectorXd::Zero(dim),
                          Eigen::VectorXd::Zero(dim),
                          "midpoint_integration_constraint"),
      dim_{dim} {}

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
