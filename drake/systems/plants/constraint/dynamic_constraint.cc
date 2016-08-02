
#include "dynamic_constraint.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace systems {
namespace {
Eigen::MatrixXd ExtractDerivativesMatrix(const drake::TaylorVecXd& vec_in) {
  if (!vec_in.size()) {
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd ret(vec_in.size(), vec_in(0).derivatives().size());
  for (int i = 0; i < ret.rows(); i++) {
    ret.row(i) = vec_in(i).derivatives();
  }
  return ret;
}
}  // namespace

DynamicConstraint::DynamicConstraint(int num_states, int num_inputs)
    : Constraint(num_states,
                 Eigen::VectorXd::Zero(num_states),
                 Eigen::VectorXd::Zero(num_states)),
      num_states_(num_states),
      num_inputs_(num_inputs) {}

DynamicConstraint::~DynamicConstraint() {}

void DynamicConstraint::Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                             Eigen::VectorXd& y) const {
  drake::TaylorVecXd y_t;
  Eval(drake::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DynamicConstraint::Eval(const Eigen::Ref<const drake::TaylorVecXd>& x,
                             drake::TaylorVecXd& y) const {
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_));

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const drake::TaylorVarXd h = x(0);
  const auto x0 = x.segment(1, num_states_);
  const auto x1 = x.segment(1 + num_states_, num_states_);
  const auto u0 = x.segment(1 + (2 * num_states_), num_inputs_);
  const auto u1 = x.segment(1 + (2 * num_states_) + num_inputs_, num_inputs_);

  // TODO(sam.creasey): We should cache the dynamics outputs to avoid
  // recalculating for every knot point as we advance.
  drake::TaylorVecXd xdot0;
  dynamics(x0, u0, &xdot0);
  Eigen::MatrixXd dxdot0 = ExtractDerivativesMatrix(xdot0);

  drake::TaylorVecXd xdot1;
  dynamics(x1, u1, &xdot1);
  Eigen::MatrixXd dxdot1 = ExtractDerivativesMatrix(xdot1);

  // Cubic interpolation to get xcol and xdotcol.
  const drake::TaylorVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const drake::TaylorVecXd xdotcol =
      -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

  drake::TaylorVecXd g;
  dynamics(xcol, 0.5 * (u0 + u1), &g);
  y = xdotcol - g;
}

}  // systems
}  // drake
