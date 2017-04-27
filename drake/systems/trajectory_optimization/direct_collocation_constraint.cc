#include "drake/systems/trajectory_optimization/direct_collocation_constraint.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace systems {
namespace {
Eigen::MatrixXd ExtractDerivativesMatrix(const AutoDiffVecXd& vec_in) {
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

DirectCollocationConstraint::DirectCollocationConstraint(int num_states,
                                                         int num_inputs)
    : Constraint(num_states, 1 + (2 * num_states) + (2 * num_inputs),
                 Eigen::VectorXd::Zero(num_states),
                 Eigen::VectorXd::Zero(num_states)),
      num_states_(num_states),
      num_inputs_(num_inputs) {}

DirectCollocationConstraint::~DirectCollocationConstraint() {}

void DirectCollocationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DirectCollocationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_));

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const AutoDiffXd h = x(0);
  const auto x0 = x.segment(1, num_states_);
  const auto x1 = x.segment(1 + num_states_, num_states_);
  const auto u0 = x.segment(1 + (2 * num_states_), num_inputs_);
  const auto u1 = x.segment(1 + (2 * num_states_) + num_inputs_, num_inputs_);

  // TODO(sam.creasey): We should cache the dynamics outputs to avoid
  // recalculating for every knot point as we advance.
  AutoDiffVecXd xdot0;
  dynamics(x0, u0, &xdot0);
  Eigen::MatrixXd dxdot0 = ExtractDerivativesMatrix(xdot0);

  AutoDiffVecXd xdot1;
  dynamics(x1, u1, &xdot1);
  Eigen::MatrixXd dxdot1 = ExtractDerivativesMatrix(xdot1);

  // Cubic interpolation to get xcol and xdotcol.
  const AutoDiffVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const AutoDiffVecXd xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

  AutoDiffVecXd g;
  dynamics(xcol, 0.5 * (u0 + u1), &g);
  y = xdotcol - g;
}

SystemDirectCollocationConstraint::SystemDirectCollocationConstraint(
    const systems::System<double>& system,
    const systems::Context<double>& context)
    : DirectCollocationConstraint(context.get_continuous_state()->size(),
                                  context.get_num_input_ports() > 0
                                      ? system.get_input_port(0).size()
                                      : 0),
      system_(systems::System<double>::ToAutoDiffXd(system)),
      context_(system_->CreateDefaultContext()),
      // Don't allocate the input port until we're past the point
      // where we might throw.
      derivatives_(system_->AllocateTimeDerivatives()) {
  DRAKE_THROW_UNLESS(system_->get_num_input_ports() <= 1);
  DRAKE_THROW_UNLESS(context.has_only_continuous_state());

  // TODO(russt): Add support for time-varying dynamics OR check for
  // time-invariance.

  context_->SetTimeStateAndParametersFrom(context);
  // Set derivatives of all parameters in the context to zero (but with the
  // correct size).
  int num_gradients = 1 + 2 * num_states() + 2 * num_inputs();
  for (int i = 0; i < context_->get_parameters().num_numeric_parameters();
       i++) {
    auto params = context_->get_mutable_parameters()
                      .get_mutable_numeric_parameter(i)
                      ->get_mutable_value();
    for (int j = 0; j < params.size(); j++) {
      auto& derivs = params(j).derivatives();
      if (derivs.size() == 0) {
        derivs.resize(num_gradients);
        derivs.setZero();
      }
    }
  }

  if (context.get_num_input_ports() > 0) {
    // Allocate the input port and keep an alias around.
    input_port_value_ = new FreestandingInputPortValue(
        system_->AllocateInputVector(system_->get_input_port(0)));
    std::unique_ptr<InputPortValue> input_port_value(input_port_value_);
    context_->SetInputPortValue(0, std::move(input_port_value));
  }
}

SystemDirectCollocationConstraint::~SystemDirectCollocationConstraint() {}

void SystemDirectCollocationConstraint::dynamics(const AutoDiffVecXd& state,
                                                 const AutoDiffVecXd& input,
                                                 AutoDiffVecXd* xdot) const {
  if (context_->get_num_input_ports() > 0) {
    input_port_value_->GetMutableVectorData<AutoDiffXd>()->SetFromVector(input);
  }
  context_->get_mutable_continuous_state()->SetFromVector(state);
  system_->CalcTimeDerivatives(*context_, derivatives_.get());
  *xdot = derivatives_->CopyToVector();
}

}  // namespace systems
}  // namespace drake
