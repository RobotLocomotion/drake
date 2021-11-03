#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using trajectories::PiecewisePolynomial;

DirectCollocationConstraint::DirectCollocationConstraint(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    bool assume_non_continuous_states_are_fixed)
    : DirectCollocationConstraint(
          system, context, context.num_continuous_states(),
          system.get_input_port_selection(input_port_index)
              ? system.get_input_port_selection(input_port_index)->size()
              : 0,
          input_port_index, assume_non_continuous_states_are_fixed) {}

DirectCollocationConstraint::DirectCollocationConstraint(
    const System<double>& system, const Context<double>& context,
    int num_states, int num_inputs,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    bool assume_non_continuous_states_are_fixed)
    : Constraint(num_states, 1 + (2 * num_states) + (2 * num_inputs),
                 Eigen::VectorXd::Zero(num_states),
                 Eigen::VectorXd::Zero(num_states)),
      system_(System<double>::ToAutoDiffXd(system)),
      context_(system_->CreateDefaultContext()),
      input_port_(system_->get_input_port_selection(input_port_index)),
      // Don't allocate the input port value until we're past the point
      // where we might throw.
      derivatives_(system_->AllocateTimeDerivatives()),
      num_states_(num_states),
      num_inputs_(num_inputs) {
  if (!assume_non_continuous_states_are_fixed) {
    DRAKE_THROW_UNLESS(context.has_only_continuous_state());
  }

  // TODO(russt): Add support for time-varying dynamics OR check for
  // time-invariance.

  context_->SetTimeStateAndParametersFrom(context);
  system_->FixInputPortsFrom(system, context, context_.get());

  if (input_port_) {
    // Verify that the input port is not abstract valued.
    if (input_port_->get_data_type() == PortDataType::kAbstractValued) {
      throw std::logic_error(
          "Port requested for differentiation is abstract, and differentiation "
          "of abstract ports is not supported.");
    }

    // Provide a fixed value for the input port and keep an alias around.
    input_port_value_ = &input_port_->FixValue(
        context_.get(),
        system_->AllocateInputVector(*input_port_)->get_value());
  }
}

void DirectCollocationConstraint::dynamics(const AutoDiffVecXd& state,
                                           const AutoDiffVecXd& input,
                                           AutoDiffVecXd* xdot) const {
  if (input_port_) {
    input_port_value_->GetMutableVectorData<AutoDiffXd>()->SetFromVector(input);
  }
  context_->SetContinuousState(state);
  system_->CalcTimeDerivatives(*context_, derivatives_.get());
  *xdot = derivatives_->CopyToVector();
}

void DirectCollocationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(math::InitializeAutoDiff(x), &y_t);
  *y = math::ExtractValue(y_t);
}

// The format of the input to the eval() function is the
// tuple { timestep, state 0, state 1, input 0, input 1 },
// which has a total length of 1 + 2*num_states + 2*num_inputs.
void DirectCollocationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_));

  // Extract our input variables:
  // h - current time (breakpoint)
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const AutoDiffXd h = x(0);
  const auto x0 = x.segment(1, num_states_);
  const auto x1 = x.segment(1 + num_states_, num_states_);
  const auto u0 = x.segment(1 + (2 * num_states_), num_inputs_);
  const auto u1 = x.segment(1 + (2 * num_states_) + num_inputs_, num_inputs_);

  // TODO(sam.creasey): Use caching to avoid recomputing the dynamics.
  // Currently the dynamics evaluated here as {u1,x1} are recomputed in the
  // next constraint as {u0,x0}.
  AutoDiffVecXd xdot0;
  dynamics(x0, u0, &xdot0);

  AutoDiffVecXd xdot1;
  dynamics(x1, u1, &xdot1);

  // Cubic interpolation to get xcol and xdotcol.
  const AutoDiffVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const AutoDiffVecXd xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

  AutoDiffVecXd g;
  dynamics(xcol, 0.5 * (u0 + u1), &g);
  *y = xdotcol - g;
}

void DirectCollocationConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "DirectCollocationConstraint does not support symbolic evaluation.");
}

Binding<Constraint> AddDirectCollocationConstraint(
    std::shared_ptr<DirectCollocationConstraint> constraint,
    const Eigen::Ref<const VectorXDecisionVariable>& timestep,
    const Eigen::Ref<const VectorXDecisionVariable>& state,
    const Eigen::Ref<const VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const VectorXDecisionVariable>& input,
    const Eigen::Ref<const VectorXDecisionVariable>& next_input,
    MathematicalProgram* prog) {
  DRAKE_DEMAND(timestep.size() == 1);
  DRAKE_DEMAND(state.size() == constraint->num_states());
  DRAKE_DEMAND(next_state.size() == constraint->num_states());
  DRAKE_DEMAND(input.size() == constraint->num_inputs());
  DRAKE_DEMAND(next_input.size() == constraint->num_inputs());
  return prog->AddConstraint(constraint,
                             {timestep, state, next_state, input, next_input});
}

DirectCollocation::DirectCollocation(
    const System<double>* system, const Context<double>& context,
    int num_time_samples, double minimum_timestep, double maximum_timestep,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    bool assume_non_continuous_states_are_fixed)
    : MultipleShooting(
          system->get_input_port_selection(input_port_index)
              ? system->get_input_port_selection(input_port_index)->size()
              : 0,
          context.num_continuous_states(), num_time_samples,
          minimum_timestep, maximum_timestep),
      system_(system),
      context_(context.Clone()),
      continuous_state_(system_->AllocateTimeDerivatives()),
      input_port_(system->get_input_port_selection(input_port_index)) {
  if (!assume_non_continuous_states_are_fixed) {
    DRAKE_DEMAND(context.has_only_continuous_state());
  }

  if (input_port_) {
    // Allocate the input port and keep an alias around.
    input_port_value_ = &input_port_->FixValue(
        context_.get(),
        system_->AllocateInputVector(*input_port_)->get_value());
  }

  // Add the dynamic constraints.
  auto constraint = std::make_shared<DirectCollocationConstraint>(
      *system, context, input_port_index,
      assume_non_continuous_states_are_fixed);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the breakpoint
  // along with the state and input vectors at that breakpoint and the
  // next.
  for (int i = 0; i < N() - 1; i++) {
    AddConstraint(constraint,
                  {h_vars().segment<1>(i),
                   x_vars().segment(i * num_states(), num_states() * 2),
                   u_vars().segment(i * num_inputs(), num_inputs() * 2)})
        .evaluator()
        ->set_description(
            fmt::format("collocation constraint for segment {}", i));
  }
}

void DirectCollocation::DoAddRunningCost(const symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(SubstitutePlaceholderVariables(g * h_vars()(0) / 2, 0));
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(SubstitutePlaceholderVariables(
        g * (h_vars()(i - 1) + h_vars()(i)) / 2, i));
  }
  AddCost(SubstitutePlaceholderVariables(g * h_vars()(N() - 2) / 2, N() - 1));
}

PiecewisePolynomial<double> DirectCollocation::ReconstructInputTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  DRAKE_DEMAND(input_port_ != nullptr);
  Eigen::VectorXd times = GetSampleTimes(result);
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = result.GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs);
}

PiecewisePolynomial<double> DirectCollocation::ReconstructStateTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());
  std::vector<Eigen::MatrixXd> derivatives(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = result.GetSolution(state(i));
    if (input_port_) {
      input_port_value_->GetMutableVectorData<double>()->SetFromVector(
          result.GetSolution(input(i)));
    }
    context_->SetContinuousState(states[i]);
    system_->CalcTimeDerivatives(*context_, continuous_state_.get());
    derivatives[i] = continuous_state_->CopyToVector();
  }
  return PiecewisePolynomial<double>::CubicHermite(times_vec, states,
                                                   derivatives);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
