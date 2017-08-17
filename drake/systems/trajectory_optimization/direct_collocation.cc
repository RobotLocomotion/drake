#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/systems/trajectory_optimization/direct_collocation_constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

DirectCollocation::DirectCollocation(const systems::System<double>* system,
                                     const systems::Context<double>& context,
                                     int num_time_samples,
                                     double trajectory_time_lower_bound,
                                     double trajectory_time_upper_bound)
    : MultipleShooting(system->get_num_total_inputs(),
                       context.get_continuous_state()->size(), num_time_samples,
                       trajectory_time_lower_bound / (num_time_samples - 1),
                       trajectory_time_upper_bound / (num_time_samples - 1)),
      system_(system),
      context_(system_->CreateDefaultContext()),
      continuous_state_(system_->AllocateTimeDerivatives()) {
  DRAKE_DEMAND(context.has_only_continuous_state());

  // TODO(russt):  This should NOT be set automatically.
  // Once it is removed, the proper constraint to be added will be
  //   AddDurationBounds(trajectory_time_lower_bound,
  //                     trajectory_time_upper_bound);
  // but that constraint is currently implied already by the min/max
  // timestep + all timesteps equal constraints.
  AddEqualTimeIntervalsConstraints();

  context_->SetTimeStateAndParametersFrom(context);

  if (context_->get_num_input_ports() > 0) {
    // Allocate the input port and keep an alias around.
    input_port_value_ = new FreestandingInputPortValue(
        system->AllocateInputVector(system->get_input_port(0)));
    std::unique_ptr<InputPortValue> input_port_value(input_port_value_);
    context_->SetInputPortValue(0, std::move(input_port_value));
  }

  // Add the dynamic constraints.
  auto constraint =
      std::make_shared<SystemDirectCollocationConstraint>(*system, context);

  DRAKE_ASSERT(static_cast<int>(constraint->num_constraints()) == num_states());

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.
  for (int i = 0; i < N() - 1; i++) {
    AddConstraint(constraint,
                  {h_vars().segment<1>(i),
                   x_vars().segment(i * num_states(), num_states() * 2),
                   u_vars().segment(i * num_inputs(), num_inputs() * 2)});
  }
}

void DirectCollocation::DoAddRunningCost(const symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  AddCost(0.5 * SubstitutePlaceholderVariables(g * h_vars()(0) / 2, 0));
  for (int i = 1; i < N() - 2; i++) {
    AddCost(SubstitutePlaceholderVariables(
        g * (h_vars()(i - 1) + h_vars()(i)) / 2, i));
  }
  AddCost(0.5 *
          SubstitutePlaceholderVariables(g * h_vars()(N() - 2) / 2, N() - 1));
}

PiecewisePolynomialTrajectory DirectCollocation::ReconstructInputTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs));
}

PiecewisePolynomialTrajectory DirectCollocation::ReconstructStateTrajectory()
    const {
  // TODO(russt): Fix this!  It's not using the same interpolation scheme as the
  // actual collocation method.
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());
  std::vector<Eigen::MatrixXd> derivatives(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
    input_port_value_->GetMutableVectorData<double>()->SetFromVector(
        GetSolution(input(i)));
    context_->get_mutable_continuous_state()->SetFromVector(states[i]);
    system_->CalcTimeDerivatives(*context_, continuous_state_.get());
    derivatives[i] = continuous_state_->CopyToVector();
  }
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::Cubic(times_vec, states, derivatives));
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
