#include "drake/planning/trajectory_optimization/direct_collocation.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using math::AreAutoDiffVecXdEqual;
using math::ExtractGradient;
using math::ExtractValue;
using math::InitializeAutoDiff;
using math::InitializeAutoDiffTuple;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using systems::Context;
using systems::FixedInputPortValue;
using systems::InputPort;
using systems::InputPortIndex;
using systems::InputPortSelection;
using systems::PortDataType;
using systems::System;
using trajectories::PiecewisePolynomial;

namespace {
int CheckAndReturnStates(int states) {
  if (states <= 0) {
    throw std::logic_error(
        "This system doesn't have any continuous states. DirectCollocation "
        "only makes sense for systems with continuous-time dynamics.");
  }
  return states;
}

typedef std::pair<std::unique_ptr<System<AutoDiffXd>>,
                  std::unique_ptr<Context<AutoDiffXd>>>
    OwnedPair;

OwnedPair MakeAutoDiffXd(const systems::System<double>& system,
                         const systems::Context<double>& context) {
  auto system_ad = System<double>::ToAutoDiffXd(system);
  auto context_ad = system_ad->CreateDefaultContext();
  // TODO(russt): Add support for time-varying dynamics OR check for
  // time-invariance.
  context_ad->SetTimeStateAndParametersFrom(context);
  system_ad->FixInputPortsFrom(system, context, context_ad.get());
  return OwnedPair{std::move(system_ad), std::move(context_ad)};
}

}  // namespace

DirectCollocation::~DirectCollocation() {}

DirectCollocationConstraint::DirectCollocationConstraint(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    bool assume_non_continuous_states_are_fixed)
    : DirectCollocationConstraint(
          MakeAutoDiffXd(system, context),
          nullptr,                    // System
          nullptr, nullptr, nullptr,  // Contexts
          context.num_continuous_states(),
          system.get_input_port_selection(input_port_index)
              ? system.get_input_port_selection(input_port_index)->size()
              : 0,
          input_port_index, assume_non_continuous_states_are_fixed) {}

DirectCollocationConstraint::DirectCollocationConstraint(
    const systems::System<AutoDiffXd>& system,
    systems::Context<AutoDiffXd>* context_sample,
    systems::Context<AutoDiffXd>* context_next_sample,
    systems::Context<AutoDiffXd>* context_collocation,
    std::variant<systems::InputPortSelection, systems::InputPortIndex>
        input_port_index,
    bool assume_non_continuous_states_are_fixed)
    : DirectCollocationConstraint(
          OwnedPair{nullptr, nullptr}, &system, context_sample,
          context_next_sample, context_collocation,
          context_sample->num_continuous_states(),
          system.get_input_port_selection(input_port_index)
              ? system.get_input_port_selection(input_port_index)->size()
              : 0,
          input_port_index, assume_non_continuous_states_are_fixed) {}

DirectCollocationConstraint::DirectCollocationConstraint(
    OwnedPair owned_pair, const systems::System<AutoDiffXd>* system,
    systems::Context<AutoDiffXd>* context_sample,
    systems::Context<AutoDiffXd>* context_next_sample,
    systems::Context<AutoDiffXd>* context_collocation, int num_states,
    int num_inputs,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    bool assume_non_continuous_states_are_fixed)
    : Constraint(CheckAndReturnStates(num_states),
                 1 + (2 * num_states) + (2 * num_inputs),
                 Eigen::VectorXd::Zero(num_states),
                 Eigen::VectorXd::Zero(num_states)),
      owned_system_(std::move(owned_pair.first)),
      owned_context_(std::move(owned_pair.second)),
      system_(owned_system_ ? *owned_system_ : DRAKE_DEREF(system)),
      context_sample_(owned_context_ ? owned_context_.get() : context_sample),
      context_next_sample_(owned_context_ ? owned_context_.get()
                                          : context_next_sample),
      context_collocation_(owned_context_ ? owned_context_.get()
                                          : context_collocation),
      input_port_(system_.get_input_port_selection(input_port_index)),
      num_states_(num_states),
      num_inputs_(num_inputs) {
  system_.ValidateContext(context_sample_);
  system_.ValidateContext(context_next_sample_);
  system_.ValidateContext(context_collocation_);
  if (!assume_non_continuous_states_are_fixed) {
    DRAKE_THROW_UNLESS(context_sample_->has_only_continuous_state());
  }

  if (input_port_) {
    // Verify that the input port is not abstract valued.
    if (input_port_->get_data_type() == PortDataType::kAbstractValued) {
      throw std::logic_error(
          "The specified input port is abstract-valued, and this constraint "
          "only supports vector-valued input ports.  Did you perhaps forget to "
          "pass a non-default `input_port_index` argument?");
    }
  }
}

DirectCollocationConstraint::~DirectCollocationConstraint() = default;

void DirectCollocationConstraint::CalcDynamics(
    const AutoDiffVecXd& x_with_dvars, const AutoDiffVecXd& u_with_dvars,
    Context<AutoDiffXd>* context, AutoDiffVecXd* xdot_with_dvars) const {
  // To have cache hits, we must match not only the x and u values, but also
  // the derivatives. To get cache hits even when dvars are different, we use
  // the chain rule locally in this method, so that the gradients in the cache
  // are always with respect to exactly the current x and u.
  AutoDiffVecXd x_with_dxu, u_with_dxu;
  std::tie(x_with_dxu, u_with_dxu) = InitializeAutoDiffTuple(
      ExtractValue(x_with_dvars), ExtractValue(u_with_dvars));

  if (input_port_ &&
      (!input_port_->HasValue(*context) ||
       !AreAutoDiffVecXdEqual(u_with_dxu, input_port_->Eval(*context)))) {
    input_port_->FixValue(context, u_with_dxu);
  }
  if (!AreAutoDiffVecXdEqual(
          x_with_dxu, context->get_continuous_state_vector().CopyToVector())) {
    context->SetContinuousState(x_with_dxu);
  }

  AutoDiffVecXd xdot_with_dxu =
      system_.EvalTimeDerivatives(*context).CopyToVector();

  const VectorXd xdot = ExtractValue(xdot_with_dxu);
  const MatrixXd dxdot_dxu = ExtractGradient(xdot_with_dxu);

  xdot_with_dvars->resize(num_states_);
  // dxdot_dvars = dxdot_dx * dx_dvars + dxdot_du * du_dvars.
  InitializeAutoDiff(
      xdot,
      dxdot_dxu.leftCols(num_states_) * ExtractGradient(x_with_dvars) +
          dxdot_dxu.rightCols(num_inputs_) *
              ExtractGradient(u_with_dvars,
                              // pass in num_derivatives in case u is empty.
                              x_with_dvars[0].derivatives().size()),
      xdot_with_dvars);
}

void DirectCollocationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(x.cast<AutoDiffXd>(), &y_t);
  *y = ExtractValue(y_t);
}

// The format of the input to the eval() function is the
// tuple { time step, state 0, state 1, input 0, input 1 },
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

  AutoDiffVecXd xdot0;
  CalcDynamics(x0, u0, context_sample_, &xdot0);

  AutoDiffVecXd xdot1;
  CalcDynamics(x1, u1, context_next_sample_, &xdot1);

  // Cubic interpolation to get xcol and xdotcol.
  const AutoDiffVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
  const AutoDiffVecXd xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

  AutoDiffVecXd g;
  CalcDynamics(xcol, 0.5 * (u0 + u1), context_collocation_, &g);
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
    const Eigen::Ref<const VectorXDecisionVariable>& time_step,
    const Eigen::Ref<const VectorXDecisionVariable>& state,
    const Eigen::Ref<const VectorXDecisionVariable>& next_state,
    const Eigen::Ref<const VectorXDecisionVariable>& input,
    const Eigen::Ref<const VectorXDecisionVariable>& next_input,
    MathematicalProgram* prog) {
  DRAKE_DEMAND(time_step.size() == 1);
  DRAKE_DEMAND(state.size() == constraint->num_states());
  DRAKE_DEMAND(next_state.size() == constraint->num_states());
  DRAKE_DEMAND(input.size() == constraint->num_inputs());
  DRAKE_DEMAND(next_input.size() == constraint->num_inputs());
  return prog->AddConstraint(constraint,
                             {time_step, state, next_state, input, next_input});
}

DirectCollocation::DirectCollocation(
    const System<double>* system, const Context<double>& context,
    int num_time_samples, double minimum_time_step, double maximum_time_step,
    std::variant<InputPortSelection, InputPortIndex> input_port_index,
    bool assume_non_continuous_states_are_fixed,
    solvers::MathematicalProgram* prog)
    : MultipleShooting(
          system->get_input_port_selection(input_port_index)
              ? system->get_input_port_selection(input_port_index)->size()
              : 0,
          CheckAndReturnStates(context.num_continuous_states()),
          num_time_samples, minimum_time_step, maximum_time_step, prog),
      system_(system),
      context_(context.Clone()),
      input_port_index_(input_port_index),
      sample_contexts_(num_time_samples) {
  system->ValidateContext(context);
  if (!assume_non_continuous_states_are_fixed) {
    DRAKE_DEMAND(context.has_only_continuous_state());
  }

  auto system_and_context = MakeAutoDiffXd(*system, context);
  system_ad_ = std::move(system_and_context.first);
  context_ad_ = std::move(system_and_context.second);

  // Allocated contexts for each sample time. We share contexts across multiple
  // constraints in order to exploit caching (the dynamics at time k are
  // evaluated both in constraint k and k+1). Note that the constraints cannot
  // be evaluated in parallel.
  for (int i = 0; i < N(); ++i) {
    sample_contexts_[i] = context_ad_->Clone();
  }
  // We don't expect to have cache hits for the collocation contexts, so can
  // just use the one context_ad.

  // Add the dynamic constraints.
  // For N-1 time steps, add a constraint which depends on the breakpoint
  // along with the state and input vectors at that breakpoint and the
  // next.
  for (int i = 0; i < N() - 1; ++i) {
    auto constraint = std::make_shared<DirectCollocationConstraint>(
        *system_ad_, sample_contexts_[i].get(), sample_contexts_[i + 1].get(),
        context_ad_.get(), input_port_index,
        assume_non_continuous_states_are_fixed);
    this->prog()
        .AddConstraint(constraint,
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

  prog().AddCost(SubstitutePlaceholderVariables(g * h_vars()(0) / 2, 0));
  for (int i = 1; i <= N() - 2; i++) {
    prog().AddCost(SubstitutePlaceholderVariables(
        g * (h_vars()(i - 1) + h_vars()(i)) / 2, i));
  }
  prog().AddCost(
      SubstitutePlaceholderVariables(g * h_vars()(N() - 2) / 2, N() - 1));
}

PiecewisePolynomial<double> DirectCollocation::ReconstructInputTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  const InputPort<double>* input_port =
      system_->get_input_port_selection(input_port_index_);
  if (!input_port) {
    return PiecewisePolynomial<double>();
  }

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

  // Provide a fixed value for the input port and keep an alias around.
  const InputPort<double>* input_port =
      system_->get_input_port_selection(input_port_index_);
  FixedInputPortValue* input_port_value{nullptr};
  if (input_port) {
    input_port_value = &input_port->FixValue(
        context_.get(), system_->AllocateInputVector(*input_port)->get_value());
  }

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = result.GetSolution(state(i));
    if (input_port) {
      input_port_value->GetMutableVectorData<double>()->SetFromVector(
          result.GetSolution(input(i)));
    }
    context_->SetContinuousState(states[i]);
    derivatives[i] = system_->EvalTimeDerivatives(*context_).CopyToVector();
  }
  return PiecewisePolynomial<double>::CubicHermite(times_vec, states,
                                                   derivatives);
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
