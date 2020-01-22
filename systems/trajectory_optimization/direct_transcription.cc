#include "drake/systems/trajectory_optimization/direct_transcription.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/system_symbolic_inspector.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using trajectories::PiecewisePolynomial;

namespace {

// Implements a constraint on the defect between the state variables
// advanced for one discrete step or one integration for a fixed timestep,
// and the decision variable representing the next state.
class DirectTranscriptionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectTranscriptionConstraint)

  // @param system The system describing the dynamics of the constraint.
  // The reference must remain valid for the lifetime of this constraint.
  // @param context A mutable pointer to a context that will be written to in
  // order to perform the dynamics evaluations.  This context must also
  // stay valid for the lifetime of this constraint.
  // @param input_port_value A pre-allocated mutable pointer for writing the
  // input value, which must be assigned as an input to @p context.  It must
  // also remain valid.
  // @param num_states the integer size of the discrete or continuous
  // state vector being optimized.
  // @param num_inputs the integer size of the input vector being optimized.
  // @param evaluation_time  The time along the trajectory at which this
  // constraint is evaluated.
  // @param fixed_timestep Defines the explicit Euler integration
  // timestep for systems with continuous state variables.
  DirectTranscriptionConstraint(const System<AutoDiffXd>& system,
                               Context<AutoDiffXd>* context,
                               FixedInputPortValue* input_port_value,
                               int num_states, int num_inputs,
                               double evaluation_time, TimeStep fixed_timestep)
      : Constraint(num_states, num_inputs + 2 * num_states,
                   Eigen::VectorXd::Zero(num_states),
                   Eigen::VectorXd::Zero(num_states)),
        system_(system),
        context_(context),
        input_port_value_(input_port_value),
        num_states_(num_states),
        num_inputs_(num_inputs),
        evaluation_time_(evaluation_time),
        fixed_timestep_(fixed_timestep.value) {
    DRAKE_DEMAND(evaluation_time >= 0.0);
    DRAKE_DEMAND(context_->has_only_discrete_state() ||
                 context_->has_only_continuous_state());
    DRAKE_DEMAND(context_ != nullptr);
    DRAKE_DEMAND(context_->num_input_ports() == 0 ||
                 input_port_value_ != nullptr);

    if (context_->has_only_discrete_state()) {
      discrete_state_ = system_.AllocateDiscreteVariables();
    } else {
      DRAKE_DEMAND(fixed_timestep_ > 0.0);
    }

    // Makes sure the autodiff vector is properly initialized.
    evaluation_time_.derivatives().resize(2 * num_states_ + num_inputs_);
    evaluation_time_.derivatives().setZero();
  }

  ~DirectTranscriptionConstraint() override = default;

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(math::initializeAutoDiff(x), &y_t);
    *y = math::autoDiffToValueMatrix(y_t);
  }

  // The format of the input to the eval() function is a vector
  // containing {input, state, next_state}.
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DRAKE_ASSERT(x.size() == num_inputs_ + (2 * num_states_));

    // Extract our input variables:
    const auto input = x.head(num_inputs_);
    const auto state = x.segment(num_inputs_, num_states_);
    const auto next_state = x.tail(num_states_);

    context_->SetTime(evaluation_time_);
    if (context_->num_input_ports() > 0) {
      input_port_value_->GetMutableVectorData<AutoDiffXd>()->SetFromVector(
          input);
    }

    if (context_->has_only_continuous_state()) {
      context_->SetContinuousState(state);
      // Compute the defect between next_state and the explicit Euler
      // integration.
      *y = next_state -
           (state + fixed_timestep_ *
                        system_.EvalTimeDerivatives(*context_).CopyToVector());
    } else {
      context_->get_mutable_discrete_state(0).SetFromVector(state);
      system_.CalcDiscreteVariableUpdates(*context_, discrete_state_.get());
      *y = next_state - discrete_state_->get_vector(0).CopyToVector();
    }
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "DirectTranscriptionConstraint does not support symbolic evaluation.");
  }

 private:
  const System<AutoDiffXd>& system_;
  Context<AutoDiffXd>* const context_{nullptr};
  std::unique_ptr<DiscreteValues<AutoDiffXd>> discrete_state_;
  FixedInputPortValue* const input_port_value_{nullptr};

  const int num_states_{0};
  const int num_inputs_{0};
  AutoDiffXd evaluation_time_{0};
  const double fixed_timestep_{0};
};

double get_period(const System<double>* system) {
  std::optional<PeriodicEventData> periodic_data =
      system->GetUniquePeriodicDiscreteUpdateAttribute();
  DRAKE_DEMAND(periodic_data.has_value());
  DRAKE_DEMAND(periodic_data->offset_sec() == 0.0);
  return periodic_data->period_sec();
}

int get_input_port_size(
    const System<double>* system,
    std::variant<InputPortSelection, InputPortIndex> input_port_index) {
  if (system->get_input_port_selection(input_port_index)) {
    return system->get_input_port_selection(input_port_index)->size();
  } else {
    return 0;
  }
}

}  // end namespace

DirectTranscription::DirectTranscription(
    const System<double>* system, const Context<double>& context,
    int num_time_samples,
    std::variant<InputPortSelection, InputPortIndex> input_port_index)
    : MultipleShooting(get_input_port_size(system, input_port_index),
                       context.num_total_states(), num_time_samples,
                       get_period(system)),
      discrete_time_system_(true) {
  // Note: this constructor is for discrete-time systems.  For continuous-time
  // systems, you must use a different constructor that specifies the timesteps.
  ValidateSystem(*system, context, input_port_index);

  // First try symbolic dynamics.
  if (!AddSymbolicDynamicConstraints(system, context)) {
    AddAutodiffDynamicConstraints(system, context, input_port_index);
  }
  ConstrainEqualInputAtFinalTwoTimesteps();
}

DirectTranscription::DirectTranscription(
    const TimeVaryingLinearSystem<double>* system,
    const Context<double>& context, int num_time_samples,
    std::variant<InputPortSelection, InputPortIndex> input_port_index)
    : MultipleShooting(get_input_port_size(system, input_port_index),
          context.num_total_states(), num_time_samples,
          std::max(system->time_period(),
                   std::numeric_limits<double>::epsilon())
          /* N.B. Ensures that MultipleShooting is well-formed */),
      discrete_time_system_(true) {
  // Note: this constructor is for discrete-time systems.  For continuous-time
  // systems, you must use a different constructor that specifies the timesteps.
  ValidateSystem(*system, context, input_port_index);

  for (int i = 0; i < N() - 1; i++) {
    const double t = system->time_period() * i;
    AddLinearEqualityConstraint(
        state(i+1).cast<symbolic::Expression>() ==
        system->A(t) * state(i).cast<symbolic::Expression>() +
        system->B(t) * input(i).cast<symbolic::Expression>());
  }
  ConstrainEqualInputAtFinalTwoTimesteps();
}

DirectTranscription::DirectTranscription(
    const System<double>* system, const Context<double>& context,
    int num_time_samples, TimeStep fixed_timestep,
    std::variant<InputPortSelection, InputPortIndex> input_port_index)
    : MultipleShooting(get_input_port_size(system, input_port_index),
                       context.num_total_states(), num_time_samples,
                       fixed_timestep.value),
      discrete_time_system_(false) {
  DRAKE_DEMAND(context.has_only_continuous_state());
  DRAKE_DEMAND(system->num_input_ports() <= 1);
  if (context.num_input_ports() > 0) {
    DRAKE_DEMAND(num_inputs() == get_input_port_size(system, input_port_index));
  }

  // First try symbolic dynamics.
  if (!AddSymbolicDynamicConstraints(system, context)) {
    AddAutodiffDynamicConstraints(system, context, input_port_index);
  }
  ConstrainEqualInputAtFinalTwoTimesteps();
}


void DirectTranscription::DoAddRunningCost(const symbolic::Expression& g) {
  DRAKE_DEMAND(discrete_time_system_);  // TODO(russt): implement
                                        // continuous-time version.

  // Cost = \sum_n g(n,x[n],u[n]) dt
  for (int i = 0; i < N() - 1; i++) {
    AddCost(SubstitutePlaceholderVariables(g * fixed_timestep(), i));
  }
}

PiecewisePolynomial<double> DirectTranscription::ReconstructInputTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = result.GetSolution(input(i));
  }
  // TODO(russt): Implement DTTrajectories and return one of those instead.
  return PiecewisePolynomial<double>::ZeroOrderHold(times_vec, inputs);
}

PiecewisePolynomial<double> DirectTranscription::ReconstructStateTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = result.GetSolution(state(i));
  }
  // TODO(russt): Implement DTTrajectories and return one of those instead.
  return PiecewisePolynomial<double>::ZeroOrderHold(times_vec, states);
}

bool DirectTranscription::AddSymbolicDynamicConstraints(
    const System<double>* system, const Context<double>& context) {
  const auto symbolic_system = system->ToSymbolicMaybe();
  if (!symbolic_system) {
    return false;
  }

  const auto inspector =
      std::make_unique<SystemSymbolicInspector>(*symbolic_system);
  if (!inspector->HasAffineDynamics()) {
    return false;
  }

  symbolic::Substitution sub;
  for (int i = 0; i < context.num_numeric_parameter_groups(); i++) {
    const auto& params = context.get_numeric_parameter(i).get_value();
    for (int j = 0; j < params.size(); j++) {
      sub.emplace(inspector->numeric_parameters(i)[j], params[j]);
    }
  }

  for (int i = 0; i < N() - 1; i++) {
    sub[inspector->time()] = i * fixed_timestep();
    // TODO(russt/soonho): Can we make a cleaner way to do substitutions
    // with Vectors to avoid these loops appearing everywhere? #6925
    for (int j = 0; j < num_inputs(); j++) {
      sub[inspector->input(0)[j]] = input(i)[j];
    }

    if (discrete_time_system_) {
      VectorX<symbolic::Expression> update = inspector->discrete_update(0);
      for (int j = 0; j < num_states(); j++) {
        sub[inspector->discrete_state(0)[j]] = state(i)[j];
      }
      for (int j = 0; j < num_states(); j++) {
        update(j) = update(j).Substitute(sub);
      }
      AddLinearEqualityConstraint(state(i + 1) == update);
    } else {
      VectorX<symbolic::Expression> derivatives = inspector->derivatives();
      for (int j = 0; j < num_states(); j++) {
        sub[inspector->continuous_state()[j]] = state(i)[j];
      }
      for (int j = 0; j < num_states(); j++) {
        derivatives(j) = derivatives(j).Substitute(sub);
      }
      // The next state should match the explicit Euler integration.
      AddLinearEqualityConstraint(state(i + 1) ==
                                  state(i) + fixed_timestep() * derivatives);
    }
  }
  return true;
}

void DirectTranscription::AddAutodiffDynamicConstraints(
    const System<double>* system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index) {
  system_ = system->ToAutoDiffXd();
  DRAKE_DEMAND(system_ != nullptr);
  context_ = system_->CreateDefaultContext();
  input_port_ = system_->get_input_port_selection(input_port_index);

  context_->SetTimeStateAndParametersFrom(context);

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

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.
  for (int i = 0; i < N() - 1; i++) {
    // Add the dynamic constraints.
    // Note that these constraints all share a context and inout_port_value,
    // so should not be evaluated in parallel.
    auto constraint = std::make_shared<DirectTranscriptionConstraint>(
        *system_, context_.get(), input_port_value_, num_states(), num_inputs(),
        i * fixed_timestep(), TimeStep{fixed_timestep()});

    AddConstraint(constraint, {input(i), state(i), state(i + 1)});
  }
}

void DirectTranscription::ConstrainEqualInputAtFinalTwoTimesteps() {
  if (num_inputs() > 0) {
    AddLinearEqualityConstraint(input(N() - 2) == input(N() - 1));
  }
}

void DirectTranscription::ValidateSystem(
    const System<double>& system, const Context<double>& context,
    std::variant<InputPortSelection, InputPortIndex> input_port_index) {
  DRAKE_DEMAND(system.IsDifferenceEquationSystem());
  DRAKE_DEMAND(num_states() == context.get_discrete_state(0).size());
  if (context.num_input_ports() > 0) {
    DRAKE_DEMAND(num_inputs() ==
                 get_input_port_size(&system, input_port_index));
  }
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
