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

class DiscreteTimeSystemConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeSystemConstraint)

  // @param evaluation_time  The time along the trajectory at which this
  // constraint is evaluated.
  DiscreteTimeSystemConstraint(const System<AutoDiffXd>& system,
                               Context<AutoDiffXd>* context,
                               DiscreteValues<AutoDiffXd>* discrete_state,
                               FixedInputPortValue* input_port_value,
                               int num_states, int num_inputs,
                               double evaluation_time)
      : Constraint(num_states, num_inputs + 2 * num_states,
                   Eigen::VectorXd::Zero(num_states),
                   Eigen::VectorXd::Zero(num_states)),
        system_(system),
        context_(context),
        input_port_value_(input_port_value),
        discrete_state_(discrete_state),
        num_states_(num_states),
        num_inputs_(num_inputs),
        evaluation_time_(evaluation_time) {
    DRAKE_DEMAND(evaluation_time >= 0.0);
    DRAKE_DEMAND(context_->has_only_discrete_state());
    DRAKE_DEMAND(context_ != nullptr);
    DRAKE_DEMAND(discrete_state_ != nullptr);
    DRAKE_DEMAND(context_->get_num_input_ports() == 0 ||
                 input_port_value_ != nullptr);

    // Makes sure the autodiff vector is properly initialized.
    evaluation_time_.derivatives().resize(2 * num_states_ + num_inputs_);
    evaluation_time_.derivatives().setZero();
  }

  ~DiscreteTimeSystemConstraint() override = default;

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

    context_->set_time(evaluation_time_);
    if (context_->get_num_input_ports() > 0) {
      input_port_value_->GetMutableVectorData<AutoDiffXd>()->SetFromVector(
          input);
    }
    context_->get_mutable_discrete_state(0).SetFromVector(state);

    system_.CalcDiscreteVariableUpdates(*context_, discrete_state_);
    *y = next_state - discrete_state_->get_vector(0).CopyToVector();
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "DiscreteTimeSystemConstraint does not support symbolic evaluation.");
  }

 private:
  const System<AutoDiffXd>& system_;
  Context<AutoDiffXd>* const context_;
  FixedInputPortValue* const input_port_value_;
  DiscreteValues<AutoDiffXd>* const discrete_state_;

  const int num_states_{0};
  const int num_inputs_{0};
  AutoDiffXd evaluation_time_{0};
};

double get_period(const System<double>* system) {
  optional<PeriodicEventData> periodic_data =
      system->GetUniquePeriodicDiscreteUpdateAttribute();
  DRAKE_DEMAND(periodic_data.has_value());
  DRAKE_DEMAND(periodic_data->offset_sec() == 0.0);
  return periodic_data->period_sec();
}

}  // end namespace

DirectTranscription::DirectTranscription(const System<double>* system,
                                         const Context<double>& context,
                                         int num_time_samples,
                                         int input_port_index)
    : MultipleShooting(system->get_input_port(input_port_index).size(),
                       context.get_num_total_states(), num_time_samples,
                       get_period(system)),
      discrete_time_system_(true) {
  // Note: this constructor is for discrete-time systems.  For continuous-time
  // systems, you must use a different constructor that specifies the timesteps.
  ValidateSystem(*system, context, input_port_index);

  // First try symbolic dynamics.
  //  if (!AddSymbolicDynamicConstraints(system, context)) {
  //    AddAutodiffDynamicConstraints(system, context);
  //  }
  AddAutodiffDynamicConstraints(system, context, input_port_index);
  ConstrainEqualInputAtFinalTwoTimesteps();
}

DirectTranscription::DirectTranscription(
    const LinearSystem<double>* linear_system,
    const Context<double>& context,
    int num_time_samples)
    : MultipleShooting(linear_system->get_num_total_inputs(),
                       context.get_num_total_states(), num_time_samples,
                       std::max(linear_system->time_period(),
                                std::numeric_limits<double>::epsilon())
                       /* N.B. Ensures that MultipleShooting is well-formed */),
      discrete_time_system_(true) {
  // Note: this constructor is for discrete-time systems.  For continuous-time
  // systems, you must use a different constructor that specifies the timesteps.
  ValidateSystem(*linear_system, context);

  for (int i = 0; i < N() - 1; i++) {
    AddLinearEqualityConstraint(
        state(i+1).cast<symbolic::Expression>() ==
        linear_system->A() * state(i).cast<symbolic::Expression>() +
        linear_system->B() * input(i).cast<symbolic::Expression>());
  }
  ConstrainEqualInputAtFinalTwoTimesteps();
}

DirectTranscription::DirectTranscription(
    const TimeVaryingLinearSystem<double>* system,
    const Context<double>& context, int num_time_samples)
    : MultipleShooting(system->get_num_total_inputs(),
                       context.get_num_total_states(), num_time_samples,
                       std::max(system->time_period(),
                                std::numeric_limits<double>::epsilon())
                       /* N.B. Ensures that MultipleShooting is well-formed */),
      discrete_time_system_(true) {
  // Note: this constructor is for discrete-time systems.  For continuous-time
  // systems, you must use a different constructor that specifies the timesteps.
  ValidateSystem(*system, context);

  for (int i = 0; i < N() - 1; i++) {
    const double t = system->time_period() * i;
    AddLinearEqualityConstraint(
        state(i+1).cast<symbolic::Expression>() ==
        system->A(t) * state(i).cast<symbolic::Expression>() +
        system->B(t) * input(i).cast<symbolic::Expression>());
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
PiecewisePolynomial<double>
DirectTranscription::ReconstructInputTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  // TODO(russt): Implement DTTrajectories and return one of those instead.
  return PiecewisePolynomial<double>::ZeroOrderHold(times_vec, inputs);
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
PiecewisePolynomial<double> DirectTranscription::ReconstructStateTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
  }
  // TODO(russt): Implement DTTrajectories and return one of those instead.
  return PiecewisePolynomial<double>::ZeroOrderHold(times_vec, states);
}
#pragma GCC diagnostic pop

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
    VectorX<symbolic::Expression> update = inspector->discrete_update(0);
    sub[inspector->time()] = i * fixed_timestep();
    // TODO(russt/soonho): Can we make a cleaner way to do substitutions
    // with Vectors to avoid these loops appearing everywhere? #6925
    for (int j = 0; j < num_states(); j++) {
      sub[inspector->discrete_state(0)[j]] = state(i)[j];
    }
    for (int j = 0; j < num_inputs(); j++) {
      sub[inspector->input(0)[j]] = input(i)[j];
    }
    for (int j = 0; j < num_states(); j++) {
      update(j) = update(j).Substitute(sub);
    }
    AddLinearEqualityConstraint(state(i + 1) == update);
  }
  return true;
}

void DirectTranscription::AddAutodiffDynamicConstraints(
    const System<double>* system, const Context<double>& context,
    int input_port_index) {
  system_ = system->ToAutoDiffXd();
  DRAKE_DEMAND(system_ != nullptr);
  context_ = system_->CreateDefaultContext();
  discrete_state_ = system_->AllocateDiscreteVariables();

  context_->SetTimeStateAndParametersFrom(context);

  if (context_->get_num_input_ports() > 0) {
    // Specify the input port of interest
    if (input_port_index >= 0 &&
        input_port_index < system->get_num_input_ports()) {
      // This is the input port we care about.
      auto input_port = &(system->get_input_port(input_port_index));

      // Verify that the input port is not abstract valued.
      if (input_port &&
          input_port->get_data_type() == PortDataType::kAbstractValued) {
        throw std::logic_error(
            "Port requested for differentiation is abstract, and "
            "differentiation "
            "of abstract ports is not supported.");
      }

      // Allocate the input port and keep an alias around.
      input_port_value_ = &context_->FixInputPort(
          input_port_index, system_->AllocateInputVector(
                                system_->get_input_port(input_port_index)));
    }
    // Fix autodiff'd versions of the inputs to the autodiff'd Context.
    for (int i = 0; i < system->get_num_input_ports(); ++i) {
      // The input port we care about shouldn't be connected.
      if (i == input_port_index) {
        continue;
      }

      const InputPort<double>& input_port_i =
          system->get_input_port(InputPortIndex(i));

      // Look for abstract valued port.
      if (input_port_i.get_data_type() == PortDataType::kAbstractValued) {
        if (input_port_i.HasValue(context)) {
          throw std::logic_error(fmt::format(
              "Unable to linearize system with connected abstract port ({}) - "
              "connected abstract ports not yet supported.",
              input_port_i.get_name()));
        }
        continue;
      }

      // Must be a vector valued port. First look to see whether it's connected
      // or zero-dimensional.
      if ((input_port_i.size() > 0) && !input_port_i.HasValue(context)) {
          Eigen::VectorXd u(input_port_i.size());
          u.setZero();
          context_->FixInputPort(i, u.cast<AutoDiffXd>());
      }
    }
  }

  // For N-1 timesteps, add a constraint which depends on the knot
  // value along with the state and input vectors at that knot and the
  // next.
  for (int i = 0; i < N() - 1; i++) {
    // Add the dynamic constraints.
    auto constraint = std::make_shared<DiscreteTimeSystemConstraint>(
        *system_, context_.get(), discrete_state_.get(), input_port_value_,
        num_states(), num_inputs(), i * fixed_timestep());

    AddConstraint(constraint, {input(i), state(i), state(i + 1)});
  }
}

void DirectTranscription::ConstrainEqualInputAtFinalTwoTimesteps() {
  if (num_inputs() > 0) {
    AddLinearEqualityConstraint(input(N() - 2) == input(N() - 1));
  }
}

void DirectTranscription::ValidateSystem(const System<double>& system,
                                         const Context<double>& context,
                                         int input_port_index) {
  DRAKE_DEMAND(context.has_only_discrete_state());
  DRAKE_DEMAND(context.get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(num_states() == context.get_discrete_state(0).size());
  DRAKE_DEMAND(num_inputs() == (context.get_num_input_ports() > 0
                                ? system.get_input_port(input_port_index).size()
                                : 0));
}

void DirectTranscription::ValidateSystem(const System<double>& system,
                                         const Context<double>& context) {
  DRAKE_DEMAND(context.has_only_discrete_state());
  DRAKE_DEMAND(context.get_num_discrete_state_groups() == 1);
  DRAKE_DEMAND(num_states() == context.get_discrete_state(0).size());
  DRAKE_DEMAND(system.get_num_input_ports() <= 1);
  DRAKE_DEMAND(num_inputs() == (context.get_num_input_ports() > 0
                                ? system.get_input_port(0).size()
                                : 0));
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
