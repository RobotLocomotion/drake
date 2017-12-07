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

namespace {

class DiscreteTimeSystemConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeSystemConstraint)

  // @param evaluation_time  The time along the trajectory at which this
  // constraint is evaluated.
  DiscreteTimeSystemConstraint(const System<AutoDiffXd>& system,
                               Context<AutoDiffXd>* context,
                               DiscreteValues<AutoDiffXd>* discrete_state,
                               FreestandingInputPortValue* input_port_value,
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
              Eigen::VectorXd& y) const override {
    AutoDiffVecXd y_t;
    Eval(math::initializeAutoDiff(x), y_t);
    y = math::autoDiffToValueMatrix(y_t);
  }

  // The format of the input to the eval() function is a vector
  // containing {input, state, next_state}.
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
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
    y = next_state - discrete_state_->get_vector(0).CopyToVector();
  }

 private:
  const System<AutoDiffXd>& system_;
  Context<AutoDiffXd>* const context_;
  FreestandingInputPortValue* const input_port_value_;
  DiscreteValues<AutoDiffXd>* const discrete_state_;

  const int num_states_{0};
  const int num_inputs_{0};
  AutoDiffXd evaluation_time_{0};
};

}  // end namespace

DirectTranscription::DirectTranscription(const System<double>* system,
                                         const Context<double>& context,
                                         int num_time_samples)
    : MultipleShooting(system->get_num_total_inputs(),
                       context.get_num_total_states(), num_time_samples,
                       0.1),  // TODO(russt): Replace this with the actual
                              // sample time of the discrete update (#6878).
      discrete_time_system_(true) {
  // Note: this constructor is for discrete-time systems.  For continuous-time
  // systems, you must use a different constructor that specifies the timesteps.
  ValidateSystem(*system, context);

  // First try symbolic dynamics.
  if (!AddSymbolicDynamicConstraints(system, context)) {
    AddAutodiffDynamicConstraints(system, context);
  }
  ConstrainEqualInputAtFinalTwoTimesteps();
}

DirectTranscription::DirectTranscription(const LinearSystem<double>* system,
                                         const Context<double>& context,
                                         int num_time_samples)
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
    AddLinearEqualityConstraint(
        state(i+1).cast<symbolic::Expression>() ==
        system->A() * state(i).cast<symbolic::Expression>() +
        system->B() * input(i).cast<symbolic::Expression>());
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

PiecewisePolynomialTrajectory DirectTranscription::ReconstructInputTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  // TODO(russt): Implement DTTrajectories and return one of those instead.
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::ZeroOrderHold(times_vec, inputs));
}

PiecewisePolynomialTrajectory DirectTranscription::ReconstructStateTrajectory()
    const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
  }
  // TODO(russt): Implement DTTrajectories and return one of those instead.
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::ZeroOrderHold(times_vec, states));
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
  for (int i = 0; i < context.num_numeric_parameters(); i++) {
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
    const System<double>* system, const Context<double>& context) {
  system_ = system->ToAutoDiffXd();
  DRAKE_DEMAND(system_ != nullptr);
  context_ = system_->CreateDefaultContext();
  discrete_state_ = system_->AllocateDiscreteVariables();

  context_->SetTimeStateAndParametersFrom(context);

  if (context_->get_num_input_ports() > 0) {
    // Allocate the input port and keep an alias around.
    input_port_value_ = new FreestandingInputPortValue(
        system_->AllocateInputVector(system_->get_input_port(0)));
    std::unique_ptr<InputPortValue> input_port_value(input_port_value_);
    context_->SetInputPortValue(0, std::move(input_port_value));
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
                                         const Context<double>& context) {
  DRAKE_THROW_UNLESS(context.has_only_discrete_state());

  // TODO(russt): Check that the system has ONLY simple periodic updates
  // (#6878).

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
