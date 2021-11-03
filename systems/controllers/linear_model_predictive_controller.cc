#include "drake/systems/controllers/linear_model_predictive_controller.h"

#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/solvers/solve.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

namespace drake {
namespace systems {
namespace controllers {

using solvers::VectorXDecisionVariable;
using trajectory_optimization::DirectTranscription;

template <typename T>
LinearModelPredictiveController<T>::LinearModelPredictiveController(
    std::unique_ptr<systems::System<double>> model,
    std::unique_ptr<systems::Context<double>> base_context,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, double time_period,
    double time_horizon)
    : state_input_index_(
          this->DeclareVectorInputPort(kUseDefaultName, Q.cols()).get_index()),
      control_output_index_(
          this->DeclareVectorOutputPort(
                  kUseDefaultName, R.cols(),
                  &LinearModelPredictiveController<T>::CalcControl)
              .get_index()),
      model_(std::move(model)),
      base_context_(std::move(base_context)),
      num_states_(model_->CreateDefaultContext()->get_discrete_state(0).size()),
      num_inputs_(model_->get_input_port(0).size()),
      Q_(Q),
      R_(R),
      time_period_(time_period),
      time_horizon_(time_horizon) {
  DRAKE_DEMAND(time_period_ > 0.);
  DRAKE_DEMAND(time_horizon_ > 0.);

  // Check that the model is SISO and has discrete states belonging to a single
  // group.
  const auto model_context = model_->CreateDefaultContext();
  DRAKE_DEMAND(model_context->num_discrete_state_groups() == 1);
  DRAKE_DEMAND(model_context->num_continuous_states() == 0);
  DRAKE_DEMAND(model_context->num_abstract_states() == 0);
  DRAKE_DEMAND(model_->num_input_ports() == 1);
  DRAKE_DEMAND(model_->num_output_ports() == 1);

  // Check that the provided  x0, u0, Q, R are consistent with the model.
  DRAKE_DEMAND(num_states_ > 0 && num_inputs_ > 0);
  DRAKE_DEMAND(Q.rows() == num_states_ && Q.cols() == num_states_);
  DRAKE_DEMAND(R.rows() == num_inputs_ && R.cols() == num_inputs_);

  // N.B. A Cholesky decomposition exists if and only if it is positive
  // semidefinite, however it turns out that Eigen's algorithm for checking this
  // is incomplete: it only succeeds on *strictly* positive definite
  // matrices. We exploit the fact here to check for strict
  // positive-definiteness of R.
  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  if (R_cholesky.info() != Eigen::Success) {
    throw std::runtime_error("R must be positive definite");
  }

  this->DeclarePeriodicDiscreteUpdate(time_period_);

  if (base_context_ != nullptr) {
    linear_model_ = Linearize(*model_, *base_context_);
  }
}

template <typename T>
void LinearModelPredictiveController<T>::CalcControl(
    const Context<T>& context, BasicVector<T>* control) const {
  const VectorX<T>& current_state = get_state_port().Eval(context);

  const Eigen::VectorXd current_input =
      SetupAndSolveQp(*base_context_, current_state);

  const VectorX<T> input_ref = model_->get_input_port(0).Eval(*base_context_);

  control->SetFromVector(current_input + input_ref);

  // TODO(jadecastro) Implement the time-varying case.
}

template <typename T>
VectorX<T> LinearModelPredictiveController<T>::SetupAndSolveQp(
    const Context<T>& base_context, const VectorX<T>& current_state) const {
  DRAKE_DEMAND(linear_model_ != nullptr);

  const int kNumSampleTimes =
      static_cast<int>(time_horizon_ / time_period_ + 0.5);

  DirectTranscription prog(linear_model_.get(), *base_context_,
                           kNumSampleTimes);

  const auto state_error = prog.state();
  const auto input_error = prog.input();

  prog.AddRunningCost(state_error.transpose() * Q_ * state_error +
                      input_error.transpose() * R_ * input_error);

  const VectorX<T> state_ref =
      base_context.get_discrete_state().get_vector().CopyToVector();
  prog.AddLinearConstraint(prog.initial_state() == current_state - state_ref);

  const auto result = Solve(prog);
  DRAKE_DEMAND(result.is_success());

  return prog.GetInputSamples(result).col(0);
}

template class LinearModelPredictiveController<double>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
