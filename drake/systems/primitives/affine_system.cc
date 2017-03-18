#include "drake/systems/primitives/affine_system.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

using std::make_unique;

template <typename T>
TimeVaryingAffineSystem<T>::TimeVaryingAffineSystem(int num_states,
                                                    int num_inputs,
                                                    int num_outputs,
                                                    double time_period)
    : num_states_(num_states),
      num_inputs_(num_inputs),
      num_outputs_(num_outputs),
      time_period_(time_period) {
  DRAKE_DEMAND(num_states_ >= 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  DRAKE_DEMAND(num_outputs_ >= 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Declare state and input/output ports.
  // Declares the state variables and (potentially) the discrete-time update.
  if (time_period_ == 0.0) {
    this->DeclareContinuousState(num_states_);
  } else {
    this->DeclareContinuousState(0);
    this->DeclareDiscreteState(num_states_);
    this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
  }
  if (num_inputs_ > 0) this->DeclareInputPort(kVectorValued, num_inputs_);
  if (num_outputs_ > 0) this->DeclareOutputPort(kVectorValued, num_outputs_);
}

template <typename T>
const InputPortDescriptor<T>& TimeVaryingAffineSystem<T>::get_input_port()
    const {
  DRAKE_DEMAND(num_inputs_ > 0);
  return System<T>::get_input_port(0);
}

template <typename T>
const OutputPortDescriptor<T>& TimeVaryingAffineSystem<T>::get_output_port()
    const {
  DRAKE_DEMAND(num_outputs_ > 0);
  return System<T>::get_output_port(0);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcOutput(const Context<T>& context,
                                              SystemOutput<T>* output) const {
  if (num_outputs_ == 0) return;

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  const T t = context.get_time();

  VectorX<T> y = y0(t);
  DRAKE_DEMAND(y.rows() == num_outputs_);

  if (num_states_ > 0) {
    const MatrixX<T> Ct = C(t);
    DRAKE_DEMAND(Ct.rows() == num_outputs_ && Ct.cols() == num_states_);
    const auto& x = dynamic_cast<const BasicVector<T>&>(
                        context.get_continuous_state_vector())
                        .get_value();
    y += Ct * x;
  }

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();
    const MatrixX<T> Dt = D(t);
    DRAKE_DEMAND(Dt.rows() == num_outputs_ && Dt.cols() == num_inputs_);
    y += Dt * u;
  }

  output_vector->SetFromVector(y);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (num_states_ == 0) return;

  const T t = context.get_time();

  VectorX<T> xdot = f0(t);
  DRAKE_DEMAND(xdot.rows() == num_states_);

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();
  const MatrixX<T> At = A(t);
  DRAKE_DEMAND(At.rows() == num_states_ && At.cols() == num_states_);
  xdot += At * x;

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    const MatrixX<T> Bt = B(t);
    DRAKE_DEMAND(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
    xdot += Bt * u;
  }
  derivatives->SetFromVector(xdot);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    drake::systems::DiscreteState<T>* updates) const {
  if (num_states_ == 0 || time_period_ == 0.0) return;

  const T t = context.get_time();

  // TODO(russt): consider demanding that t is a multiple of time_period_.
  // But this could be non-trivial for non-double T.

  VectorX<T> xn = f0(t);
  DRAKE_DEMAND(xn.rows() == num_states_);

  const auto& x = context.get_discrete_state(0)->get_value();

  const MatrixX<T> At = A(t);
  DRAKE_DEMAND(At.rows() == num_states_ && At.cols() == num_states_);
  xn += At * x;

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    const MatrixX<T> Bt = B(t);
    DRAKE_DEMAND(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
    xn += Bt * u;
  }
  updates->get_mutable_discrete_state(0)->SetFromVector(xn);
}

template class TimeVaryingAffineSystem<double>;
template class TimeVaryingAffineSystem<AutoDiffXd>;

template <typename T>
AffineSystem<T>::AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::VectorXd>& f0,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              const Eigen::Ref<const Eigen::VectorXd>& y0,
                              double time_period)
    : TimeVaryingAffineSystem<T>(f0.size(), D.cols(), D.rows(), time_period),
      A_(A),
      B_(B),
      f0_(f0),
      C_(C),
      D_(D),
      y0_(y0) {
  DRAKE_DEMAND(this->num_states() == A.rows());
  DRAKE_DEMAND(this->num_states() == A.cols());
  DRAKE_DEMAND(this->num_states() == B.rows());
  DRAKE_DEMAND(this->num_states() == C.cols());
  DRAKE_DEMAND(this->num_inputs() == B.cols());
  DRAKE_DEMAND(this->num_inputs() == D.cols());
  DRAKE_DEMAND(this->num_outputs() == C.rows());
  DRAKE_DEMAND(this->num_outputs() == D.rows());
}

// Setup equivalent system with a different scalar type.
template <typename T>
AffineSystem<AutoDiffXd>* AffineSystem<T>::DoToAutoDiffXd() const {
  return new AffineSystem<AutoDiffXd>(A_, B_, f0_, C_, D_, y0_,
                                      this->time_period());
}

template <typename T>
void AffineSystem<T>::DoCalcOutput(const Context<T>& context,
                                   SystemOutput<T>* output) const {
  if (this->num_outputs() == 0) return;

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  auto y = output_vector->get_mutable_value();
  y = C_ * x + y0_;

  if (this->num_inputs()) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();
    y += D_ * u;
  }
}

template <typename T>
void AffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (this->num_states() == 0 || this->time_period() > 0.0) return;

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  VectorX<T> xdot = A_ * x + f0_;

  if (this->num_inputs() > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    xdot += B_ * u;
  }
  derivatives->SetFromVector(xdot);
}

template <typename T>
void AffineSystem<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    drake::systems::DiscreteState<T>* updates) const {
  if (this->num_states() == 0 || this->time_period() == 0.0) return;

  const auto& x = context.get_discrete_state(0)->get_value();

  VectorX<T> xnext = A_ * x + f0_;

  if (this->num_inputs() > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    xnext += B_ * u;
  }
  updates->get_mutable_discrete_state(0)->SetFromVector(xnext);
}

template class AffineSystem<double>;
template class AffineSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
