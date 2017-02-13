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
AffineSystem<T>::AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::VectorXd>& f0,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              const Eigen::Ref<const Eigen::VectorXd>& y0,
                              double time_period)
    : A_(A),
      B_(B),
      f0_(f0),
      C_(C),
      D_(D),
      y0_(y0),
      num_inputs_(D.cols()),
      num_outputs_(D.rows()),
      num_states_(f0.size()),
      time_period_(time_period) {
  DRAKE_DEMAND(num_states_ == A.rows());
  DRAKE_DEMAND(num_states_ == A.cols());
  DRAKE_DEMAND(num_states_ == B.rows());
  DRAKE_DEMAND(num_states_ == C.cols());
  DRAKE_DEMAND(num_inputs_ == B.cols());
  DRAKE_DEMAND(num_inputs_ == D.cols());
  DRAKE_DEMAND(num_outputs_ == C.rows());
  DRAKE_DEMAND(num_outputs_ == D.rows());
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Declares input port for u.
  if (num_inputs_ > 0) this->DeclareInputPort(kVectorValued, num_inputs_);

  // Declares output port for y.
  if (num_outputs_ > 0) this->DeclareOutputPort(kVectorValued, num_outputs_);

  // Declares the state variables and (potentially) the discrete-time update.
  if (time_period == 0.0) {
    this->DeclareContinuousState(num_states_);
  } else {
    this->DeclareContinuousState(0);
    this->DeclareDiscreteState(num_states_);
    this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
  }
}

// Setup equivalent system with a different scalar type.
template <typename T>
AffineSystem<AutoDiffXd>* AffineSystem<T>::DoToAutoDiffXd() const {
  return new AffineSystem<AutoDiffXd>(A_, B_, f0_, C_, D_, y0_, time_period_);
}

template <typename T>
const InputPortDescriptor<T>& AffineSystem<T>::get_input_port() const {
  DRAKE_DEMAND(num_inputs_ > 0);
  return System<T>::get_input_port(0);
}

template <typename T>
const OutputPortDescriptor<T>& AffineSystem<T>::get_output_port() const {
  DRAKE_DEMAND(num_outputs_ > 0);
  return System<T>::get_output_port(0);
}

template <typename T>
void AffineSystem<T>::DoCalcOutput(const Context<T>& context,
                                   SystemOutput<T>* output) const {
  if (num_outputs_ == 0) return;

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  auto y = output_vector->get_mutable_value();
  y = C_ * x + y0_;

  if (num_inputs_) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();
    y += D_ * u;
  }
}

template <typename T>
void AffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (num_states_ == 0 || time_period_ > 0.0) return;

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  VectorX<T> xdot = A_ * x + f0_;

  if (num_inputs_ > 0) {
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
  if (num_states_ == 0 || time_period_ == 0.0) return;

  const auto& x = context.get_discrete_state(0)->get_value();

  VectorX<T> xnext = A_ * x + f0_;

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    xnext += B_ * u;
  }
  updates->get_mutable_discrete_state(0)->SetFromVector(xnext);
}

template class AffineSystem<double>;
template class AffineSystem<AutoDiffXd>;

template <typename T>
TimeVaryingAffineSystem<T>::TimeVaryingAffineSystem(int num_states,
                                                    int num_inputs,
                                                    int num_outputs)
    : num_states_(num_states),
      num_inputs_(num_inputs),
      num_outputs_(num_outputs) {
  // Declare state and input/output ports.
  this->DeclareContinuousState(num_states_);
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

  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  T t = context.get_time();

  VectorX<T> y = y0(t);
  DRAKE_DEMAND(y.rows() == num_outputs_);

  if (num_states_ > 0) {
    MatrixX<T> Ct = C(t);
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
    MatrixX<T> Dt = D(t);
    DRAKE_DEMAND(Dt.rows() == num_outputs_ && Dt.cols() == num_inputs_);
    y += Dt * u;
  }

  output_vector->SetFromVector(y);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (num_states_ == 0) return;
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  T t = context.get_time();

  VectorX<T> xdot = f0(t);
  DRAKE_DEMAND(xdot.rows() == num_states_);

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();
  MatrixX<T> At = A(t);
  DRAKE_DEMAND(At.rows() == num_states_ && At.cols() == num_states_);
  xdot += At * x;

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    MatrixX<T> Bt = B(t);
    DRAKE_DEMAND(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
    xdot += Bt * u;
  }
  derivatives->SetFromVector(xdot);
}

template class TimeVaryingAffineSystem<double>;
template class TimeVaryingAffineSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
