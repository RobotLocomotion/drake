#include "drake/systems/framework/primitives/pid_controller2.h"

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_state_vector.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidController<T>::PidController(
    const T& Kp, const T& Ki, const T& Kd, int length) :
    kp_(Kp), ki_(Ki), kd_(Kd) {
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);
  DRAKE_ASSERT(length > 0);
  // Input port "0" is for the error signal.
  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  // Input port "1" is for the error signal's rate.
  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  // The output is the control signal.
  this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);
}

template <typename T>
bool PidController<T>::has_any_direct_feedthrough() const {
  return true;
}

template <typename T>
const SystemPortDescriptor<T>& PidController<T>::get_error_signal_port() const {
  return this->get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>&
PidController<T>::get_error_signal_rate_port() const {
  return this->get_input_port(1);
}

template <typename T>
void PidController<T>::SetDefaultState(ContextBase<T>* context) const {
  const int length = this->get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(length));
}

template <typename T>
void PidController<T>::set_integral_value(
    ContextBase<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  // TODO(amcastro-tri): Provide simple accessors here to avoid lengthy
  // constructions.
  auto state_vector =
      context->get_mutable_state()->continuous_state->get_mutable_state();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(value.rows() == state_vector->size() && value.cols() == 1);
  context->get_mutable_state()->continuous_state->
      get_mutable_state()->SetFromVector(value);
}

template <typename T>
std::unique_ptr<ContinuousState<T>> PidController<T>::AllocateContinuousState()
const {
  // The PID controller's state contains the current value of the integral part.
  // The integrator's state is first-order; its state vector length is the
  // same as the input (and output) vector length.
  const int length = System<T>::get_output_port(0).get_size();
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == length);
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(length));
}

template <typename T>
void PidController<T>::EvalTimeDerivatives(const ContextBase<T>& context,
                                           ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  const VectorBase<T>* input = context.get_vector_input(0);
  derivatives->get_mutable_state()->SetFromVector(input->get_value());
}

template <typename T>
void PidController<T>::EvalOutput(const ContextBase<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  auto error = context.get_vector_input(0)->get_value();
  auto error_rate = context.get_vector_input(1)->get_value();
  auto integral = this->CopyContinuousStateVector(context);
  auto control = kp_ * error + ki_ * integral + kd_ * error_rate;
  output->GetMutableVectorData(0)->set_value(control);
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<double>;

}  // namespace systems
}  // namespace drake
