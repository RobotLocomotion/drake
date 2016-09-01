#include "drake/systems/framework/primitives/pid_controller.h"

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidController<T>::PidController(
    const T& Kp, const T& Ki, const T& Kd, int length) : Diagram<T>() {
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);
  DRAKE_ASSERT(length > 0);
  pass_through_ = make_unique<PassThrough<T>>(length);
  proportional_gain_ = make_unique<Gain<T>>(Kp /* gain */, length /* length */);
  integral_gain_ = make_unique<Gain<T>>(Ki /* gain */, length /* length */);
  derivative_gain_ = make_unique<Gain<T>>(Kd /* gain */, length /* length */);
  integrator_ = make_unique<Integrator<T>>(length);
  adder_ = make_unique<Adder<T>>(3 /* inputs */, length /* length */);

  DiagramBuilder<T> builder;
  // Input 0 connects to the proportional and integral components.
  builder.ExportInput(pass_through_->get_input_port(0));
  // Input 1 connects directly to the derivative component.
  builder.ExportInput(derivative_gain_->get_input_port(0));
  builder.Connect(*pass_through_, *proportional_gain_);
  builder.Connect(*pass_through_, *integrator_);
  builder.Connect(*integrator_, *integral_gain_);
  builder.Connect(proportional_gain_->get_output_port(0),
                  adder_->get_input_port(0));
  builder.Connect(integral_gain_->get_output_port(0),
                  adder_->get_input_port(1));
  builder.Connect(derivative_gain_->get_output_port(0),
                  adder_->get_input_port(2));
  builder.ExportOutput(adder_->get_output_port(0));
  builder.BuildInto(this);
}

template <typename T>
T PidController<T>::get_Kp() const {
  return proportional_gain_->get_gain();
}

template <typename T>
T PidController<T>::get_Ki() const {
  return integral_gain_->get_gain();
}

template <typename T>
T PidController<T>::get_Kd() const {
  return derivative_gain_->get_gain();
}

template <typename T>
bool PidController<T>::has_any_direct_feedthrough() const {
  if (get_Kp() == 0 && get_Kd() == 0) return false;
  return true;
}

template <typename T>
const SystemPortDescriptor<T>& PidController<T>::get_error_port() const {
  return Diagram<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>&
PidController<T>::get_error_derivative_port() const {
  return Diagram<T>::get_input_port(1);
}

template <typename T>
void PidController<T>::SetDefaultState(ContextBase<T>* context) const {
  const int length = Diagram<T>::get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(length));
}

template <typename T>
void PidController<T>::set_integral_value(
    ContextBase<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  ContextBase<T>* integrator_context =
      Diagram<T>::GetMutableSubsystemContext(context, integrator_.get());
  integrator_->set_integral_value(integrator_context, value);
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<double>;

}  // namespace systems
}  // namespace drake
