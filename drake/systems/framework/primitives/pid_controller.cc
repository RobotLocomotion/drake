#include "drake/systems/framework/primitives/pid_controller.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidController<T>::PidController(
    const T& Kp, const T& Ki, const T& Kd, int size) : Diagram<T>() {
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);
  DRAKE_ASSERT(size > 0);

  DiagramBuilder<T> builder;
  pass_through_ = builder.AddSystem(make_unique<PassThrough<T>>(size));
  proportional_gain_ = builder.AddSystem(make_unique<Gain<T>>(Kp, size));
  integral_gain_ = builder.AddSystem(make_unique<Gain<T>>(Ki, size));
  derivative_gain_ = builder.AddSystem(make_unique<Gain<T>>(Kd, size));
  integrator_ = builder.AddSystem(make_unique<Integrator<T>>(size));
  adder_ = builder.AddSystem(make_unique<Adder<T>>(3 /* inputs */, size));

  // Input 0 connects to the proportional and integral components.
  builder.ExportInput(pass_through_->get_input_port(0));
  // Input 1 connects directly to the derivative component.
  builder.ExportInput(derivative_gain_->get_input_port());
  builder.Connect(*pass_through_, *proportional_gain_);
  builder.Connect(*pass_through_, *integrator_);
  builder.Connect(*integrator_, *integral_gain_);
  builder.Connect(proportional_gain_->get_output_port(),
                  adder_->get_input_port(0));
  builder.Connect(integral_gain_->get_output_port(),
                  adder_->get_input_port(1));
  builder.Connect(derivative_gain_->get_output_port(),
                  adder_->get_input_port(2));
  builder.ExportOutput(adder_->get_output_port());
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
const SystemPortDescriptor<T>&
PidController<T>::get_control_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void PidController<T>::SetDefaultState(Context<T>* context) const {
  const int size = Diagram<T>::get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(size));
}

template <typename T>
void PidController<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  Context<T>* integrator_context =
      Diagram<T>::GetMutableSubsystemContext(context, integrator_);
  integrator_->set_integral_value(integrator_context, value);
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
