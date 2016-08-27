#include "drake/systems/framework/examples/controlled_spring_mass_system.h"

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidControlledSpringMassSystem<T>::PidControlledSpringMassSystem(
    const T& spring_stiffness, const T& mass,
    const T& Kp, const T& Ki, const T& Kd) : Diagram<T>() {
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);
  DRAKE_ASSERT(length > 0);

  plant_ = make_unique<SpringMassSystem>(
      spring_stiffness, mass, true /* is forced */);
  controller_ = make_unique<PidController<T>>(Kp, Ki, Kd, 1);


  DiagramBuilder<T> builder;
  builder.Connect(plant_->get_output_port(0),
                  controller_->get_error_signal_port());

  builder.Connect(pass_through_->get_output_port(0),
                  integrator_->get_input_port(0));
  builder.Connect(integrator_->get_output_port(0),
                  integral_gain_->get_input_port(0));
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
bool PidControlledSpringMassSystem<T>::has_any_direct_feedthrough() const {
  return true;
}

template <typename T>
const SystemPortDescriptor<T>& PidControlledSpringMassSystem<T>::get_error_signal_port() const {
  return Diagram<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>&
PidControlledSpringMassSystem<T>::get_error_signal_rate_port() const {
  return Diagram<T>::get_input_port(1);
}

template <typename T>
void PidControlledSpringMassSystem<T>::SetDefaultState(ContextBase<T>* context) const {
  const int length = Diagram<T>::get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(length));
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_integral_value(
    ContextBase<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  ContextBase<T>* integrator_context =
      Diagram<T>::GetMutableSubSystemContext(context, integrator_.get());
  integrator_->set_integral_value(integrator_context, value);
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidControlledSpringMassSystem<double>;

}  // namespace systems
}  // namespace drake
