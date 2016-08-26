#include "drake/systems/framework/primitives/pid_controller.h"

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidController<T>::PidController(
    const T& Kp, const T& Ki, int length) : Diagram<T>() {
  pass_through_ = make_unique<PassThrough<T>>(length);
  proportional_gain_ = make_unique<Gain<T>>(Kp /* gain */, length /* length */);
  integrator_gain_ = make_unique<Gain<T>>(Ki /* gain */, length /* length */);
  integrator_ = make_unique<Integrator<T>>(length);
  adder_ = make_unique<Adder<T>>(2 /* inputs */, length /* length */);

  DiagramBuilder<T> builder;
  builder.ExportInput(pass_through_->get_input_port(0));
  builder.Connect(pass_through_->get_output_port(0),
                  proportional_gain_->get_input_port(0));
  builder.Connect(pass_through_->get_output_port(0),
                  integrator_->get_input_port(0));
  builder.Connect(integrator_->get_output_port(0),
                  integrator_gain_->get_input_port(0));
  builder.Connect(proportional_gain_->get_output_port(0),
                  adder_->get_input_port(0));
  builder.Connect(integrator_gain_->get_output_port(0),
                  adder_->get_input_port(1));
  builder.ExportOutput(adder_->get_output_port(0));
  builder.BuildInto(this);
}

template <typename T>
bool PidController<T>::has_any_direct_feedthrough() const {
  return true;
}

template <typename T>
void PidController<T>::SetDefaultState(ContextBase<T>* context) const {
  const int length = Diagram<T>::get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(length));
}

template <typename T>
void PidController<T>::set_integral_value(
    ContextBase<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  auto diagram_context = dynamic_cast<DiagramContext<T>*>(context);
  ContextBase<T>* integrator_context =
      diagram_context->GetMutableSubsystemContext(
          Diagram<T>::GetSystemIndex(integrator_.get()));
  integrator_->set_integral_value(integrator_context, value);
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<double>;

}  // namespace systems
}  // namespace drake
