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
                  integrator_gain_->get_input_port(0));
  builder.Connect(integrator_gain_->get_output_port(0),
                  integrator_->get_input_port(0));
  builder.Connect(proportional_gain_->get_output_port(0),
                  adder_->get_input_port(0));
  builder.Connect(integrator_->get_output_port(0),
                  adder_->get_input_port(1));
  builder.ExportOutput(adder_->get_output_port(0));

  builder.BuildInto(this);
}

template <typename T>
bool PidController<T>::has_any_direct_feedthrough() const {
  return true;
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<double>;

}  // namespace systems
}  // namespace drake
