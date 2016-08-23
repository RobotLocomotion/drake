#include "drake/systems/framework/primitives/pid_controller.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

using std::make_unique;

namespace drake {
namespace systems {

template<typename T>
Diagram<T> MakePidController(const T& Kp, const T& Ki, int length) {
  DiagramBuilder<T> builder;

  auto pass_through = make_unique<PassThrough>();
  auto proportional_gain = make_unique<Gain<T>>(Kp /* gain */, length /* length */);
  auto integrator_gain = make_unique<Gain<T>>(Ki /* gain */, length /* length */);
  auto integrator = make_unique<Integrator<T>>(length);
  auto adder = make_unique<Adder<T>>(2 /* inputs */, length /* length */);

  builder.ExportInput(pass_through->get_input_port(0));

  // The PassThrough system is used so that we can connect the two gain systems
  // to the Diagram input.
  builder.Connect(pass_through->get_output_port(0),
                  proportional_gain->get_input_port(0));
  builder.Connect(pass_through->get_output_port(0),
                  integrator_gain->get_input_port(0));

  builder.Connect(integrator_gain->get_output_port(0),
                  integrator->get_input_port(0));

  builder.Connect(proportional_gain->get_input_port(0),
                  adder->get_input_port(0));
  builder.Connect(integrator->get_input_port(0),
                  adder->get_input_port(1));

  builder.ExportOutput(adder->get_output_port(0));

  return builder.Build();
}

template <typename T>
PidController<T>::PidController(const T& Kp, const T& Ki, int length)
    : length_(length) {
  pass_through_ = make_unique<PassThrough>();
  proportional_gain_ = make_unique<Gain<T>>(Kp /* gain */, length /* length */);
  integrator_gain_ = make_unique<Gain<T>>(Ki /* gain */, length /* length */);
  integrator_ = make_unique<Integrator<T>>(length);
  adder_ = make_unique<Adder<T>>(2 /* inputs */, length /* length */);


  this->ExportInput(pass_through_.get(), 0);



  this->ExportOutput(adder_.get(), 0);
}

template class DRAKESYSTEMFRAMEWORK_EXPORT PidController<double>;

}  // namespace systems
}  // namespace drake
