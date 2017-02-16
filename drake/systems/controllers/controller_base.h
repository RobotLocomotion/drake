#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace systems {

template<typename T>
class Controller : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Controller)

  const InputPortDescriptor<T>& get_measured_state_input_port() const {
    return Diagram<T>::get_input_port(0);
  }

  const InputPortDescriptor<T>& get_desired_state_input_port() const {
    return Diagram<T>::get_input_port(1);
  }

  const OutputPortDescriptor<T>& get_control_output_port() const {
    return Diagram<T>::get_output_port(0);
  }

 protected:
  Controller() {}
};

}  // namespace systems
}  // namespace drake
