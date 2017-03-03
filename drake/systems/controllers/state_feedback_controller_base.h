#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace systems {

/**
 * Interface for state feedback controllers. This class needs to be extended by
 * concrete implementations. It provides named accessors to actual and desired
 * state input ports and control output port.
 */
template<typename T>
class StateFeedbackController : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateFeedbackController)

  /**
   * Returns the input port for the estimated state.
   */
  const InputPortDescriptor<T>& get_input_port_estimated_state() const {
    return Diagram<T>::get_input_port(0);
  }

  /**
   * Returns the input port for the desired state.
   */
  const InputPortDescriptor<T>& get_input_port_desired_state() const {
    return Diagram<T>::get_input_port(1);
  }

  /**
   * Returns the output port for computed control.
   */
  const OutputPortDescriptor<T>& get_output_port_control() const {
    return Diagram<T>::get_output_port(0);
  }

 protected:
  StateFeedbackController() {}
};

}  // namespace systems
}  // namespace drake
