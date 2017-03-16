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
    DRAKE_DEMAND(input_port_index_estimated_state_ >= 0);
    return Diagram<T>::get_input_port(input_port_index_estimated_state_);
  }

  /**
   * Returns the input port for the desired state.
   */
  const InputPortDescriptor<T>& get_input_port_desired_state() const {
    DRAKE_DEMAND(input_port_index_desired_state_ >= 0);
    return Diagram<T>::get_input_port(input_port_index_desired_state_);
  }

  /**
   * Returns the output port for computed control.
   */
  const OutputPortDescriptor<T>& get_output_port_control() const {
    DRAKE_DEMAND(output_port_index_control_ >= 0);
    return Diagram<T>::get_output_port(output_port_index_control_);
  }

 protected:
  void set_input_port_index_estimated_state(int index) {
    input_port_index_estimated_state_ = index;
  }

  void set_input_port_index_desired_state(int index) {
    input_port_index_desired_state_ = index;
  }

  void set_output_port_index_control(int index) {
    output_port_index_control_ = index;
  }

  StateFeedbackController() {}
  int input_port_index_estimated_state_{-1};
  int input_port_index_desired_state_{-1};
  int output_port_index_control_{-1};
};

}  // namespace systems
}  // namespace drake
