#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace systems {
namespace controllers {

/**
 * Interface for state feedback controllers. This class needs to be extended by
 * concrete implementations. It provides named accessors to actual and desired
 * state input ports and control output port.
 */
template <typename T>
class StateFeedbackControllerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateFeedbackControllerInterface)

  /**
   * Returns the input port for the estimated state.
   */
  virtual const InputPort<T>& get_input_port_estimated_state()
      const = 0;

  /**
   * Returns the input port for the desired state.
   */
  virtual const InputPort<T>& get_input_port_desired_state()
      const = 0;

  /**
   * Returns the output port for computed control.
   */
  virtual const OutputPort<T>& get_output_port_control() const = 0;

 protected:
  StateFeedbackControllerInterface() {}
  virtual ~StateFeedbackControllerInterface() {}
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
