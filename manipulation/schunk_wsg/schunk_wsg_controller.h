#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// This class implements a controller for a Schunk WSG gripper.  It
/// has two input ports which receive lcmt_schunk_wsg_command messages
/// and the current state, and an output port which emits the target
/// force for the actuated finger.  The internal implementation
/// consists of a PID controller (which controls the target position
/// from the command message) combined with a saturation block (which
/// applies the force control from the command message).
class SchunkWsgController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgController)

  // The gains here are somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  explicit SchunkWsgController(double kp = 2000.0, double ki = 0.0,
                               double kd = 5.0);

  const systems::InputPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(command_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

 private:
  int command_input_port_{};
  int state_input_port_{};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
