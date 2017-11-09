#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

enum class ControlMode {
  kPosition = 0,
  kForce = 1,
  kPositionAndForce = 2,
};

/// This class implements a controller for a Schunk WSG gripper.  It
/// has two input ports which receive lcmt_schunk_wsg_command messages
/// and the current state, and an output port which emits the target
/// force for the actuated finger.  The internal implementation
/// consists of a PID controller (which controls the target position
/// from the command message) combined with a saturation block (which
/// applies the force control from the command message).
class SchunkWsgPlainController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPlainController)
  explicit SchunkWsgPlainController(
      ControlMode control_mode = ControlMode::kPosition);

  const systems::InputPortDescriptor<double>& get_desired_state_input_port()
      const {
    return this->get_input_port(desired_state_input_port_);
  }

  const systems::InputPortDescriptor<double>&
  get_feed_forward_force_input_port() const {
    return this->get_input_port(feed_forward_force_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_max_force_input_port() const {
    return this->get_input_port(max_force_input_port_);
  }

 private:
  int desired_state_input_port_{-1};
  int feed_forward_force_input_port_{-1};
  int state_input_port_{-1};
  int max_force_input_port_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
