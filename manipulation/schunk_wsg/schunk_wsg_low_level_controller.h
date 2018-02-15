#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
;

/// This class implements a controller for a Schunk WSG gripper as a
/// `systems::Diagram`. The composition of this diagram is determined by the
/// ControlMode specified for the controller.
class SchunkWsgLowLevelController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgLowLevelController)
  explicit SchunkWsgLowLevelController();

  const systems::InputPortDescriptor<double>&
  get_commanded_grip_force_input_port() const {
    return this->get_input_port(commanded_grip_force_input_port_);
  }

  const systems::InputPortDescriptor<double>&
  get_estimated_joint_state_input_port() const {
    return this->get_input_port(estimated_joint_state_input_port_);
  }

  const systems::OutputPort<double>& get_comanded_joint_force_output_port()
      const {
    return this->get_output_port(commanded_joint_force_output_port_);
  }

  const systems::OutputPort<double>& get_estimated_grip_state_output_port()
      const {
    return this->get_output_port(estimated_grip_state_output_port_);
  }

 private:
  // Input ports.
  int commanded_grip_force_input_port_{-1};
  int estimated_joint_state_input_port_{-1};
  // Output ports.
  int commanded_joint_force_output_port_{-1};
  int estimated_grip_state_output_port_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
