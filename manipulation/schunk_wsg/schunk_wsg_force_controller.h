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
class SchunkWsgForceController
    : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgForceController)
  explicit SchunkWsgForceController();

  const systems::InputPortDescriptor<double>&
  get_feed_forward_force_input_port() const {
    return this->get_input_port(feed_forward_force_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(estimated_state_input_port_);
  }

 private:
  int feed_forward_force_input_port_{-1};
  int estimated_state_input_port_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
