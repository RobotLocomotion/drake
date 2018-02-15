#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// This class implements a controller for a Schunk WSG gripper as a
/// `systems::Diagram`. The composition of this diagram is determined by the
/// ControlMode specified for the controller.
class SchunkWsgPositionController
    : public systems::Diagram<double>,
      public systems::controllers::StateFeedbackControllerInterface<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPositionController)
  explicit SchunkWsgPositionController();

  const systems::InputPortDescriptor<double>& get_max_force_input_port() const {
    return this->get_input_port(max_force_input_port_);
  }

  // Implement StateFeedbackControllerInterface
  virtual const systems::InputPortDescriptor<double>&
  get_input_port_estimated_state() const override {
    return this->get_input_port(estimated_state_input_port_);
  }

  virtual const systems::InputPortDescriptor<double>&
  get_input_port_desired_state() const override {
    return this->get_input_port(desired_state_input_port_);
  }

  virtual const systems::OutputPort<double>& get_output_port_control()
      const override {
    return get_output_port(0);
  }

 private:
  int desired_state_input_port_{-1};
  int estimated_state_input_port_{-1};
  int max_force_input_port_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
