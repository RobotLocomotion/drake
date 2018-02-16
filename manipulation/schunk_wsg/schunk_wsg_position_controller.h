#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/**
 * This class implements force-limited PID controller for a single degree of
 * freedom gripper.
 * ```
 *                    ┌─────────────┐            ┌──────────┐
 * estimated state ──▶│             │ ┌─────────▶│          │
 *                    │PidController├───────────▶│Saturation├────▶ control
 * desired state ────▶│             │ │       ┌─▶│          │
 *                    └─────────────┘ │       │  └──────────┘
 *                    ┌─────────────┐ │  ┌──┐ │
 * max force ────────▶│PassThough   ├─┴─▶│-1├─┘
 *                    └─────────────┘    └──┘
 * ```
 */
template <typename T>
class SchunkWsgPositionController
    : public systems::Diagram<T>,
      public systems::controllers::StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPositionController)
  explicit SchunkWsgPositionController();

  const systems::InputPortDescriptor<T>& get_max_force_input_port() const {
    return this->get_input_port(max_force_input_port_);
  }

  // Implement StateFeedbackControllerInterface
  virtual const systems::InputPortDescriptor<T>&
  get_input_port_estimated_state() const override {
    return this->get_input_port(estimated_state_input_port_);
  }

  virtual const systems::InputPortDescriptor<T>&
  get_input_port_desired_state() const override {
    return this->get_input_port(desired_state_input_port_);
  }

  virtual const systems::OutputPort<T>& get_output_port_control()
      const override {
    return this->get_output_port(0);
  }

 private:
  int desired_state_input_port_{-1};
  int estimated_state_input_port_{-1};
  int max_force_input_port_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
