#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
;

/** This system coordinates the fingers of a parallel-jaw gripper.
 * When attached to a plant with two degrees-of-freedom (position of each
 * finger), the resulting combined system acts like a plant with a single
 * degree-of-freeedom (gripper width).
 * ```
 *                                 ┌──────────────┐
 * commanded                       │JointState    │                estimated
 * grip force ───────────┐      ┌─▶│ToGripState   ├──────────────▶ grip state
 *                       │      │  └──────────────┘
 *                       │      │  ┌─────────────┐
 *                       │      │  │GripForce    │     ┌─────┐
 *                       └────────▶│ToJointForce ├────▶│     │
 *                              │  └─────────────┘     │     │
 *                ┌───────────┐ │                      │     │
 * estimated      │Pass       │ │  ┌─────────────┐     │Adder├───▶ commanded
 * joint state ──▶│Through    ├─┴─▶│             │     │     │     joint force
 *                └───────────┘    │             │     │     │
 *                ┌───────────┐    │PidController├────▶│     │
 *                │DesiredMean│    │             │     └─────┘
 *                │Finger     ├───▶│             │
 *                │Position   │    └─────────────┘
 *                └───────────┘
 * ```
 */
class SchunkWsgLowLevelController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgLowLevelController)
  explicit SchunkWsgLowLevelController(
      const Vector2<double>& closed_joint_position = Vector2<double>::Zero(),
      const Vector2<double>& open_joint_position = Vector2<double>(1, -1));

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
