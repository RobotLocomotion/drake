#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

enum class ControlMode { kPosition = 0, kForce = 1 };

/** This class implements a controller for a Schunk WSG gripper as a
 * `systems::Diagram`. The composition of this diagram is determined by the
 * control mode specified for the controller, which can be either
 * ControlMode::kPosition or ControlMode::kForce. In both cases, the overall
 * layout of the diagram is:
 *```
 *             ┌─────────────┐
 * joint       │Joint State  │   ┌──────────┐
 * state ─────▶│To Control   ├──▶│          │
 *             │State        │   │          │
 *             └─────────────┘   │PID       │   ╔════════════╗
 *             ╔═════════════╗   │Controller├──▶║            ╟─────┐
 * desired     ║Generate     ║   │          │   ║            ║     │
 * grip ──────▶║Desired      ╟──▶│          │   ║Handle      ║     │
 * state       ║Control State║   └──────────┘   ║Feed-Forward║     │
 *             ╚═════════════╝                  ║Force       ║     │
 * feed                                         ║            ║     │
 * forward ────────────────────────────────────▶║            ╟──┐  │
 * force                                        ╚════════════╝  │  │
 *                                                              │  │
 *                           ┌──────────────────────────────────┘  │
 *                           │              ┌──────────────────────┘
 *                           │              │
 *                           │              │   ┌───────────┐
 *                           │              │   │Mean Finger│   ┌───┐
 *                           │              └──▶│Force To   ├──▶│   │
 *                           │                  │Joint Force│   │   │
 *                           │                  └───────────┘   │   │
 *                           │                                  │ + ├──▶ control
 *                           │   ┌──────────┐   ┌───────────┐   │   │
 *                 ┌─────────│──▶│          │   │Grip Force │   │   │
 *                 │   ┌──┐  └──▶│Saturation├──▶│To Joint   ├──▶│   │
 * max force ──────┴──▶│-1├─────▶│          │   │Force      │   └───┘
 *                     └──┘      └──────────┘   └───────────┘
 *```
 * The blocks with double outlines (══) differ between the two control modes:
 *  - Generate Desired Control State
 *    - ControlMode::kPosition
 *```
 *        ┌───────────┐
 *        │Desired    │
 *        │Mean Finger├──▶█
 *        │State      │   █   ┌─────────────┐
 *        └───────────┘   █   │Muxed States │    desired
 *                        █──▶│To Control   ├──▶ control
 *                        █   │State        │    state
 *         desired        █   └─────────────┘
 *         grip   ───────▶█
 *         state
 *
 *```
 *    - ControlMode::kForce
 *```
 *        ┌───────────┐
 *        │Desired    │                          desired
 *        │Mean Finger├────────────────────────▶ control
 *        │State      │                          state
 *        └───────────┘
 *
 *         desired        ┌────────┐
 *         grip   ───────▶│IGNORED │
 *         state          └────────┘
 *```
 *  - Handle Feed-Forward Force
 *    - ControlMode::kPosition
 *```
 *                                     █────▶ mean finger force
 *         pid                         █
 *         controller ────────────────▶█
 *         output                      █
 *                                     █────▶ grip force
 *         feed           ┌────────┐
 *         forward ──────▶│IGNORED │
 *         force          └────────┘
 *```
 *    - ControlMode::kForce
 *```
 *         pid
 *         controller ──────────────────────▶ mean finger force
 *         output
 *
 *         feed
 *         forward ─────────────────────────▶ grip force
 *         force
 *
 *```
 * The remaining blocks differ only in their numerical parameters.
 *
 * Note that the "feed forward force" input is ignored for
 * ControlMode::kPosition and the "desired grip state" input is ignored for
 * ControlMode::kPosition.
 */
class SchunkWsgPlainController
    : public systems::Diagram<double>,
      public systems::controllers::StateFeedbackControllerInterface<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgPlainController)
  /** Specify control gains and mode. Mode defaults to position control. */
  SchunkWsgPlainController(ControlMode control_mode = ControlMode::kPosition,
                           double kp = 2000, double ki = 0, double kd = 5);

  /** Returns the descriptor for the feed-forward force input port.
   * @pre `this` was constructed with `control_mode` set to
   * `ControlMode::kForce`.*/
  const systems::InputPortDescriptor<double>&
  get_input_port_feed_forward_force() const {
    DRAKE_ASSERT(feed_forward_force_input_port_ >= 0);
    return this->get_input_port(feed_forward_force_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_max_force() const {
    return this->get_input_port(max_force_input_port_);
  }

  // These methods implement StateFeedbackControllerInterface.
  const systems::InputPortDescriptor<double>& get_input_port_estimated_state()
      const override {
    return this->get_input_port(state_input_port_);
  }

  /** Returns the descriptor for the desired grip state input port.
   * @pre `this` was constructed with `control_mode` set to
   * `ControlMode::kPosition`.*/
  const systems::InputPortDescriptor<double>& get_input_port_desired_state()
      const override {
    DRAKE_ASSERT(state_input_port_ >= 0);
    return this->get_input_port(desired_grip_state_input_port_);
  }

  const systems::OutputPort<double>& get_output_port_control() const override {
    return systems::Diagram<double>::get_output_port(0);
  }

  const systems::OutputPort<double>& get_output_port(int) const = delete;

 private:
  int desired_grip_state_input_port_{-1};
  int feed_forward_force_input_port_{-1};
  int state_input_port_{-1};
  int max_force_input_port_{-1};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
