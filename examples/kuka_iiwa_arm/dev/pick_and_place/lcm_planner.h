#pragma once

#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/// A custom `systems::Diagram` composed of a `PickAndPlaceStateMachineSystem`
/// and `drake::manipulation::OptitrackPoseExtractor` systems.
class LcmPlanner : public systems::Diagram<double> {
 public:
  LcmPlanner(
      const pick_and_place::PlannerConfiguration& configuration,
      const pick_and_place::OptitrackConfiguration optitrack_configuration,
      bool single_move);

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `lcmt_iiwa_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_status()
      const {
    return this->get_input_port(input_port_iiwa_status_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the wsg
   * status message (LCM `lcmt_schunk_wsg_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_wsg_status()
      const {
    return this->get_input_port(input_port_wsg_status_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with the
   * optitrack message (LCM `optitrack::optitrack_frame_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_optitrack_message()
      const {
    return get_input_port(input_port_optitrack_message_);
  }

  const systems::OutputPort<double>& get_output_port_iiwa_plan() const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPort<double>& get_output_port_wsg_command() const {
    return this->get_output_port(output_port_wsg_command_);
  }

  pick_and_place::PickAndPlaceState state(
      const systems::Context<double>& context) const {
    return state_machine_->state(
        this->GetSubsystemContext(*state_machine_, context));
  }

  const pick_and_place::WorldState& world_state(
      const systems::Context<double>& context) const {
    return state_machine_->world_state(
        this->GetSubsystemContext(*state_machine_, context));
  }

 private:
  // Input ports.
  int input_port_iiwa_status_{-1};
  int input_port_wsg_status_{-1};
  int input_port_optitrack_message_{-1};

  // Output ports.
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_command_{-1};

  PickAndPlaceStateMachineSystem* state_machine_{};
};
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
