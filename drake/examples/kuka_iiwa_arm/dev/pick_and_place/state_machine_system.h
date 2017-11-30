#pragma once

#include <memory>
#include <string>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_symbolic_inspector.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

/**
 * A class that implements the Finite-State-Machine logic for the
 * Pick-And-Place demo. This system should be used by coupling the outputs with
 * the `IiwaMove` and `GripperAction` systems and the inputs are to be
 * connected to the appropriate output ports of the `IiwaStatusSender`,
 * `SchunkWsgStatusSender` and `OracularStateEstimator` systems.
 */
class PickAndPlaceStateMachineSystem : public systems::LeafSystem<double> {
 public:
  /**
   * Constructor for the PickAndPlaceStateMachineSystem
   * @param iiwa_base, The pose of the base of the IIWA robot system.
   * @param period_sec : The update interval of the unrestricted update of
   * this system. This should be bigger than that of the PlanSource components.
   */
  PickAndPlaceStateMachineSystem(
      const pick_and_place::PlannerConfiguration& configuration,
      bool single_move);

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  // This kind of a system is not a direct feedthrough.
  optional<bool> DoHasDirectFeedthrough(int, int) const final { return false; }

  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const override;

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * state message (LCM `lcmt_iiwa_status` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_state()
      const {
    return this->get_input_port(input_port_iiwa_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with iiwa
   * base pose.
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_iiwa_base_pose()
      const {
    return this->get_input_port(input_port_iiwa_base_pose_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_box_state() const {
    return this->get_input_port(input_port_box_state_);
  }

  /**
   * Getter for the input port corresponding to the abstract input with box
   * state message (LCM `botcore::robot_state_t` message).
   * @return The corresponding `sytems::InputPortDescriptor`.
   */
  const systems::InputPortDescriptor<double>& get_input_port_table_state(
      int index) const {
    DRAKE_THROW_UNLESS(
        index >= 0 && index < static_cast<int>(input_port_table_state_.size()));
    return this->get_input_port(input_port_table_state_[index]);
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

  const systems::OutputPort<double>& get_output_port_iiwa_plan() const {
    return this->get_output_port(output_port_iiwa_plan_);
  }

  const systems::OutputPort<double>& get_output_port_wsg_command() const {
    return this->get_output_port(output_port_wsg_command_);
  }

  /// Return the state of the pick and place state machine.
  pick_and_place::PickAndPlaceState state(
      const systems::Context<double>&) const;

  /// Return the state of the pick and place world.  Note that this
  /// reference is into data contained inside the passed in context.
  const pick_and_place::WorldState& world_state(
      const systems::Context<double>&) const;

 private:
  void CalcIiwaPlan(const systems::Context<double>& context,
                    robotlocomotion::robot_plan_t* iiwa_plan) const;

  void CalcWsgCommand(const systems::Context<double>& context,
                      lcmt_schunk_wsg_command* wsg_command) const;

  int num_tables() const { return configuration_.num_tables; }

  const std::string& iiwa_model_path() const {
    return configuration_.model_path;
  }

  const std::string& end_effector_name() const {
    return configuration_.end_effector_name;
  }

  const Vector3<double>& target_dimensions() const {
    return configuration_.target_dimensions;
  }

  struct InternalState;

  RigidBodyTree<double> iiwa_tree_{};
  // Input ports.
  int input_port_iiwa_state_{-1};
  int input_port_iiwa_base_pose_{-1};
  int input_port_box_state_{-1};
  std::vector<int> input_port_table_state_;
  int input_port_wsg_status_{-1};
  // Output ports.
  int output_port_iiwa_plan_{-1};
  int output_port_wsg_command_{-1};

  const pick_and_place::PlannerConfiguration configuration_;

  bool single_move_;
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
