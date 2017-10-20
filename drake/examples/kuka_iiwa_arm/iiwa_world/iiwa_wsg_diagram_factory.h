#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// A custom `systems::Diagram` composed of a `systems::RigidBodyPlant`, and a
/// `systems::InverseDynamicsController`, `systems::PidController`, and two
/// `OracularStateEstimation` systems. The `systems::RigidBodyPlant` must be
/// generate from a `RigidBodyTree` containing a Kuka IIWA and a Schunk WSG
/// model and at least one additional `RigidBody`. The
/// `OracularStateEstimation` systems are coupled with the output of the
/// `systems::RigidBodyPlant`. The resulting diagram exposes input ports for
/// the IIWA state and acceleration (for the
/// `systems::InverseDynamicsController` of the IIWA robot), WSG
/// (directly feeds through to the actuator for the Schunk WSG
/// Gripper) and output ports for IIWA state, WSG state, the complete
/// `systems::RigidBodyPlant` state messages for the IIWA robot and an
/// object for manipulation.
///
/// This class is explicitly instantiated for the following scalar type(s). No
/// other scalar types are supported.
/// - double
template <typename T>
class IiwaAndWsgPlantWithStateEstimator : public systems::Diagram<T> {
 public:
  /// Constructs the IiwaAndWsgPlantWithStateEstimator.  `combined_plant` :  a
  /// `RigidBodyPlant` that is assumed to be generated containing a Kuka IIWA
  /// and a Schunk WSG model, as well as a "box" object. The arguments
  /// `iiwa_info`, `wsg_info`, and `box_info` are the
  /// `manipulation::util::ModelInstanceInfo` objects corresponding to the Kuka
  /// IIWA, Schunk WSG and the box object respectively.
  IiwaAndWsgPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      const std::vector<manipulation::util::ModelInstanceInfo<T>>& iiwa_info,
      const std::vector<manipulation::util::ModelInstanceInfo<T>>& wsg_info,
      const std::vector<manipulation::util::ModelInstanceInfo<T>>& box_info);

  /// Constructs an IiwaAndWsgPlantWithStateEstimator a RigidBodyPlant @p
  /// combined_plant that contains a single Kuka IIWA, Schunk WSG, and box.
  IiwaAndWsgPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      const manipulation::util::ModelInstanceInfo<T>& iiwa_info,
      const manipulation::util::ModelInstanceInfo<T>& wsg_info,
      const manipulation::util::ModelInstanceInfo<T>& box_info)
      : IiwaAndWsgPlantWithStateEstimator(
            std::move(combined_plant),
            std::vector<manipulation::util::ModelInstanceInfo<T>>{iiwa_info},
            std::vector<manipulation::util::ModelInstanceInfo<T>>{wsg_info},
            std::vector<manipulation::util::ModelInstanceInfo<T>>{box_info}) {}

  const systems::RigidBodyPlant<T>& get_plant() const { return *plant_; }

  const RigidBodyTree<T>& get_tree() const {
    return plant_->get_rigid_body_tree();
  }

  const systems::InputPortDescriptor<T>& get_input_port_iiwa_state_command(
      int index = 0) const {
    return this->get_input_port(input_port_iiwa_state_command_.at(index));
  }

  const systems::InputPortDescriptor<T>&
  get_input_port_iiwa_acceleration_command(int index = 0) const {
    return this->get_input_port(
        input_port_iiwa_acceleration_command_.at(index));
  }

  const systems::InputPortDescriptor<T>& get_input_port_wsg_command(
      int index = 0) const {
    return this->get_input_port(input_port_wsg_command_.at(index));
  }

  const systems::OutputPort<T>& get_output_port_iiwa_state(
      int index = 0) const {
    return this->get_output_port(output_port_iiwa_state_.at(index));
  }

  const systems::OutputPort<T>& get_output_port_wsg_state(int index = 0) const {
    return this->get_output_port(output_port_wsg_state_.at(index));
  }

  const systems::OutputPort<T>& get_output_port_plant_state() const {
    return this->get_output_port(output_port_plant_state_);
  }

  const systems::OutputPort<T>& get_output_port_iiwa_robot_state_msg(
      int index = 0) const {
    return this->get_output_port(output_port_iiwa_robot_state_t_.at(index));
  }

  const systems::OutputPort<T>& get_output_port_box_robot_state_msg(
      int index = 0) const {
    return this->get_output_port(output_port_box_robot_state_t_.at(index));
  }

  const systems::OutputPort<T>& get_output_port_contact_results() const {
    return this->get_output_port(output_port_contact_results_t_);
  }

 private:
  std::vector<std::unique_ptr<RigidBodyTree<T>>> objects_;
  systems::RigidBodyPlant<T>* plant_{nullptr};

  std::vector<int> input_port_iiwa_state_command_;
  std::vector<int> input_port_iiwa_acceleration_command_;
  std::vector<int> input_port_wsg_command_;
  std::vector<int> output_port_iiwa_state_;
  std::vector<int> output_port_wsg_state_;
  std::vector<int> output_port_iiwa_robot_state_t_;
  std::vector<int> output_port_box_robot_state_t_;
  int output_port_plant_state_{-1};
  int output_port_contact_results_t_{-1};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
