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

/// A custom systems::Diagram composed of a systems::RigidBodyPlant, a
/// systems::controllers::InverseDynamicsController, and a number of
/// OracularStateEstimation systems. The systems::RigidBodyPlant must be
/// generated from a RigidBodyTree containing at least one Kuka IIWA/Schunk WSG
/// pair and at least one additional object. The OracularStateEstimation
/// systems are coupled with the output of the systems::RigidBodyPlant. The
/// resulting diagram exposes an input port for the desired state and
/// acceleration of each IIWA arm as well as the actuator command for each WSG
/// gripper. It exposes an output port for the state of each IIWA arm and WSG
/// gripper, as well as one for the complete state of the
/// systems::RigidBodyPlant and another for the contact results of the
/// systems::RigidBodyPlant. It also exposes a `bot_core::robot_state_t` message
/// output port for each IIWA arm and each of the additional objects.
///
/// This class is explicitly instantiated for the following scalar type(s). No
/// other scalar types are supported.
/// - double
template <typename T>
class IiwaAndWsgPlantWithStateEstimator : public systems::Diagram<T> {
 public:
  /// Constructs the IiwaAndWsgPlantWithStateEstimator.
  /// @param combined_plant a systems::RigidBodyPlant containing some number of
  ///        Kuka IIWA/Schunk WSG pairs, and some number of other objects.
  /// @param iiwa_instances vector of ModelInstanceInfo objects corresponding to
  ///        the Kuka IIWA arms in @p combined_plant.
  /// @param wsg_instances vector of ModelInstanceInfo objects corresponding to
  ///        the Schunk WSG grippers in @p combined_plant.
  /// @param object_instances vector of ModelInstanceInfo objects corresponding
  ///        to the other objects in @p combined_plant.
  /// @throws std::logic_error if @p iiwa_instances and @p wsg_instances are not
  ///         the same size.
  IiwaAndWsgPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      const std::vector<manipulation::util::ModelInstanceInfo<T>>&
          iiwa_instances,
      const std::vector<manipulation::util::ModelInstanceInfo<T>>&
          wsg_instances,
      const std::vector<manipulation::util::ModelInstanceInfo<T>>&
          object_instances);

  /// Constructs an IiwaAndWsgPlantWithStateEstimator for a RigidBodyPlant @p
  /// combined_plant that contains a single Kuka IIWA, Schunk WSG, and object.
  /// @param combined_plant a systems::RigidBodyPlant containing some number of
  ///        Kuka IIWA/Schunk WSG pairs, and some number of other objects.
  /// @param iiwa_instances a ModelInstanceInfo object corresponding to the
  ///        Kuka IIWA arm in @p combined_plant.
  /// @param wsg_instances a ModelInstanceInfo object corresponding to the
  ///        Schunk WSG gripper in @p combined_plant.
  /// @param object_instances a ModelInstanceInfo object corresponding to the
  ///        other object in @p combined_plant.
  IiwaAndWsgPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      const manipulation::util::ModelInstanceInfo<T>& iiwa_instance,
      const manipulation::util::ModelInstanceInfo<T>& wsg_instance,
      const manipulation::util::ModelInstanceInfo<T>& object_instance)
      : IiwaAndWsgPlantWithStateEstimator(
            std::move(combined_plant),
            std::vector<manipulation::util::ModelInstanceInfo<T>>{
                iiwa_instance},
            std::vector<manipulation::util::ModelInstanceInfo<T>>{wsg_instance},
            std::vector<manipulation::util::ModelInstanceInfo<T>>{
                object_instance}) {}

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

  const systems::OutputPort<T>& get_output_port_object_robot_state_msg(
      int index = 0) const {
    return this->get_output_port(output_port_object_robot_state_t_.at(index));
  }

  const systems::OutputPort<T>& get_output_port_contact_results() const {
    return this->get_output_port(output_port_contact_results_);
  }

  const systems::OutputPort<T>& get_output_port_kinematics_results() const {
    return this->get_output_port(output_port_kinematics_results_);
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
  std::vector<int> output_port_object_robot_state_t_;
  int output_port_plant_state_{-1};
  int output_port_contact_results_{-1};
  int output_port_kinematics_results_{-1};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
