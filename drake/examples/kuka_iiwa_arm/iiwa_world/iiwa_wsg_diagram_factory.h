# pragma once

#include <memory>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {


/// A custom `systems::Diagram` composed of a `systems::RigidBodyPlant`, and a
/// `systems::InverseDynamicsController`, `systems::PidController`, and two
/// `OracularStateEstimation` systems. The `systems::RigidBodyPlant` must be
/// generated
/// from a `RigidBodyTree` containing a Kuka IIWA and a Schunk WSG model and atleast one
/// additional `RigidBody`. The `OracularStateEstimation` systems are coupled with the
/// output of the `systems::RigidBodyPlant`. The resulting diagram exposes input ports for
/// the IIWA state and acceleration(for the `systems::InverseDynamicsController`
/// of the IIWA robot), WSG (for the `systems::PidController` for the Schunk WSG Gripper)
/// and output ports for IIWA state, WSG state, the complete `systems::RigidBodyPlant`
/// state messages for the IIWA robot and an object for manipulation.
template<typename T>
class IiwaAndWsgPlantWithStateEstimator: public systems::Diagram<T> {
 public:
  ///
  IiwaAndWsgPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      ModelInstanceInfo<T> iiwa_info, ModelInstanceInfo<T> wsg_info,
      ModelInstanceInfo<T> box_info);

  const systems::RigidBodyPlant<T> &get_plant() const { return *plant_; }

  const systems::InputPortDescriptor<T> &get_iiwa_state_input_port() const {
    return this->get_input_port(0);
  }

  const systems::InputPortDescriptor<T> &get_iiwa_acceleration_input_port() const {
    return this->get_input_port(1);
  }

  const systems::OutputPortDescriptor<T> &get_iiwa_state_port() const {
    return this->get_output_port(0);
  }

  const systems::InputPortDescriptor<T> &get_wsg_input_port() const {
    return this->get_input_port(2);
  }

  const systems::OutputPortDescriptor<T> &get_wsg_state_port() const {
    return this->get_output_port(1);
  }

  const systems::OutputPortDescriptor<T> &get_plant_output_port() const {
    return this->get_output_port(2);
  }

  const systems::OutputPortDescriptor<T> &get_iiwa_robot_state_msg_port() const {
    return this->get_output_port(3);
  }

  const systems::OutputPortDescriptor<T> &get_box_robot_state_msg_port() const {
    return this->get_output_port(4);
  }

 private:
  OracularStateEstimation<T> *iiwa_state_est{nullptr};
  OracularStateEstimation<T> *box_state_est{nullptr};
  std::unique_ptr<RigidBodyTree<T>> object_;
  systems::InverseDynamicsController<T> *iiwa_controller{nullptr};
  systems::PidController<T> *wsg_controller;
  systems::RigidBodyPlant<T> *plant_{nullptr};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
