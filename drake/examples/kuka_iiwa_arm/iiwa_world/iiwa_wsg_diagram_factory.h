#pragma once

#include <memory>

#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
using systems::PassThrough;
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
/// `systems::InverseDynamicsController` of the IIWA robot), WSG (for the
/// `systems::PidController` for the Schunk WSG Gripper) and output ports for
/// IIWA state, WSG state, the complete `systems::RigidBodyPlant` state
/// messages for the IIWA robot and an object for manipulation.
///
/// This class is explicitly instantiated for the following scalar type(s). No
/// other scalar types are supported.
/// - double
template <typename T>
class IiwaAndWsgPlantWithStateEstimator : public systems::Diagram<T> {
 public:
  /// Constructs the IiwaAndWsgPlantWithStateEstimator.
  /// `combined_plant` :  a `RigidBodyPlant` that is assumed to be generated
  /// containing a Kuka IIWA and a Schunk WSG model, as well as a "box"
  /// object. The arguments `iiwa_info`, `wsg_info`, and `box_info` are the
  /// `ModelInstanceInfo` objects corresponding to the Kuka IIWA, Schunk WSG
  /// and the box object respectively.
  IiwaAndWsgPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      const ModelInstanceInfo<T>& iiwa_info,
      const ModelInstanceInfo<T>& wsg_info,
      const ModelInstanceInfo<T>& box_info);

  const systems::RigidBodyPlant<T>& get_plant() const { return *plant_; }

  const RigidBodyTree<T>& get_tree() const {
    return plant_->get_rigid_body_tree();
  }

  const systems::InputPortDescriptor<T>& get_input_port_iiwa_state_command()
      const {
    return this->get_input_port(input_port_iiwa_state_command_);
  }

  const systems::InputPortDescriptor<T>&
  get_input_port_iiwa_acceleration_command() const {
    return this->get_input_port(input_port_iiwa_acceleration_command_);
  }

  const systems::InputPortDescriptor<T>& get_input_port_wsg_command() const {
    return this->get_input_port(input_port_wsg_command_);
  }

  const systems::OutputPort<T>& get_output_port_iiwa_state() const {
    return this->get_output_port(output_port_iiwa_state_);
  }

  const systems::OutputPort<T>& get_output_port_wsg_state() const {
    return this->get_output_port(output_port_wsg_state_);
  }

  const systems::OutputPort<T>& get_output_port_plant_state() const {
    return this->get_output_port(output_port_plant_state_);
  }

  const systems::OutputPort<T>& get_output_port_iiwa_robot_state_msg()
      const {
    return this->get_output_port(output_port_iiwa_robot_state_t_);
  }

  const systems::OutputPort<T>& get_output_port_box_robot_state_msg()
      const {
    return this->get_output_port(output_port_box_robot_state_t_);
  }

 private:
  OracularStateEstimation<T>* iiwa_state_est_{nullptr};
  OracularStateEstimation<T>* box_state_est_{nullptr};
  std::unique_ptr<RigidBodyTree<T>> object_{nullptr};
  systems::InverseDynamicsController<T>* iiwa_controller_{nullptr};
  systems::PidController<T>* wsg_controller_{nullptr};
  systems::RigidBodyPlant<T>* plant_{nullptr};

  int input_port_iiwa_state_command_{-1};
  int input_port_iiwa_acceleration_command_{-1};
  int input_port_wsg_command_{-1};
  int output_port_iiwa_state_{-1};
  int output_port_wsg_state_{-1};
  int output_port_plant_state_{-1};
  int output_port_iiwa_robot_state_t_{-1};
  int output_port_box_robot_state_t_{-1};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
