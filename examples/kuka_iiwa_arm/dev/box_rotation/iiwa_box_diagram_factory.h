#pragma once

#include <memory>

#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace box_rotation {

/// A custom `systems::Diagram` composed of a `systems::RigidBodyPlant`, and a
/// `systems::InverseDynamicsController`, and two
/// `OracularStateEstimation` systems. The `systems::RigidBodyPlant` must be
/// generated from a `RigidBodyTree` containing two Kuka IIWAs and a box
/// `RigidBody`. The `OracularStateEstimation` systems are coupled with the
/// output of the `systems::RigidBodyPlant`. The resulting diagram exposes input
/// ports for the IIWA state and acceleration (for the
/// `systems::InverseDynamicsController` of the IIWA robot), and output ports
/// for IIWA state, the complete `systems::RigidBodyPlant` state messages for
/// the IIWA robot and a box for manipulation.
///
/// This class is explicitly instantiated for the following scalar type(s). No
/// other scalar types are supported.
/// - double
template<typename T>
class IiwaAndBoxPlantWithStateEstimator : public systems::Diagram<T> {
 public:
  /// Constructs the IiwaAndBoxPlantWithStateEstimator.
  /// `combined_plant` :  a `RigidBodyPlant` that is assumed to be generated
  /// containing the Kuka IIWAs as well as a "box"
  /// object. The arguments `iiwa_info` and `box_info` are the
  /// `ModelInstanceInfo` objects corresponding to the Kuka IIWA and the box,
  /// respectively.
  IiwaAndBoxPlantWithStateEstimator(
      std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
      const manipulation::util::ModelInstanceInfo<T>& iiwa_info,
      const manipulation::util::ModelInstanceInfo<T>& box_info);

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

  const systems::OutputPort<T>& get_output_port_iiwa_state() const {
    return this->get_output_port(output_port_iiwa_state_);
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

  const systems::OutputPort<T>& get_output_port_contact_results()
  const {
    return this->get_output_port(output_port_contact_results_);
  }

  const systems::OutputPort<T> &get_output_port_kinematics_results()
  const {
    return this->get_output_port(output_port_kinematics_results_);
  }

  void SetPositionControlledGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                  Eigen::VectorXd* Kd);

 private:
  OracularStateEstimation<T>* iiwa_state_est_{nullptr};
  OracularStateEstimation<T>* box_state_est_{nullptr};
  std::unique_ptr<RigidBodyTree<T>> object_{nullptr};
  systems::controllers::InverseDynamicsController<T>* iiwa_controller_{nullptr};
  systems::RigidBodyPlant<T>* plant_{nullptr};

  int input_port_iiwa_state_command_{-1};
  int input_port_iiwa_acceleration_command_{-1};
  int output_port_iiwa_state_{-1};
  int output_port_plant_state_{-1};
  int output_port_iiwa_robot_state_t_{-1};
  int output_port_box_robot_state_t_{-1};
  int output_port_contact_results_{-1};
  int output_port_kinematics_results_{-1};
};

}  // namespace box_rotation
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
