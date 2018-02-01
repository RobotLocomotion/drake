#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include <map>
#include <set>
#include <string>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaAndWsgPlantWithStateEstimator<T>::IiwaAndWsgPlantWithStateEstimator(
    std::unique_ptr<systems::RigidBodyPlant<T>> combined_plant,
    const std::vector<manipulation::util::ModelInstanceInfo<T>>& iiwa_instances,
    const std::vector<manipulation::util::ModelInstanceInfo<T>>& wsg_instances,
    const std::vector<manipulation::util::ModelInstanceInfo<T>>&
        object_instances) {
  if (iiwa_instances.size() != wsg_instances.size()) {
    throw std::logic_error("The number of arms and grippers do not match.");
  }
  const int kNumArms = iiwa_instances.size();
  const int kNumObjects = object_instances.size();
  this->set_name("IiwaAndWsgPlantWithStateEstimator");

  manipulation::util::SimDiagramBuilder<T> builder;
  systems::DiagramBuilder<T>* base_builder = builder.get_mutable_builder();

  plant_ = builder.AddPlant(std::move(combined_plant));
  plant_->set_name("IiwaAndWsgCombinedPlant");

  for (int i = 0; i < kNumArms; ++i) {
    const std::string suffix{"_" + std::to_string(i)};
    const auto& iiwa_output_port =
        plant_->model_instance_state_output_port(iiwa_instances[i].instance_id);

    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
    // Uses integral gains to deal with the added mass from the grasped object.
    iiwa_ki << 1, 1, 1, 1, 1, 1, 1;

    // Exposing feedforward acceleration. Should help with more dynamic
    // motions.
    auto single_arm = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        iiwa_instances[i].absolute_model_path, multibody::joints::kFixed,
        iiwa_instances[i].world_offset, single_arm.get());

    auto iiwa_controller = builder.template AddController<
        systems::controllers::InverseDynamicsController<T>>(
        iiwa_instances[i].instance_id, std::move(single_arm), iiwa_kp, iiwa_ki,
        iiwa_kd, true /* with feedforward acceleration */);
    iiwa_controller->set_name("IIWAInverseDynamicsController" + suffix);

    // Updates the controller's model's end effector's inertia to include
    // the added gripper.
    const std::string kEndEffectorLinkName = "iiwa_link_7";
    Matrix6<T> lumped_gripper_inertia_EE =
        ComputeLumpedGripperInertiaInEndEffectorFrame(
            plant_->get_rigid_body_tree(), iiwa_instances[i].instance_id,
            kEndEffectorLinkName, wsg_instances[i].instance_id);
    RigidBody<T>* controller_ee =
        iiwa_controller->get_robot_for_control().FindBody(kEndEffectorLinkName);
    controller_ee->set_spatial_inertia(lumped_gripper_inertia_EE);
    // Export iiwa's desired state input, and state output.
    input_port_iiwa_state_command_.push_back(base_builder->ExportInput(
        iiwa_controller->get_input_port_desired_state()));
    input_port_iiwa_acceleration_command_.push_back(base_builder->ExportInput(
        iiwa_controller->get_input_port_desired_acceleration()));
    output_port_iiwa_state_.push_back(
        base_builder->ExportOutput(iiwa_output_port));
    // Export the inverse dynamics controller computed torque output
    output_port_iiwa_torque_.push_back(
        base_builder->ExportOutput(iiwa_controller->get_output_port_control()));
    // Export iiwa's measured joint torque.
    output_port_iiwa_measured_torque_.push_back(
        base_builder->ExportOutput(
            plant_->model_instance_torque_output_port(
                iiwa_instances[i].instance_id)));

    // Sets up the WSG gripper part.
    const auto& wsg_input_port =
        plant_->model_instance_actuator_command_input_port(
            wsg_instances[i].instance_id);
    const auto& wsg_output_port =
        plant_->model_instance_state_output_port(wsg_instances[i].instance_id);

    //  Export wsg's actuator command input, and state output.
    input_port_wsg_command_.push_back(
        base_builder->ExportInput(wsg_input_port));
    output_port_wsg_state_.push_back(
        base_builder->ExportOutput(wsg_output_port));
    // Export wsg's measured joint torque.
    output_port_wsg_measured_torque_.push_back(
        base_builder->ExportOutput(
            plant_->model_instance_torque_output_port(
                wsg_instances[i].instance_id)));

    // Sets up a "state estimator" for iiwa that generates
    // bot_core::robot_state_t messages.
    auto iiwa_state_est =
        base_builder->template AddSystem<OracularStateEstimation<T>>(
            iiwa_controller->get_robot_for_control());
    iiwa_state_est->set_name("OracularStateEstimationIIWAState" + suffix);
    base_builder->Connect(iiwa_output_port,
                          iiwa_state_est->get_input_port_state());
    output_port_iiwa_robot_state_t_.push_back(
        base_builder->ExportOutput(iiwa_state_est->get_output_port_msg()));
  }

  output_port_plant_state_ =
      base_builder->ExportOutput(plant_->get_output_port(0));

  for (int i = 0; i < kNumObjects; ++i) {
    // Sets up a "state estimator" for the object that generates
    // bot_core::robot_state_t messages.
    // Make a object RBT for the fake state estimator.
    const std::string suffix{"_" + std::to_string(i)};
    objects_.emplace_back(new RigidBodyTree<T>);
    parsers::urdf::AddModelInstanceFromUrdfFile(
        object_instances[i].absolute_model_path, multibody::joints::kQuaternion,
        object_instances[i].world_offset, objects_.back().get());
    auto object_state_est =
        base_builder->template AddSystem<OracularStateEstimation<T>>(
            *objects_.back());
    object_state_est->set_name("OracularStateEstimationObjectState" + suffix);
    base_builder->Connect(plant_->model_instance_state_output_port(
                              object_instances[i].instance_id),
                          object_state_est->get_input_port_state());
    output_port_object_robot_state_t_.push_back(
        base_builder->ExportOutput(object_state_est->get_output_port_msg()));
  }

  output_port_contact_results_ =
      base_builder->ExportOutput(plant_->contact_results_output_port());

  output_port_kinematics_results_ =
      base_builder->ExportOutput(plant_->kinematics_results_output_port());

  builder.BuildInto(this);
}
template class IiwaAndWsgPlantWithStateEstimator<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
