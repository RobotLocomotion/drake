#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::RigidBodyPlant;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaAndWsgPlantWithStateEstimator<T>::IiwaAndWsgPlantWithStateEstimator(
    std::unique_ptr<RigidBodyPlant<T>> combined_plant,
    const ModelInstanceInfo<T>& iiwa_info, const ModelInstanceInfo<T>& wsg_info,
    const ModelInstanceInfo<T>& box_info) {
  this->set_name("IiwaAndWsgPlantWithStateEstimator");

  SimDiagramBuilder<T> builder;
  DiagramBuilder<T>* base_builder = builder.get_mutable_builder();

  plant_ = builder.AddPlant(std::move(combined_plant));
  plant_->set_name("IiwaAndWsgCombinedPlant");

  const auto& iiwa_output_port =
      plant_->model_instance_state_output_port(iiwa_info.instance_id);

  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  // Uses integral gains to deal with the added mass from the grasped object.
  iiwa_ki << 1, 1, 1, 1, 1, 1, 1;

  // Exposing feedforward acceleration. Should help with more dynamic
  // motions.
  auto single_arm = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      iiwa_info.model_path, multibody::joints::kFixed, iiwa_info.world_offset,
      single_arm.get());

  iiwa_controller_ = builder.template AddController<
      systems::controllers::InverseDynamicsController<T>>(
      iiwa_info.instance_id, std::move(single_arm), iiwa_kp, iiwa_ki, iiwa_kd,
      true /* with feedforward acceleration */);
  iiwa_controller_->set_name("IIWAInverseDynamicsController");

  // Updates the controller's model's end effector's inertia to include
  // the added gripper.
  const std::string kEndEffectorLinkName = "iiwa_link_7";
  Matrix6<T> lumped_gripper_inertia_EE =
      ComputeLumpedGripperInertiaInEndEffectorFrame(
          plant_->get_rigid_body_tree(), iiwa_info.instance_id,
          kEndEffectorLinkName, wsg_info.instance_id);
  RigidBody<T>* controller_ee =
      iiwa_controller_->get_robot_for_control().FindBody(kEndEffectorLinkName);
  controller_ee->set_spatial_inertia(lumped_gripper_inertia_EE);

  // Export iiwa's desired state input, and state output.
  input_port_iiwa_state_command_ = base_builder->ExportInput(
      iiwa_controller_->get_input_port_desired_state());
  input_port_iiwa_acceleration_command_ = base_builder->ExportInput(
      iiwa_controller_->get_input_port_desired_acceleration());
  output_port_iiwa_state_ = base_builder->ExportOutput(iiwa_output_port);

  // Sets up the WSG gripper part.
  const auto& wsg_input_port =
      plant_->model_instance_actuator_command_input_port(wsg_info.instance_id);
  const auto& wsg_output_port =
      plant_->model_instance_state_output_port(wsg_info.instance_id);

  //  Export wsg's actuator command input, and state output.
  input_port_wsg_command_ = base_builder->ExportInput(wsg_input_port);
  output_port_wsg_state_ = base_builder->ExportOutput(wsg_output_port);

  output_port_plant_state_ =
      base_builder->ExportOutput(plant_->get_output_port(0));

  // Sets up a "state estimator" for iiwa that generates
  // bot_core::robot_state_t messages.
  iiwa_state_est_ =
      base_builder->template AddSystem<OracularStateEstimation<T>>(
          iiwa_controller_->get_robot_for_control());
  iiwa_state_est_->set_name("OracularStateEstimationIIWAState");
  base_builder->Connect(iiwa_output_port,
                        iiwa_state_est_->get_input_port_state());
  output_port_iiwa_robot_state_t_ =
      base_builder->ExportOutput(iiwa_state_est_->get_output_port_msg());

  // Sets up a "state estimator" for the box that generates
  // bot_core::robot_state_t messages.
  // Make a box RBT for the fake state estimator.
  object_ = std::make_unique<RigidBodyTree<T>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      box_info.model_path, multibody::joints::kQuaternion,
      box_info.world_offset, object_.get());
  box_state_est_ =
      base_builder->template AddSystem<OracularStateEstimation<T>>(*object_);
  box_state_est_->set_name("OracularStateEstimationBoxState");
  base_builder->Connect(
      plant_->model_instance_state_output_port(box_info.instance_id),
      box_state_est_->get_input_port_state());
  output_port_box_robot_state_t_ =
      base_builder->ExportOutput(box_state_est_->get_output_port_msg());

  output_port_contact_results_t_ =
      base_builder->ExportOutput(plant_->contact_results_output_port());

  builder.BuildInto(this);
}
template class IiwaAndWsgPlantWithStateEstimator<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
