#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include <map>
#include <set>
#include <string>
#include <utility>
#include "drake/util/drakeGeometryUtil.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/schunk_wsg/schunk_wsg_constants.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::InputPortDescriptor;
using systems::OutputPortDescriptor;
using systems::RigidBodyPlant;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaAndWsgPlantWithStateEstimator<T>::IiwaAndWsgPlantWithStateEstimator(
    std::unique_ptr<RigidBodyPlant<T>> combined_plant,
    const ModelInstanceInfo<T>& iiwa_info,
    const ModelInstanceInfo<T>& wsg_info,
    const ModelInstanceInfo<T>& box_info) {
  this->set_name("IiwaAndWsgPlantWithStateEstimator");

  SimDiagramBuilder<T> builder;
  DiagramBuilder<T>* base_builder = builder.get_mutable_builder();

  plant_ = builder.AddPlant(std::move(combined_plant));

  const auto& iiwa_output_port =
    plant_->model_instance_state_output_port(iiwa_info.instance_id);

  const auto& wsg_output_port =
    plant_->model_instance_state_output_port(wsg_info.instance_id);

  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  // Uses integral gains to deal with the added mass from the grasped object.
  iiwa_ki << 1, 1, 1, 1, 1, 1, 1;

  // Exposing feedforward acceleration. Should help with more dynamic
  // motions.
  iiwa_controller_ =
      builder.template AddController<systems::InverseDynamicsController<T>>(
          iiwa_info.instance_id,
          iiwa_info.model_path, iiwa_info.world_offset, iiwa_kp, iiwa_ki,
          iiwa_kd, true /* with feedforward acceleration */);

  // Updates the controller's model's end effector's inertia to include
  // the added gripper.
  const std::string kEndEffectorLinkName = "iiwa_link_7";
  Matrix6<T> lumped_gripper_inertia_EE =
    ComputeLumpedGripperInertiaInEndEffectorFrame(
        plant_->get_rigid_body_tree(),
        iiwa_info.instance_id, kEndEffectorLinkName, wsg_info.instance_id);
  RigidBody<T>* controller_ee =
    iiwa_controller_->get_robot_for_control().FindBody(kEndEffectorLinkName);
  controller_ee->set_spatial_inertia(lumped_gripper_inertia_EE);

  // Export iiwa's desired state input, and state output.
  base_builder->ExportInput(iiwa_controller_->get_input_port_desired_state());
  base_builder->ExportInput(
      iiwa_controller_->get_input_port_desired_acceleration());
  base_builder->ExportOutput(iiwa_output_port);

  // Sets up the WSG gripper part.
  std::unique_ptr<systems::MatrixGain<T>> feedback_selector =
    std::make_unique<systems::MatrixGain<T>>(
        schunk_wsg::GetSchunkWsgFeedbackSelector<T>());
  // TODO(sam.creasey) The choice of position gains below is completely
  // arbitrary. We'll need to revisit this once we switch to force control
  // for the gripper.
  const int kWsgActDim = schunk_wsg::kSchunkWsgNumActuators;
  const VectorX<T> wsg_kp = VectorX<T>::Constant(kWsgActDim, 300.0);
  const VectorX<T> wsg_ki = VectorX<T>::Constant(kWsgActDim, 0.0);
  const VectorX<T> wsg_kd = VectorX<T>::Constant(kWsgActDim, 5.0);

  wsg_controller_ =
      builder.template AddController<systems::PidController<T>>(
          wsg_info.instance_id,
          std::move(feedback_selector), wsg_kp, wsg_ki, wsg_kd);

  //  Export wsg's desired state input, and state output.
  base_builder->ExportInput(wsg_controller_->get_input_port_desired_state());
  base_builder->ExportOutput(wsg_output_port);

  base_builder->ExportOutput(plant_->get_output_port(0));

  // Sets up a "state estimator" for iiwa that generates
  // bot_core::robot_state_t messages.
  iiwa_state_est_ =
      base_builder->template AddSystem<OracularStateEstimation<T>>(
          iiwa_controller_->get_robot_for_control(),
          iiwa_controller_->get_robot_for_control().get_body(1));
  base_builder->Connect(iiwa_output_port,
      iiwa_state_est_->get_input_port_state());
  base_builder->ExportOutput(iiwa_state_est_->get_output_port_msg());

  // Sets up a "state estimator" for the box that generates
  // bot_core::robot_state_t messages.
  // Make a box RBT for the fake state estimator.
  object_ = std::make_unique<RigidBodyTree<T>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      box_info.model_path, multibody::joints::kQuaternion,
      box_info.world_offset, object_.get());
  box_state_est_ =
      base_builder->template AddSystem<OracularStateEstimation<T>>(
          *object_, object_->get_body(1));
  base_builder->Connect(
      plant_->model_instance_state_output_port(box_info.instance_id),
      box_state_est_->get_input_port_state());
  base_builder->ExportOutput(box_state_est_->get_output_port_msg());

  builder.BuildInto(this);
}

template class IiwaAndWsgPlantWithStateEstimator<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
