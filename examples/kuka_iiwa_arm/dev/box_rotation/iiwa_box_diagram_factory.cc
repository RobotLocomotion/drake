#include "drake/examples/kuka_iiwa_arm/dev/box_rotation/iiwa_box_diagram_factory.h"

#include <utility>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
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
namespace box_rotation {

template<typename T>
IiwaAndBoxPlantWithStateEstimator<T>::IiwaAndBoxPlantWithStateEstimator(
    std::unique_ptr<RigidBodyPlant<T>> combined_plant,
    const manipulation::util::ModelInstanceInfo<T>& iiwa_info,
    const manipulation::util::ModelInstanceInfo<T>& box_info) {
  this->set_name("IiwaAndBoxPlantWithStateEstimator");

  manipulation::util::SimDiagramBuilder<T> builder;
  DiagramBuilder<T>* base_builder = builder.get_mutable_builder();

  plant_ = builder.AddPlant(std::move(combined_plant));
  plant_->set_name("IiwaAndBoxCombinedPlant");

  const auto& iiwa_output_port =
      plant_->model_instance_state_output_port(iiwa_info.instance_id);

  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;

  SetPositionControlledGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

  auto single_arm = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      iiwa_info.model_path, multibody::joints::kFixed, iiwa_info.world_offset,
      single_arm.get());

  DRAKE_DEMAND(single_arm->get_num_positions() % kIiwaArmNumJoints == 0);
  for (int offset = kIiwaArmNumJoints; offset < single_arm->get_num_positions();
       offset += kIiwaArmNumJoints) {
    const int end = offset + kIiwaArmNumJoints;
    iiwa_kp.conservativeResize(end);
    iiwa_kp.segment(offset, kIiwaArmNumJoints) =
        iiwa_kp.head(kIiwaArmNumJoints);
    iiwa_ki.conservativeResize(end);
    iiwa_ki.segment(offset, kIiwaArmNumJoints) =
        iiwa_ki.head(kIiwaArmNumJoints);
    iiwa_kd.conservativeResize(end);
    iiwa_kd.segment(offset, kIiwaArmNumJoints) =
        iiwa_kd.head(kIiwaArmNumJoints);
  }

  iiwa_controller_ = builder.template AddController<
      systems::controllers::InverseDynamicsController<T>>(
      iiwa_info.instance_id, std::move(single_arm), iiwa_kp, iiwa_ki, iiwa_kd,
      true /* with feedforward acceleration */);
  iiwa_controller_->set_name("IIWAInverseDynamicsController");

  // Export iiwa's desired state input, and state output.
  input_port_iiwa_state_command_ = base_builder->ExportInput(
      iiwa_controller_->get_input_port_desired_state());
  input_port_iiwa_acceleration_command_ = base_builder->ExportInput(
      iiwa_controller_->get_input_port_desired_acceleration());
  output_port_iiwa_state_ = base_builder->ExportOutput(iiwa_output_port);

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

  output_port_contact_results_ =
      base_builder->ExportOutput(plant_->contact_results_output_port());

  output_port_kinematics_results_ =
      base_builder->ExportOutput(plant_->kinematics_results_output_port());

  builder.BuildInto(this);
}
template
class IiwaAndBoxPlantWithStateEstimator<double>;

template<typename T>
void IiwaAndBoxPlantWithStateEstimator<T>::SetPositionControlledGains(
    Eigen::VectorXd* Kp, Eigen::VectorXd* Ki, Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(7);
  *Kp << 5000, 5000, 5000, 5000, 5000, 5000, 5000;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(7);
}

}  // namespace box_rotation
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
