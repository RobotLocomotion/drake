#include "drake/examples/qp_inverse_dynamics/manipulator_joint_space_controller.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_system.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_output_translator_system.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state_translator_system.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::controllers::qp_inverse_dynamics::QpInverseDynamicsSystem;
using systems::controllers::qp_inverse_dynamics::QpOutputTranslatorSystem;
using systems::controllers::qp_inverse_dynamics::
    RobotKinematicStateTranslatorSystem;
using systems::controllers::qp_inverse_dynamics::ParamSet;

ManipulatorJointSpaceController::ManipulatorJointSpaceController(
    const std::string& model_path,
    std::unique_ptr<RigidBodyTreeAliasGroups<double>>* alias_groups,
    std::shared_ptr<RigidBodyTree<double>> robot_for_control,
    std::unique_ptr<ParamSet>* paramset,
    double dt, std::shared_ptr<RigidBodyFrame<double>> world_offset) {
  robot_for_control_ = robot_for_control;
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, world_offset,
      robot_for_control_.get());

  systems::DiagramBuilder<double> builder;

  // Converts raw state to humanoid status.
  RobotKinematicStateTranslatorSystem<double>* rs_wrapper =
      builder.AddSystem<RobotKinematicStateTranslatorSystem<double>>(
          robot_for_control.get());
  // Converts qp output to raw torque.
  QpOutputTranslatorSystem* joint_level_controller =
      builder.AddSystem<QpOutputTranslatorSystem>(*robot_for_control);
  // Generates qp_input from desired q and v vd.
  plan_eval_ = builder.AddSystem<ManipulatorMoveJointPlanEvalSystem>(
      robot_for_control.get(), alias_groups, paramset, dt);
  // Inverse dynamics controller
  QpInverseDynamicsSystem* id_controller =
      builder.AddSystem<QpInverseDynamicsSystem>(robot_for_control.get(), dt);

  // Connects state translator to plan eval.
  builder.Connect(rs_wrapper->get_output_port(),
                  plan_eval_->get_input_port_kinematic_state());

  // Connects state translator to inverse dynamics.
  builder.Connect(rs_wrapper->get_output_port(),
                  id_controller->get_input_port_kinematic_state());

  // Connects plan eval to inverse dynamics.
  builder.Connect(plan_eval_->get_output_port_qp_input(),
                  id_controller->get_input_port_qp_input());

  // Connects inverse dynamics to torque translator.
  builder.Connect(id_controller->get_output_port_qp_output(),
                  joint_level_controller->get_input_port_qp_output());

  // Exposes raw estimated state input.
  input_port_index_estimated_state_ =
      builder.ExportInput(rs_wrapper->get_input_port());

  // Exposes desired q + vd input.
  input_port_index_desired_state_ =
      builder.ExportInput(plan_eval_->get_input_port_desired_state());

  // Exposes desired vd input.
  input_port_index_desired_acceleration_ =
      builder.ExportInput(plan_eval_->get_input_port_desired_acceleration());

  // Exposes raw torque output.
  output_port_index_control_ =
      builder.ExportOutput(joint_level_controller->get_output_port_torque());

  // Exposes plan eval's debug output.
  output_port_index_plan_eval_debug_ =
      builder.ExportOutput(plan_eval_->get_output_port_debug_info());

  // Exposes plan eval's QpInput output.
  output_port_index_qp_input_ =
      builder.ExportOutput(plan_eval_->get_output_port_qp_input());

  // Exposes inverse dynamics' debug output.
  output_port_index_inverse_dynamics_debug_ =
      builder.ExportOutput(plan_eval_->get_output_port_debug_info());

  // Exposes inverse dynamics' QpOutput output.
  output_port_index_qp_output_ =
      builder.ExportOutput(id_controller->get_output_port_qp_output());

  builder.BuildInto(this);
}

void ManipulatorJointSpaceController::Initialize(
    systems::Context<double>* context) {
  systems::Context<double>& plan_eval_context =
      GetMutableSubsystemContext(*plan_eval_, context);
  systems::State<double>* plan_eval_state =
      plan_eval_context.get_mutable_state();
  plan_eval_->Initialize(plan_eval_state);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
