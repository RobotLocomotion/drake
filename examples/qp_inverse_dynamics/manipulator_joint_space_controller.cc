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

ManipulatorJointSpaceController::ManipulatorJointSpaceController(
    const std::string& model_path, const std::string& alias_group_path,
    const std::string& controller_config_path, double dt,
    std::shared_ptr<RigidBodyFrame<double>> world_offset) {
  robot_for_control_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, multibody::joints::kFixed, world_offset,
      robot_for_control_.get());

  const RigidBodyTree<double>& robot = *robot_for_control_;

  systems::DiagramBuilder<double> builder;

  // Converts raw state to humanoid status.
  RobotKinematicStateTranslatorSystem<double>* rs_wrapper =
      builder.AddSystem<RobotKinematicStateTranslatorSystem<double>>(&robot);
  // Converts qp output to raw torque.
  QpOutputTranslatorSystem* joint_level_controller =
      builder.AddSystem<QpOutputTranslatorSystem>(robot);
  // Generates qp_input from desired q and v vd.
  plan_eval_ = builder.AddSystem<ManipulatorMoveJointPlanEvalSystem>(
      &robot, alias_group_path, controller_config_path, dt);
  // Inverse dynamics controller
  QpInverseDynamicsSystem* id_controller =
      builder.AddSystem<QpInverseDynamicsSystem>(&robot, dt);

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
  systems::State<double>& plan_eval_state =
      plan_eval_context.get_mutable_state();
  plan_eval_->Initialize(&plan_eval_state);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
