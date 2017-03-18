#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_inverse_dynamics_controller.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_status_translator_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

ManipulatorInverseDynamicsController::ManipulatorInverseDynamicsController(
    const std::string& model_path, const std::string& alias_group_path,
    const std::string& controller_config_path, double dt,
    std::shared_ptr<RigidBodyFrame<double>> world_offset)
    : systems::ModelBasedController<double>(model_path, world_offset,
                                            multibody::joints::kFixed) {
  const RigidBodyTree<double>& robot = get_robot_for_control();

  this->set_name("ManipulatorInverseDynamicsController");

  systems::DiagramBuilder<double> builder;

  // Converts raw state to humanoid status.
  StateToHumanoidStatusSystem* rs_wrapper =
      builder.AddSystem<StateToHumanoidStatusSystem>(robot, alias_group_path);
  // Converts qp output to raw torque.
  TrivialJointLevelControllerSystem* joint_level_controller =
      builder.AddSystem<TrivialJointLevelControllerSystem>(robot);
  // Generates qp_input from desired q and v vd.
  plan_eval_ = builder.AddSystem<ManipulatorPlanEvalSystem>(
      robot, alias_group_path, controller_config_path, dt);
  // Inverse dynamics controller
  QpControllerSystem* id_controller =
      builder.AddSystem<QpControllerSystem>(robot, dt);

  // Connects state translator to plan eval.
  builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
                  plan_eval_->get_input_port_humanoid_status());

  // Connects state translator to inverse dynamics.
  builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
                  id_controller->get_input_port_humanoid_status());

  // Connects plan eval to inverse dynamics.
  builder.Connect(plan_eval_->get_output_port_qp_input(),
                  id_controller->get_input_port_qp_input());

  // Connects inverse dynamics to torque translator.
  builder.Connect(id_controller->get_output_port_qp_output(),
                  joint_level_controller->get_input_port_qp_output());

  // Exposes raw estimated state input.
  int index = builder.ExportInput(rs_wrapper->get_input_port_state());
  this->set_input_port_index_estimated_state(index);

  // Exposes desired q + vd input.
  index = builder.ExportInput(plan_eval_->get_input_port_desired_state());
  this->set_input_port_index_desired_state(index);

  // Exposes desired vd input.
  input_port_index_desired_acceleration_ =
      builder.ExportInput(plan_eval_->get_input_port_desired_acceleration());

  // Exposes raw torque output.
  index =
      builder.ExportOutput(joint_level_controller->get_output_port_torque());
  this->set_output_port_index_control(index);

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

void ManipulatorInverseDynamicsController::Initialize(
    systems::Context<double>* context) {
  systems::Context<double>* plan_eval_context =
      GetMutableSubsystemContext(context, plan_eval_);
  systems::State<double>* plan_eval_state =
      plan_eval_context->get_mutable_state();
  plan_eval_->Initialize(plan_eval_state);
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
