#include <iostream>
#include <memory>
#include <thread>

#include "bot_core/atlas_command_t.hpp"
#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/common/text_logging.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/atlas_joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_status_translator_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/Valkyrie/valkyrie_constants.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// This is an example qp based inverse dynamics controller loop for Valkyrie
// built from the Systems blocks.
//
// The overall input and output is a LCM message of type
// bot_core::robot_state_t and bot_core::atlas_command_t.
void controller_loop() {
  DRAKE_ABORT_MSG("this example is temporarily broken, and will be "
      "fixed after #5672 is merged");

  const std::string kModelFileName =
      drake::GetDrakePath() +
      "/examples/Valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
  const std::string kAliasGroupPath = drake::GetDrakePath() +
                                      "/examples/QPInverseDynamicsForHumanoids/"
                                      "config/valkyrie.alias_groups";
  const std::string kControlConfigPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/"
      "config/valkyrie.id_controller_config";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelFileName, multibody::joints::kRollPitchYaw, robot.get());

  systems::DiagramBuilder<double> builder;
  lcm::DrakeLcm lcm;

  RobotStateMsgToHumanoidStatusSystem* rs_msg_to_rs =
      builder.AddSystem(std::make_unique<RobotStateMsgToHumanoidStatusSystem>(
          *robot, kAliasGroupPath));
  HumanoidPlanEvalSystem* plan_eval =
      builder.AddSystem(std::make_unique<HumanoidPlanEvalSystem>(
          *robot, kAliasGroupPath, kControlConfigPath, 0.003));
  QpControllerSystem* qp_con =
      builder.AddSystem(std::make_unique<QpControllerSystem>(*robot, 0.003));
  AtlasJointLevelControllerSystem* joint_con =
      builder.AddSystem<AtlasJointLevelControllerSystem>(*robot);

  auto& robot_state_subscriber = *builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));
  auto& atlas_command_publisher = *builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<bot_core::atlas_command_t>(
          "ROBOT_COMMAND", &lcm));

  // lcm -> rs
  builder.Connect(robot_state_subscriber.get_output_port(0),
                  rs_msg_to_rs->get_input_port_robot_state_msg());
  // rs -> qp_input
  builder.Connect(rs_msg_to_rs->get_output_port_humanoid_status(),
                  plan_eval->get_input_port_humanoid_status());
  // rs + qp_input -> qp_output
  builder.Connect(rs_msg_to_rs->get_output_port_humanoid_status(),
                  qp_con->get_input_port_humanoid_status());
  builder.Connect(plan_eval->get_output_port_qp_input(),
                  qp_con->get_input_port_qp_input());
  // qp_output -> atlas_command_t
  builder.Connect(qp_con->get_output_port_qp_output(),
                  joint_con->get_input_port_qp_output());
  // atlas_command_t -> lcm
  builder.Connect(joint_con->get_output_port_atlas_command(),
                  atlas_command_publisher.get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Makes a Lcm driven loop that's blocked by robot_state_subscriber.
  systems::lcm::LcmDrivenLoop loop(
      *diagram, robot_state_subscriber, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<bot_core::robot_state_t>>());

  // Do initialization based on the first received message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  loop.get_mutable_context()->set_time(msg_time);

  // Sets plan eval's desired to the nominal state.
  systems::Context<double>* plan_eval_context =
      diagram->GetMutableSubsystemContext(loop.get_mutable_context(),
                                          plan_eval);
  DRAKE_DEMAND(valkyrie::kRPYValkyrieDof == robot->get_num_positions());
  VectorX<double> desired_q =
      valkyrie::RPYValkyrieFixedPointState().head(valkyrie::kRPYValkyrieDof);
  plan_eval->Initialize(desired_q, plan_eval_context->get_mutable_state());

  // Starts the loop.
  loop.RunToSecondsAssumingInitialized();
}

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake

int main() { drake::examples::qp_inverse_dynamics::controller_loop(); }
