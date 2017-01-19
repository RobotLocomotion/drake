#include <iostream>
#include <memory>
#include <thread>

#include "bot_core/atlas_command_t.hpp"
#include "bot_core/robot_state_t.hpp"
#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_state_decoder_system.h"
#include "drake/examples/Valkyrie/valkyrie_constants.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// This is an example qp based inverse dynamics controller loop for Valkyrie
// built from the system2 blocks.
//
// The overall input and output is a lcm message of type
// bot_core::robot_state_t and bot_core::atlas_command_t.
void controller_loop() {
  // Loads model.
  std::string urdf = drake::GetDrakePath() + "/examples/Valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kRollPitchYaw, robot.get());

  systems::DiagramBuilder<double> builder;

  lcm::DrakeLcm lcm;

  RobotStateDecoderSystem* rs_msg_to_rs =
      builder.AddSystem(std::make_unique<RobotStateDecoderSystem>(*robot));
  PlanEvalSystem* plan_eval =
      builder.AddSystem(std::make_unique<PlanEvalSystem>(*robot));
  QPControllerSystem* qp_con =
      builder.AddSystem(std::make_unique<QPControllerSystem>(*robot));
  JointLevelControllerSystem* joint_con =
      builder.AddSystem<JointLevelControllerSystem>(*robot);

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
  // rs + qp_output -> atlas_command_t
  builder.Connect(rs_msg_to_rs->get_output_port_humanoid_status(),
                  joint_con->get_input_port_humanoid_status());
  builder.Connect(qp_con->get_output_port_qp_output(),
                  joint_con->get_input_port_qp_output());
  // atlas_command_t -> lcm
  builder.Connect(joint_con->get_output_port_atlas_command(),
                  atlas_command_publisher.get_input_port(0));

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);
  systems::UpdateActions<double> actions;

  systems::Context<double>* plan_eval_context =
      diagram->GetMutableSubsystemContext(context.get(), plan_eval);
  systems::Context<double>* qp_controller_context =
      diagram->GetMutableSubsystemContext(context.get(), qp_con);

  systems::State<double>* plan_eval_state =
      plan_eval_context->get_mutable_state();
  systems::State<double>* qp_controller_state =
      qp_controller_context->get_mutable_state();

  // Set plan eval's desired to the nominal state.
  DRAKE_DEMAND(valkyrie::kRPYValkyrieDof == robot->get_num_positions());
  VectorX<double> desired_q =
      valkyrie::RPYValkyrieFixedPointState().head(valkyrie::kRPYValkyrieDof);
  plan_eval->SetDesired(desired_q, plan_eval_state);

  lcm.StartReceiveThread();

  std::cout << "controller started\n";

  systems::UpdateActions<double> update_actions;
  double next_control_time = context->get_time();

  while (true) {
    // Computes control.
    if (next_control_time <= context->get_time()) {
      // TODO(siyuanfeng): should directly call on diagrams stuff when diagram can handle unrestriced updates.
      plan_eval->DoCalcUnrestrictedUpdate(*plan_eval_context, plan_eval_state);
      qp_con->DoCalcUnrestrictedUpdate(*qp_controller_context, qp_controller_state);

      next_control_time = plan_eval->CalcNextUpdateTime(*plan_eval_context, &update_actions);

      /*
      for (const systems::DiscreteEvent<double>& event : update_actions.events) {
        if (event.action == systems::DiscreteEvent<double>::kUnrestrictedUpdateAction) {
          systems::State<double>* x = context->get_mutable_state();
          DRAKE_DEMAND(x != nullptr);
          diagram->CalcUnrestrictedUpdate(*context, event, x);
        }
      }
      update_actions.events.clear();
      next_control_time = diagram->CalcNextUpdateTime(*context, &update_actions);
      std::cout << next_control_time << " " << context->get_time() << std::endl;
      */

      // Sends the bot_core::atlas_command_t msg.
      diagram->Publish(*context);
    }

    // Sets Context's time to the timestamp in the bot_core::robot_state_t msg.
    const bot_core::robot_state_t* msg =
        rs_msg_to_rs->EvalInputValue<bot_core::robot_state_t>(
            diagram->GetSubsystemContext(*context, rs_msg_to_rs), 0);
    context->set_time(static_cast<double>(msg->utime) / 1e6);
  }
}

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake

int main() { drake::examples::qp_inverse_dynamics::controller_loop(); }
