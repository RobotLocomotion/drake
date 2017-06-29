#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/valkyrie_controller.h"
#include "drake/examples/Valkyrie/valkyrie_constants.h"
#include "drake/systems/lcm/lcm_driven_loop.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// This is an example qp based inverse dynamics controller loop for Valkyrie
// built from the Systems blocks.
//
// The overall input and output is a LCM message of type
// bot_core::robot_state_t and bot_core::atlas_command_t.
void controller_loop() {
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

  drake::lcm::DrakeLcm lcm;
  ValkyrieController valkyrie_controller(kModelFileName, kControlConfigPath,
                                         kAliasGroupPath, &lcm);
  HumanoidPlanEvalSystem* plan_eval =
      valkyrie_controller.get_mutable_plan_eval();
  const systems::lcm::LcmSubscriberSystem& state_msg_subscriber =
      valkyrie_controller.get_state_msg_subscriber();

  // Makes a Lcm driven loop that's blocked by robot_state_subscriber.
  systems::lcm::LcmDrivenLoop loop(
      valkyrie_controller, state_msg_subscriber, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<bot_core::robot_state_t>>());

  // Do initialization based on the first received message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  loop.get_mutable_context()->set_time(msg_time);

  // Sets plan eval's desired to the nominal state.
  systems::Context<double>& plan_eval_context =
      valkyrie_controller.GetMutableSubsystemContext(*plan_eval,
          loop.get_mutable_context());
  VectorX<double> desired_q =
      valkyrie::RPYValkyrieFixedPointState().head(valkyrie::kRPYValkyrieDof);
  plan_eval->Initialize(desired_q, plan_eval_context.get_mutable_state());

  // Starts the loop.
  loop.RunToSecondsAssumingInitialized();
}

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake

int main() { drake::examples::qp_inverse_dynamics::controller_loop(); }
