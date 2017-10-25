#include "drake/common/find_resource.h"
#include "drake/examples/humanoid_controller/humanoid_controller.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/systems/lcm/lcm_driven_loop.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

// This is an example qp based inverse dynamics controller loop for Valkyrie
// built from the Systems blocks.
//
// The overall input and output is a LCM message of type
// bot_core::robot_state_t and bot_core::atlas_command_t.
void controller_loop() {
  const std::string kModelFileName = FindResourceOrThrow(
      "drake/examples/valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  const std::string kAliasGroupPath = FindResourceOrThrow(
      "drake/examples/humanoid_controller/"
      "config/valkyrie.alias_groups");
  const std::string kControlConfigPath = FindResourceOrThrow(
      "drake/examples/humanoid_controller/"
      "config/valkyrie.id_controller_config");

  drake::lcm::DrakeLcm lcm;
  HumanoidController valkyrie_controller(kModelFileName, kControlConfigPath,
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

  // Initializes based on the first received message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  loop.get_mutable_context().set_time(msg_time);

  // Decodes the message into q and v.
  const bot_core::robot_state_t& raw_msg =
      first_msg.GetValueOrThrow<bot_core::robot_state_t>();
  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelFileName, multibody::joints::kRollPitchYaw, &robot);
  RigidBodyTreeAliasGroups<double> alias_groups(&robot);
  alias_groups.LoadFromFile(kAliasGroupPath);

  VectorX<double> q(robot.get_num_positions());
  VectorX<double> v(robot.get_num_velocities());
  manipulation::RobotStateLcmMessageTranslator translator(robot);
  translator.DecodeMessageKinematics(raw_msg, q, v);

  HumanoidStatus robot_status(&robot, alias_groups);
  v.setZero();
  robot_status.UpdateKinematics(msg_time, q, v);

  // Sets plan eval's desired to the measured state.
  systems::Context<double>& plan_eval_context =
      valkyrie_controller.GetMutableSubsystemContext(
          *plan_eval, &loop.get_mutable_context());
  plan_eval->Initialize(robot_status, &plan_eval_context.get_mutable_state());

  // Starts the loop.
  loop.RunToSecondsAssumingInitialized();
}

}  // namespace humanoid_controller
}  // namespace examples
}  // end namespace drake

int main() { drake::examples::humanoid_controller::controller_loop(); }
