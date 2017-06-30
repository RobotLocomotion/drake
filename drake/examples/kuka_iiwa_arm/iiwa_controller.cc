/// @file
///
/// Implements a controller for a KUKA iiwa arm.

#include <memory>

#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"

using robotlocomotion::robot_plan_t;

DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kIiwaUrdf = "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

// Create a system which has an integrator on the interpolated
// reference position for received plans.
int DoMain() {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));
  plan_sub->set_name("plan_sub");

  const std::string urdf = (!FLAGS_urdf.empty() ? FLAGS_urdf :
                            FindResourceOrThrow(kIiwaUrdf));
  auto plan_source =
      builder.AddSystem<RobotPlanInterpolator>(urdf);
  plan_source->set_name("plan_source");
  const int num_joints = plan_source->tree().get_num_positions();

  auto status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
          kLcmStatusChannel, &lcm));
  status_sub->set_name("status_sub");

  auto status_receiver = builder.AddSystem<IiwaStatusReceiver>(num_joints);
  status_receiver->set_name("status_receiver");

  auto target_demux =
      builder.AddSystem<systems::Demultiplexer>(num_joints * 2, num_joints);
  target_demux->set_name("target_demux");

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
          kLcmCommandChannel, &lcm));
  command_pub->set_name("command_pub");

  auto command_sender = builder.AddSystem<IiwaCommandSender>(num_joints);
  command_sender->set_name("command_sender");

  builder.Connect(plan_sub->get_output_port(0),
                  plan_source->get_plan_input_port());
  builder.Connect(status_sub->get_output_port(0),
                  status_receiver->get_input_port(0));
  builder.Connect(status_receiver->get_measured_position_output_port(),
                  plan_source->get_state_input_port());
  builder.Connect(plan_source->get_output_port(0),
                  target_demux->get_input_port(0));
  builder.Connect(target_demux->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port(0));
  auto diagram = builder.Build();

  drake::log()->info("controller started");

  systems::lcm::LcmDrivenLoop loop(
      *diagram, *status_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<lcmt_iiwa_status>>());

  // Waits for the first message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  const lcmt_iiwa_status& first_status = first_msg.GetValue<lcmt_iiwa_status>();
  VectorX<double> q0(num_joints);
  DRAKE_DEMAND(num_joints == first_status.num_joints);
  for (int i = 0; i < num_joints; i++)
    q0[i] = first_status.joint_position_measured[i];

  systems::Context<double>* diagram_context = loop.get_mutable_context();
  systems::Context<double>& status_sub_context =
      diagram->GetMutableSubsystemContext(*status_sub, diagram_context);
  status_sub->SetDefaults(&status_sub_context);

  // Explicit initialization.
  diagram_context->set_time(msg_time);
  auto& plan_source_context =
      diagram->GetMutableSubsystemContext(*plan_source, diagram_context);
  plan_source->Initialize(msg_time, q0,
                          plan_source_context.get_mutable_state());

  loop.RunToSecondsAssumingInitialized();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
