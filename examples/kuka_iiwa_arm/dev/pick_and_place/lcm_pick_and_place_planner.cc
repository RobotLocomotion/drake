#include <string>

#include <gflags/gflags.h>
#include "optitrack/optitrack_frame_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_planner.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration_parsing.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_int32(task_index, 0,
             "Index of the task to use from the configuration file.");
DEFINE_bool(use_channel_suffix, false,
            "If true, append a suffix to channel names");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/pick_and_place/configuration/"
              "yellow_posts.pick_and_place_configuration",
              "Path to the configuration file.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

int DoMain(void) {
  // Parse the configuration file.
  const pick_and_place::PlannerConfiguration planner_configuration =
      pick_and_place::ParsePlannerConfigurationOrThrow(
          FLAGS_configuration_file,
          pick_and_place::TaskIndex(FLAGS_task_index));

  const pick_and_place::OptitrackConfiguration optitrack_configuration =
      pick_and_place::ParseOptitrackConfigurationOrThrow(
          FLAGS_configuration_file);

  // Instantiate an LCM instance for use with publishers and subscribers.
  lcm::DrakeLcm lcm;

  // Begin setting up the diagram.
  systems::DiagramBuilder<double> builder;

  // Add the LcmPlanner block, which contains all of the demo logic.
  auto planner = builder.AddSystem<pick_and_place::LcmPlanner>(
      planner_configuration, optitrack_configuration, false);

  // All LCM traffic for this planner will occur on channels that end with the
  // suffix specified below.
  std::string suffix =
      (FLAGS_use_channel_suffix)
          ? "_" + std::to_string(planner_configuration.robot_index)
          : "";

  // Add LCM subscribers.
  auto iiwa_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
          "IIWA_STATUS" + suffix, &lcm));
  auto wsg_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS" + suffix, &lcm));
  auto optitrack_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<optitrack::optitrack_frame_t>(
          "OPTITRACK_FRAMES", &lcm));

  // Add LCM publishers.
  auto iiwa_plan_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::robot_plan_t>(
          "COMMITTED_ROBOT_PLAN" + suffix, &lcm,
          planner_configuration.period_sec /* publish period */));
  auto wsg_command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND" + suffix, &lcm,
          planner_configuration.period_sec /* publish period */));

  // Connect subscribers to planner input ports.
  builder.Connect(wsg_status_sub->get_output_port(),
                  planner->get_input_port_wsg_status());
  builder.Connect(iiwa_status_sub->get_output_port(),
                  planner->get_input_port_iiwa_status());
  builder.Connect(optitrack_sub->get_output_port(),
                  planner->get_input_port_optitrack_message());

  // Connect publishers to planner output ports.
  builder.Connect(planner->get_output_port_iiwa_plan(),
                  iiwa_plan_pub->get_input_port());
  builder.Connect(planner->get_output_port_wsg_command(),
                  wsg_command_pub->get_input_port());

  // Build the diagram.
  auto sys = builder.Build();

  // Create and configure the lcm-driven loop.
  systems::lcm::LcmDrivenLoop loop(
      *sys, *iiwa_status_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<lcmt_iiwa_status>>());
  loop.set_publish_on_every_received_message(false);

  // Wait for messages from each subscriber before proceeding.
  drake::log()->info("Waiting for optitrack message ...");
  optitrack_sub->WaitForMessage(0);
  drake::log()->info("Optitrack message received.");
  drake::log()->info("Waiting for WSG status message ...");
  wsg_status_sub->WaitForMessage(0);
  drake::log()->info("WSG status message received.");
  drake::log()->info("Waiting for IIWA status message ...");
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  drake::log()->info("IIWA status message received.");
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);

  // Initialize and run the lcm-driven loop.
  loop.get_mutable_context().set_time(msg_time);
  loop.RunToSecondsAssumingInitialized();

  return 0;
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::pick_and_place::DoMain();
}
