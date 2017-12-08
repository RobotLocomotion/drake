/// @file
///
/// Implements a controller for a KUKA iiwa arm.

#include <memory>

#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/lcm_plan_interpolator.h"
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
DEFINE_string(interp_type, "cubic",
              "Robot plan interpolation type. Can be {zoh, foh, cubic, pchip}");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_command_channel, "IIWA_COMMAND",
              "Channel on which to publish lcmt_iiwa_command messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to listen for robot_plan_t messages.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using manipulation::planner::RobotPlanInterpolator;
using manipulation::planner::InterpolatorType;

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

// Create a system which has an integrator on the interpolated
// reference position for received plans.
int DoMain() {
  const std::string kLcmStatusChannel = FLAGS_lcm_status_channel;
  const std::string kLcmCommandChannel = FLAGS_lcm_command_channel;
  const std::string kLcmPlanChannel = FLAGS_lcm_plan_channel;
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  drake::log()->debug("Status channel: {}", kLcmStatusChannel);
  drake::log()->debug("Command channel: {}", kLcmCommandChannel);
  drake::log()->debug("Plan channel: {}", kLcmPlanChannel);

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));

  // Sets the robot plan interpolation type.
  std::string interp_str(FLAGS_interp_type);
  std::transform(interp_str.begin(), interp_str.end(),
                 interp_str.begin(), ::tolower);
  InterpolatorType interpolator_type{};
  if (interp_str == "zoh") {
    interpolator_type = InterpolatorType::ZeroOrderHold;
  } else if (interp_str == "foh") {
    interpolator_type = InterpolatorType::FirstOrderHold;
  } else if (interp_str == "cubic") {
    interpolator_type = InterpolatorType::Cubic;
  } else if (interp_str == "pchip") {
    interpolator_type = InterpolatorType::Pchip;
  } else {
    DRAKE_ABORT_MSG(
        "Robot plan interpolation type not recognized. "
        "Use the gflag --helpshort to display "
        "flag options for interpolator type.");
  }
  auto plan_interpolator =
      builder.AddSystem<LcmPlanInterpolator>(urdf, interpolator_type);
  const int kNumJoints = plan_interpolator->num_joints();

  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));
  plan_sub->set_name("plan_sub");

  auto status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
          kLcmStatusChannel, &lcm));
  status_sub->set_name("status_sub");

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
          kLcmCommandChannel, &lcm));
  command_pub->set_name("command_pub");

  // Connect subscribers to input ports.
  builder.Connect(plan_sub->get_output_port(0),
                  plan_interpolator->get_input_port_iiwa_plan());
  builder.Connect(status_sub->get_output_port(0),
                  plan_interpolator->get_input_port_iiwa_status());

  // Connect publisher to output port.
  builder.Connect(plan_interpolator->get_output_port_iiwa_command(),
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
  VectorX<double> q0(kNumJoints);
  DRAKE_DEMAND(kNumJoints == first_status.num_joints);
  for (int i = 0; i < kNumJoints; i++)
    q0[i] = first_status.joint_position_measured[i];

  systems::Context<double>& diagram_context = loop.get_mutable_context();
  systems::Context<double>& status_sub_context =
      diagram->GetMutableSubsystemContext(*status_sub, &diagram_context);
  status_sub->SetDefaultContext(&status_sub_context);

  // Explicit initialization.
  diagram_context.set_time(msg_time);
  auto& plan_interpolator_context =
      diagram->GetMutableSubsystemContext(*plan_interpolator, &diagram_context);
  plan_interpolator->Initialize(msg_time, q0, &plan_interpolator_context);

  loop.RunToSecondsAssumingInitialized();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::DoMain();
}
