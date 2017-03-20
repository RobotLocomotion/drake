/// @file
///
/// Implements a controller for a KUKA iiwa arm.

#include <memory>

#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_plan_source.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"

using robotlocomotion::robot_plan_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kIiwaUrdf =
    "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf";
const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const int kNumJoints = 7;

// Create a system which has an integrator on the interpolated
// reference position for received plans.
int DoMain() {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
          kLcmStatusChannel, &lcm));
  auto status_receiver = builder.AddSystem<IiwaStatusReceiver>();
  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));
  auto plan_source =
      builder.AddSystem<IiwaPlanSource>(GetDrakePath() + kIiwaUrdf);
  auto target_demux =
      builder.AddSystem<systems::Demultiplexer>(kNumJoints * 2, kNumJoints);
  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
          kLcmCommandChannel, &lcm));
  command_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto command_sender = builder.AddSystem<IiwaCommandSender>();

  builder.Connect(plan_sub->get_output_port(0),
                  plan_source->get_plan_input_port());
  builder.Connect(status_sub->get_output_port(0),
                  plan_source->get_status_input_port());

  builder.Connect(status_sub->get_output_port(0),
                  status_receiver->get_input_port(0));

  Eigen::VectorXd Kp = Eigen::VectorXd::Zero(kNumJoints);
  Eigen::VectorXd Ki = Eigen::VectorXd::Ones(kNumJoints) * 0.05;
  Eigen::VectorXd Kd = Eigen::VectorXd::Zero(kNumJoints);

  // q_d = q_ref + ki * int (q_d - q).
  auto control_ports = systems::PidControlledSystem<double>::ConnectController(
      command_sender->get_input_port(0),
      status_receiver->get_measured_position_output_port(), nullptr, Kp, Ki, Kd,
      &builder);
  builder.Connect(plan_source->get_output_port(0),
                  target_demux->get_input_port(0));
  builder.Connect(target_demux->get_output_port(0),
                  control_ports.control_input_port);
  builder.Connect(plan_source->get_output_port(0),
                  control_ports.state_input_port);
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port(0));
  auto diagram = builder.Build();

  lcm.StartReceiveThread();
  drake::log()->info("controller started");

  // Loops until the first status message arrives.
  std::unique_ptr<systems::Context<double>> initial_context =
      diagram->CreateDefaultContext();
  while (true) {
    // Sets Context's time to the timestamp in the bot_core::robot_state_t msg.
    const lcmt_iiwa_status* msg =
        status_receiver->EvalInputValue<lcmt_iiwa_status>(
            diagram->GetSubsystemContext(*initial_context, status_receiver), 0);
    initial_context->set_time(static_cast<double>(msg->utime) / 1e6);
    if (initial_context->get_time() != 0) break;
  }
  double iiwa_time = initial_context->get_time();
  drake::log()->info("status received");

  systems::Simulator<double> simulator(*diagram, std::move(initial_context));
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  // Loop forever, continuing to update the simulation based on the incoming
  // status message.
  while (true) {
    while (iiwa_time <= simulator.get_context().get_time()) {
      const lcmt_iiwa_status* msg =
          status_receiver->EvalInputValue<lcmt_iiwa_status>(
              diagram->GetSubsystemContext(
                  simulator.get_context(), status_receiver), 0);
      iiwa_time = static_cast<double>(msg->utime) / 1e6;
    }
    simulator.StepTo(iiwa_time);
  }
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
