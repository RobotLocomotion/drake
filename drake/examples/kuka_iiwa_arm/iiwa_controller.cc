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
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

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

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);
  std::unique_ptr<systems::DiscreteState<double>> discrete_state =
      diagram->AllocateDiscreteVariables();
  std::unique_ptr<systems::State<double>> tmp_state = context->CloneState();

  lcm.StartReceiveThread();
  drake::log()->info("controller started");

  // TODO(sam.creasey) All of the remaining code in this function is
  // basically duplicative of a similar implementation in
  // valkyrie_balancing_controller_system.cc, and I think they could
  // easily be merged into a single function templated on the message
  // type.  Do this once we get to rule-of-three.

  // Loops until the first status message arrives.
  while (true) {
    // Sets Context's time to the timestamp in the bot_core::robot_state_t msg.
    const lcmt_iiwa_status* msg =
        status_receiver->EvalInputValue<lcmt_iiwa_status>(
            diagram->GetSubsystemContext(*context, status_receiver), 0);
    context->set_time(static_cast<double>(msg->utime) / 1e6);
    if (context->get_time() != 0) break;
  }
  drake::log()->info("status received");

  // Create an integrator to advance the state of the integrator
  // portion of the PidControlledSystem above.  A basic euler
  // integrator should do, precision is not critical for this
  // application.
  systems::ExplicitEulerIntegrator<double> integrator(
      *diagram, kIiwaLcmStatusPeriod, context.get());
  integrator.Initialize();

  systems::UpdateActions<double> update_actions;
  double next_control_time =
      diagram->CalcNextUpdateTime(*context, &update_actions);

  // Loop forever, continuing to update the time based on the incoming
  // status message.
  while (true) {
    if (next_control_time <= context->get_time()) {
      for (const systems::DiscreteEvent<double>& event :
           update_actions.events) {
        if (event.action ==
            systems::DiscreteEvent<double>::kUnrestrictedUpdateAction) {
          diagram->CalcUnrestrictedUpdate(*context, event, tmp_state.get());
          context->get_mutable_state()->CopyFrom(*tmp_state);
        } else if (event.action ==
                   systems::DiscreteEvent<double>::kDiscreteUpdateAction) {
          diagram->CalcDiscreteVariableUpdates(*context, event,
                                               discrete_state.get());
          context->get_mutable_discrete_state()->CopyFrom(*discrete_state);
        }
      }

      diagram->Publish(*context);
      next_control_time =
          diagram->CalcNextUpdateTime(*context, &update_actions);

      const double dt = next_control_time - context->get_time();
      integrator.StepOnceAtMost(dt, dt, dt);
    }

    // TODO(siyuan): This is a busy polling loop on LCM message. Should switch
    // to a descheduled version.
    // Sets Context's time to the timestamp in the iiwa status msg.
    const lcmt_iiwa_status* msg =
        status_receiver->EvalInputValue<lcmt_iiwa_status>(
            diagram->GetSubsystemContext(*context, status_receiver), 0);
    context->set_time(static_cast<double>(msg->utime) / 1e6);
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
