/// @file
///
/// Implements a controller for a Kinova Jaco arm.

#include <memory>

#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kinova_jaco_arm/jaco_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

using robotlocomotion::robot_plan_t;

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_int32(num_joints,
             drake::examples::kinova_jaco_arm::kJacoDefaultArmNumJoints,
             "Number of joints in the arm (not including fingers)");
DEFINE_int32(num_fingers,
             drake::examples::kinova_jaco_arm::kJacoDefaultArmNumFingers,
             "Number of fingers on the arm");

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {
using manipulation::planner::RobotPlanInterpolator;

const char* const kIiwaUrdf =
    "drake/manipulation/models/jaco_description/urdf/j2s7s300.urdf";
const char* const kLcmStatusChannel = "KINOVA_JACO_STATUS";
const char* const kLcmCommandChannel = "KINOVA_JACO_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

int DoMain() {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));

  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));
  auto plan_source = builder.AddSystem<RobotPlanInterpolator>(urdf);

  builder.Connect(plan_sub->get_output_port(0),
                  plan_source->get_plan_input_port());

  const int num_joints = FLAGS_num_joints;
  const int num_fingers = FLAGS_num_fingers;
  DRAKE_DEMAND(plan_source->tree().get_num_positions() ==
               num_joints + num_fingers);
  DRAKE_DEMAND(plan_source->tree().get_num_velocities() ==
               num_joints + num_fingers);

  auto status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_jaco_status>(
          kLcmStatusChannel, &lcm));
  auto status_receiver = builder.AddSystem<JacoStatusReceiver>(
      num_joints, num_fingers);

  builder.Connect(status_sub->get_output_port(0),
                  status_receiver->get_input_port(0));
  builder.Connect(status_receiver->get_output_port(0),
                  plan_source->get_state_input_port());

  // The driver is operating in joint velocity mode, so that's the
  // meaningful part of the command message we'll eventually
  // construct.  We create a pid controller which calculates
  //
  // y = kp * (q_desired - q) + kd * (v_desired - v)
  //
  // (feedback term) which we'll add to v_desired from the plan source
  // (feed forward term).
  Eigen::VectorXd jaco_kp = Eigen::VectorXd::Zero(num_joints + num_fingers);
  Eigen::VectorXd jaco_ki = Eigen::VectorXd::Zero(num_joints + num_fingers);
  Eigen::VectorXd jaco_kd = Eigen::VectorXd::Zero(num_joints + num_fingers);

  // I (sam.creasey) have no idea what would be good values here.
  // This seems to work OK at the low speeds of the jaco.
  jaco_kp.head(num_joints).fill(10);
  jaco_kp.tail(num_fingers).fill(1);
  jaco_kd.head(num_joints).fill(1);

  // The finger velocities reported from the Jaco aren't meaningful,
  // so we shouldn't try to control based on them.
  jaco_kd.tail(num_fingers).fill(0);

  auto pid_controller = builder.AddSystem<systems::controllers::PidController>(
      jaco_kp, jaco_ki, jaco_kd);

  builder.Connect(status_receiver->get_output_port(0),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(plan_source->get_output_port(0),
                  pid_controller->get_input_port_desired_state());

  // Split the plan source into q_d (sent in the command message for
  // informational purposes) and v_d (feed forward term for control).
  auto target_demux =
      builder.AddSystem<systems::Demultiplexer>(
          (num_joints + num_fingers) * 2, num_joints + num_fingers);
  builder.Connect(plan_source->get_output_port(0),
                  target_demux->get_input_port(0));

  // Sum the outputs of the pid controller and v_d.
  auto adder = builder.AddSystem<systems::Adder>(2, num_joints + num_fingers);
  builder.Connect(pid_controller->get_output_port_control(),
                  adder->get_input_port(0));
  builder.Connect(target_demux->get_output_port(1),
                  adder->get_input_port(1));

  // Multiplex the q_d and velocity command streams back into a single
  // port.
  std::vector<int> mux_sizes(2, num_joints + num_fingers);
  auto command_mux = builder.AddSystem<systems::Multiplexer>(mux_sizes);
  builder.Connect(target_demux->get_output_port(0),
                  command_mux->get_input_port(0));
  builder.Connect(adder->get_output_port(),
                  command_mux->get_input_port(1));

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_jaco_command>(
          kLcmCommandChannel, &lcm));
  auto command_sender = builder.AddSystem<JacoCommandSender>(num_joints);
  builder.Connect(command_mux->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port(0));

  auto diagram = builder.Build();

  systems::lcm::LcmDrivenLoop loop(
      *diagram, *status_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<lcmt_jaco_status>>());

  // Waits for the first message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  const lcmt_jaco_status& first_status = first_msg.GetValue<lcmt_jaco_status>();
  DRAKE_DEMAND(first_status.num_joints == 0 ||
               first_status.num_joints == num_joints);
  DRAKE_DEMAND(first_status.num_fingers == 0 ||
               first_status.num_fingers == num_fingers);

  VectorX<double> q0 = VectorX<double>::Zero(num_joints + num_fingers);
  for (int i = 0; i < first_status.num_joints; ++i) {
    q0(i) = first_status.joint_position[i];
  }

  for (int i = 0; i < first_status.num_fingers; ++i) {
    q0(i + num_joints) = first_status.finger_position[i];
  }

  systems::Context<double>& diagram_context = loop.get_mutable_context();
  systems::Context<double>& status_sub_context =
      diagram->GetMutableSubsystemContext(*status_sub, &diagram_context);
  status_sub->SetDefaultContext(&status_sub_context);

  // Explicit initialization.
  diagram_context.set_time(msg_time);
  auto& plan_source_context =
      diagram->GetMutableSubsystemContext(*plan_source, &diagram_context);
  plan_source->Initialize(msg_time, q0,
                          &plan_source_context.get_mutable_state());

  loop.RunToSecondsAssumingInitialized();
  return 0;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kinova_jaco_arm::DoMain();
}
