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
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
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

  builder.Connect(plan_sub->get_output_port(),
                  plan_source->get_plan_input_port());

  const int num_joints = FLAGS_num_joints;
  const int num_fingers = FLAGS_num_fingers;
  DRAKE_DEMAND(plan_source->plant().num_positions() ==
               num_joints + num_fingers);
  DRAKE_DEMAND(plan_source->plant().num_velocities() ==
               num_joints + num_fingers);

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

  // We'll directly fix the input to the status receiver later from our lcm
  // subscriber.
  auto status_receiver = builder.AddSystem<JacoStatusReceiver>(
      num_joints, num_fingers);

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
                  command_pub->get_input_port());

  auto owned_diagram = builder.Build();
  const systems::Diagram<double>* diagram = owned_diagram.get();
  systems::Simulator<double> simulator(std::move(owned_diagram));

  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_jaco_status");
  lcm::Subscriber<lcmt_jaco_status> status_sub(&lcm, kLcmStatusChannel);
  LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });

  const lcmt_jaco_status& first_status = status_sub.message();
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

  systems::Context<double>& diagram_context = simulator.get_mutable_context();
  const double t0 = first_status.utime * 1e-6;
  diagram_context.SetTime(t0);

  auto& plan_source_context =
      diagram->GetMutableSubsystemContext(*plan_source, &diagram_context);
  plan_source->Initialize(t0, q0,
                          &plan_source_context.get_mutable_state());

  systems::Context<double>& status_context =
      diagram->GetMutableSubsystemContext(*status_receiver, &diagram_context);
  auto& status_value = status_receiver->get_input_port(0).FixValue(
      &status_context, first_status);

  // Run forever, using the lcmt_jaco_status message to dictate when simulation
  // time advances.  The robot_plan_t message is handled whenever the next
  // lcmt_jaco_status occurs.
  drake::log()->info("Controller started");
  while (true) {
    // Wait for an lcmt_jaco_status message.
    status_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm, [&]() { return status_sub.count() > 0; });
    // Write the lcmt_jaco_status message into the context and advance.
    status_value.GetMutableData()->set_value(status_sub.message());
    const double time = status_sub.message().utime * 1e-6;
    simulator.AdvanceTo(time);
    // Force-publish the lcmt_jaco_command (via the command_pub system within
    // the diagram).
    diagram->Publish(diagram_context);
  }

  // We should never reach here.
  return EXIT_FAILURE;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kinova_jaco_arm::DoMain();
}
