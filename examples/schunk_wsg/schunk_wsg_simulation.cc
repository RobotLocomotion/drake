/// @file
///
/// Implements a simulation of the Schunk WSG 50 gripper.  Like the
/// driver for the physical gripper, this simulation communicates over
/// LCM using the lcmt_schunk_status and lcmt_schunk_command messages.
/// It is intended to be a direct replacement for the Schunk WSG
/// driver.

#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/schunk_wsg/simulated_schunk_wsg_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

using manipulation::schunk_wsg::SchunkWsgStatusSender;
using manipulation::schunk_wsg::SchunkWsgController;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;

int DoMain() {
  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem(CreateSimulatedSchunkWsgSystem<double>());
  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer =
      builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  visualizer->set_name("visualizer");
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::MakeFixedSize(
          lcmt_schunk_wsg_command{}, "SCHUNK_WSG_COMMAND", &lcm));
  command_sub->set_name("command_subscriber");

  auto wsg_controller = builder.AddSystem<SchunkWsgController>();

  auto status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", &lcm,
          manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod
              /* publish period */));
  status_pub->set_name("status_publisher");

  auto status_sender = builder.AddSystem<SchunkWsgStatusSender>();
  auto mbp_state_to_wsg_state = builder.AddSystem(
      manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
  status_sender->set_name("status_sender");

  builder.Connect(command_sub->get_output_port(),
                  wsg_controller->GetInputPort("command_message"));
  builder.Connect(wsg_controller->GetOutputPort("force"),
                  plant->actuator_command_input_port());
  builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));
  builder.Connect(plant->state_output_port(),
                  mbp_state_to_wsg_state->get_input_port());
  builder.Connect(mbp_state_to_wsg_state->get_output_port(),
                  status_sender->get_state_input_port());
  builder.Connect(plant->state_output_port(),
                  wsg_controller->GetInputPort("state"));
  builder.Connect(*status_sender, *status_pub);
  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_sec);
  lcm.StopReceiveThread();
  return 0;
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::schunk_wsg::DoMain();
}
