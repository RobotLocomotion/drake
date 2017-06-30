/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;

int DoMain() {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const char* kModelPath = "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf = (!FLAGS_urdf.empty() ? FLAGS_urdf :
                            FindResourceOrThrow(kModelPath));
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf, multibody::joints::kFixed, tree.get());
    plant = builder.AddPlant(std::move(tree));
  }
  // Creates and adds LCM publisher for visualization.
  builder.AddVisualizer(&lcm);
  builder.get_visualizer()->set_publish_period(kIiwaLcmStatusPeriod);

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  const int num_joints = tree.get_num_positions();

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

  DRAKE_DEMAND(tree.get_num_positions() % kIiwaArmNumJoints == 0);
  for (int offset = kIiwaArmNumJoints; offset < tree.get_num_positions();
       offset += kIiwaArmNumJoints) {
    const int end = offset + kIiwaArmNumJoints;
    iiwa_kp.conservativeResize(end);
    iiwa_kp.segment(offset, kIiwaArmNumJoints) =
        iiwa_kp.head(kIiwaArmNumJoints);
    iiwa_ki.conservativeResize(end);
    iiwa_ki.segment(offset, kIiwaArmNumJoints) =
        iiwa_ki.head(kIiwaArmNumJoints);
    iiwa_kd.conservativeResize(end);
    iiwa_kd.segment(offset, kIiwaArmNumJoints) =
        iiwa_kd.head(kIiwaArmNumJoints);
  }

  auto controller =
      builder.AddController<systems::InverseDynamicsController<double>>(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
          tree.Clone(), iiwa_kp, iiwa_ki, iiwa_kd,
          false /* without feedforward acceleration */);

  // Create the command subscriber and status publisher.
  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();
  auto command_sub = base_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>(
          "IIWA_COMMAND", &lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      base_builder->AddSystem<IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");
  auto status_pub = base_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
          "IIWA_STATUS", &lcm));
  status_pub->set_name("status_publisher");
  status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto status_sender = base_builder->AddSystem<IiwaStatusSender>(num_joints);
  status_sender->set_name("status_sender");

  base_builder->Connect(command_sub->get_output_port(0),
                  command_receiver->get_input_port(0));
  base_builder->Connect(command_receiver->get_output_port(0),
                  controller->get_input_port_desired_state());
  base_builder->Connect(plant->get_output_port(0),
                  status_sender->get_state_input_port());
  base_builder->Connect(command_receiver->get_output_port(0),
                  status_sender->get_command_input_port());
  base_builder->Connect(status_sender->get_output_port(0),
                  status_pub->get_input_port(0));
  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  command_receiver->set_initial_position(
      &sys->GetMutableSubsystemContext(*command_receiver,
                                       simulator.get_mutable_context()),
      VectorX<double>::Zero(tree.get_num_positions()));


  // Simulate for a very long time.
  simulator.StepTo(FLAGS_simulation_sec);

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
