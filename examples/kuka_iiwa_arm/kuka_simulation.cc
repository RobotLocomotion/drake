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
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_bool(visualize_frames, true, "Visualize end effector frames");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(torque_control, false, "Simulate using torque control mode.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using manipulation::util::SimDiagramBuilder;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::FrameVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::controllers::rbt::InverseDynamicsController;
using systems::controllers::StateFeedbackControllerInterface;

int DoMain() {
  SimDiagramBuilder<double> builder;
  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kModelPath));
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf, multibody::joints::kFixed, tree.get());
    multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
    plant = builder.AddPlant(std::move(tree));
  }
  // Creates and adds LCM publisher for visualization.
  auto lcm = base_builder->AddSystem<systems::lcm::LcmInterfaceSystem>();
  builder.AddVisualizer(lcm);
  builder.get_visualizer()->set_publish_period(kIiwaLcmStatusPeriod);

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  const int num_joints = tree.get_num_positions();
  DRAKE_DEMAND(num_joints % kIiwaArmNumJoints == 0);
  const int num_iiwa = num_joints/kIiwaArmNumJoints;

  // Adds a iiwa controller.
  StateFeedbackControllerInterface<double>* controller = nullptr;
  if (FLAGS_torque_control) {
    VectorX<double> stiffness, damping_ratio;
    SetTorqueControlledIiwaGains(&stiffness, &damping_ratio);
    stiffness = stiffness.replicate(num_iiwa, 1);
    damping_ratio = damping_ratio.replicate(num_iiwa, 1);
    controller = builder.AddController<KukaTorqueController<double>>(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, tree.Clone(),
        stiffness, damping_ratio);
  } else {
    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
    SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
    iiwa_kp = iiwa_kp.replicate(num_iiwa, 1);
    iiwa_kd = iiwa_kd.replicate(num_iiwa, 1);
    iiwa_ki = iiwa_ki.replicate(num_iiwa, 1);
    controller = builder.AddController<InverseDynamicsController<double>>(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId, tree.Clone(),
        iiwa_kp, iiwa_ki, iiwa_kd,
        false /* without feedforward acceleration */);
  }

  // Create the command subscriber and status publisher.
  auto command_sub = base_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      base_builder->AddSystem<IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");
  std::vector<int> iiwa_instances =
      {RigidBodyTreeConstants::kFirstNonWorldModelInstanceId};
  auto external_torque_converter =
      base_builder->AddSystem<IiwaContactResultsToExternalTorque>(
          tree, iiwa_instances);
  auto status_pub = base_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, kIiwaLcmStatusPeriod /* publish period */));
  status_pub->set_name("status_publisher");
  auto status_sender = base_builder->AddSystem<IiwaStatusSender>(num_joints);
  status_sender->set_name("status_sender");

  base_builder->Connect(command_sub->get_output_port(),
                        command_receiver->get_input_port());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // TODO(jwnimmer-tri) The IIWA LCM systems should not know about velocities,
  // we should add velocity estimation into this simulation, not use state
  // ports on the LCM systems (the KUKA doesn't use velocities).
  base_builder->Connect(command_receiver->get_commanded_state_output_port(),
                        controller->get_input_port_desired_state());
  base_builder->Connect(plant->get_output_port(0),
                        status_sender->get_state_input_port());
#pragma GCC diagnostic pop
  base_builder->Connect(command_receiver->get_commanded_position_output_port(),
                        status_sender->get_position_commanded_input_port());
  base_builder->Connect(controller->get_output_port_control(),
                        status_sender->get_torque_commanded_input_port());
  base_builder->Connect(plant->torque_output_port(),
                        status_sender->get_torque_measured_input_port());
  base_builder->Connect(plant->contact_results_output_port(),
                        external_torque_converter->get_input_port(0));
  base_builder->Connect(external_torque_converter->get_output_port(0),
                        status_sender->get_torque_external_input_port());
  base_builder->Connect(status_sender->get_output_port(),
                        status_pub->get_input_port());
  // Connect the torque input in torque control
  if (FLAGS_torque_control) {
    KukaTorqueController<double>* torque_controller =
        dynamic_cast<KukaTorqueController<double>*>(controller);
    DRAKE_DEMAND(torque_controller);
    base_builder->Connect(command_receiver->get_commanded_torque_output_port(),
                          torque_controller->get_input_port_commanded_torque());
  }

  if (FLAGS_visualize_frames) {
    // TODO(sam.creasey) This try/catch block is here because even
    // though RigidBodyTree::FindBody returns a pointer and could return
    // null to indicate failure, it throws instead.  At any rate, warn
    // instead of dying if the links aren't named as expected.  This
    // happens (for example) when loading
    // dual_iiwa14_polytope_collision.urdf.
    try {
    // Visualizes the end effector frame and 7th body's frame.
      std::vector<RigidBodyFrame<double>> local_transforms;
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_ee", tree.FindBody("iiwa_link_ee"),
                                 Isometry3<double>::Identity()));
      local_transforms.push_back(
          RigidBodyFrame<double>("iiwa_link_7", tree.FindBody("iiwa_link_7"),
                                 Isometry3<double>::Identity()));
      auto frame_viz = base_builder->AddSystem<systems::FrameVisualizer>(
          &tree, local_transforms, lcm);
      base_builder->Connect(plant->get_output_port(0),
                            frame_viz->get_input_port(0));
      frame_viz->set_publish_period(kIiwaLcmStatusPeriod);
    } catch (std::logic_error& ex) {
      drake::log()->error("Unable to visualize end effector frames:\n{}\n"
                          "Maybe use --novisualize_frames?",
                          ex.what());
      return 1;
    }
  }

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  command_receiver->set_initial_position(
      &sys->GetMutableSubsystemContext(*command_receiver,
                                       &simulator.get_mutable_context()),
      VectorX<double>::Zero(tree.get_num_positions()));

  // Simulate for a very long time.
  simulator.AdvanceTo(FLAGS_simulation_sec);

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
