/// @file
///
/// This file set up a simulation environment of an allegro hand an object.The
/// only controllable interface in this instance is the target positions of the
/// joints on the hand fingers. The simulator connects with the control program
/// through LCM, with the output of the hand state, and take in the command of
/// the finger joint positions. The simulation also measures the pose of the
/// object and publish it through LCM, which is similar to an ideal motion
/// tracking system.

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/allegro_hand/allegro_common.h"
#include "drake/examples/allegro_hand/allegro_lcm.h"
#include "drake/examples/allegro_hand/in_hand_manipulation/object_pose_publisher.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_allegro_command.hpp"
#include "drake/lcmt_allegro_status.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace allegro_hand {
namespace {

using drake::multibody::multibody_plant::MultibodyPlant;

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "Desired duration of the simulation in seconds");
DEFINE_bool(use_right_hand, true,
            "Which hand to model: true for right hand or false for left hand");
DEFINE_double(max_time_step, 1.5e-4,
              "Simulation time step used for intergrator.");
DEFINE_bool(add_gravity, false,
            "Whether adding gravity (9.81 m/s^2) in the simulation");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;
  lcm::DrakeLcm lcm;

  geometry::SceneGraph<double>* scene_graph =
      builder.AddSystem<geometry::SceneGraph>();
  scene_graph->set_name("scene_graph");

  MultibodyPlant<double>* plant =
      builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step);
  std::string HandSdfPath;
  if (FLAGS_use_right_hand)
    HandSdfPath = FindResourceOrThrow(
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_right.sdf");
  else
    HandSdfPath = FindResourceOrThrow(
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_left.sdf");
  const std::string ObjectModelPath = FindResourceOrThrow(
      "drake/examples/"
      "allegro_hand/in_hand_manipulation/models/simple_mug.sdf");
  auto hand_index =
      multibody::parsing::AddModelFromSdfFile(HandSdfPath, plant, scene_graph);
  auto obj_index = multibody::parsing::AddModelFromSdfFile(ObjectModelPath,
                                                           plant, scene_graph);

  // Weld the hand to the world frame
  const auto& joint_hand_root = plant->GetBodyByName("hand_root");
  plant->AddJoint<multibody::WeldJoint>("weld_hand", plant->world_body(), {},
                                        joint_hand_root, {},
                                        Isometry3<double>::Identity());

  // Add gravity, if needed
  if (FLAGS_add_gravity)
    plant->AddForceElement<multibody::UniformGravityFieldElement>(
        -9.81 * Eigen::Vector3d::UnitZ());
  plant->Finalize(scene_graph);

  // Visualization
  geometry::ConnectDrakeVisualizer(&builder, *scene_graph);
  DRAKE_DEMAND(!!plant->get_source_id());
  builder.Connect(
      plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(plant->get_source_id().value()));
  builder.Connect(scene_graph->get_query_output_port(),
                  plant->get_geometry_query_input_port());

  // Publish contact results for visualization.
  const auto contact_results_to_lcm =
      builder.AddSystem<multibody::multibody_plant::ContactResultsToLcmSystem>(
          *plant);
  const auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  // Contact results to lcm msg.
  builder.Connect(plant->get_contact_results_output_port(),
                  contact_results_to_lcm->get_input_port(0));
  builder.Connect(contact_results_to_lcm->get_output_port(0),
                  contact_results_publisher->get_input_port());

  // PID Controller for the hand joints
  VectorX<double> kp, kd, ki;
  MatrixX<double> Px, Py;
  GetControlPortMapping(*plant, &Px, &Py);
  SetPositionControlledGains(&kp, &ki, &kd);
  auto hand_controller = builder.AddSystem<systems::controllers::PidController>(
      Px, Py, kp, ki, kd);
  builder.Connect(plant->get_continuous_state_output_port(),
                  hand_controller->get_input_port_estimated_state());
  builder.Connect(hand_controller->get_output_port_control(),
                  plant->get_actuation_input_port());

  // Creat an output port from the plant that only outputs the status of the
  // hand fingers
  const auto hand_status_converter =
      builder.AddSystem<systems::MatrixGain<double>>(Px);
  builder.Connect(plant->get_continuous_state_output_port(),
                  hand_status_converter->get_input_port());
  const auto hand_output_torque_converter =
      builder.AddSystem<systems::MatrixGain<double>>(Py);
  builder.Connect(hand_controller->get_output_port_control(),
                  hand_output_torque_converter->get_input_port());

  // Create the command subscriber and status publisher.
  // External torque measurement is not abalable here
  auto hand_command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_allegro_command>(
          "ALLEGRO_COMMAND", &lcm));
  hand_command_sub->set_name("hand_command_subscriber");
  auto hand_command_receiver =
      builder.AddSystem<AllegroCommandReceiver>(kAllegroNumJoints);
  hand_command_receiver->set_name("hand_command_receiver");
  auto hand_status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_allegro_status>(
          "ALLEGRO_STATUS", &lcm));
  hand_status_pub->set_name("hand_status_publisher");
  hand_status_pub->set_publish_period(kLcmStatusPeriod);
  auto status_sender =
      builder.AddSystem<AllegroStatusSender>(kAllegroNumJoints);
  status_sender->set_name("status_sender");

  builder.Connect(hand_command_sub->get_output_port(),
                  hand_command_receiver->get_input_port(0));
  builder.Connect(hand_command_receiver->get_commanded_state_output_port(),
                  hand_controller->get_input_port_desired_state());
  builder.Connect(hand_status_converter->get_output_port(),
                  status_sender->get_state_input_port());
  builder.Connect(hand_command_receiver->get_output_port(0),
                  status_sender->get_command_input_port());
  builder.Connect(hand_output_torque_converter->get_output_port(),
                  status_sender->get_commanded_torque_input_port());
  builder.Connect(status_sender->get_output_port(0),
                  hand_status_pub->get_input_port());

  // Publish the pose of the mug through LCM
  auto obj_pose_publisher =
      builder.AddSystem<ObjectPosePublisher>(*plant, "main_body");
  builder.Connect(plant->get_continuous_state_output_port(),
                  obj_pose_publisher->get_state_input_port());

  // Now the model is complete.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  geometry::DispatchLoadMessage(*scene_graph, &lcm);
  // Create a context for this system
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*plant, diagram_context.get());

  // Initialize the mug pose to be right in the middle between the fingers.
  const multibody::Body<double>& mug =
      plant->GetBodyByName("main_body", obj_index);
  const multibody::Body<double>& hand =
      plant->GetBodyByName("hand_root", hand_index);
  std::vector<Eigen::Isometry3d> X_WB_all;
  plant->tree().CalcAllBodyPosesInWorld(plant_context, &X_WB_all);
  const Eigen::Vector3d& p_WHand = X_WB_all[hand.index()].translation();
  Eigen::Isometry3d X_WM;
  Eigen::Vector3d rpy(M_PI / 2, 0, 0);
  X_WM.linear() =
      math::RotationMatrix<double>(math::RollPitchYaw<double>(rpy)).matrix();
  X_WM.translation() = p_WHand + Eigen::Vector3d(0.095, 0.075, 0.100);
  X_WM.makeAffine();
  plant->tree().SetFreeBodyPoseOrThrow(mug, X_WM, &plant_context);

  lcm.StartReceiveThread();

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  hand_command_receiver->set_initial_position(
      &diagram->GetMutableSubsystemContext(*hand_command_receiver,
                                           &simulator.get_mutable_context()),
      VectorX<double>::Zero(plant->num_actuators()));

  sleep(1);

  simulator.StepTo(FLAGS_simulation_time);
}

}  // namespace
}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple dynamic simulation for the Allegro hand moving under constant"
      " torques.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::allegro_hand::DoMain();
  return 0;
}
