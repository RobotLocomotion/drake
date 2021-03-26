/// @file
///
/// This demo sets up a gravity compensated JACO arm within a MultibodyPlant
/// simulation controlled via LCM.

#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/manipulation/kinova_jaco/jaco_command_receiver.h"
#include "drake/manipulation/kinova_jaco/jaco_constants.h"
#include "drake/manipulation/kinova_jaco/jaco_status_sender.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(realtime_rate, 1.0, "");
DEFINE_double(time_step, 3e-3,
              "The time step to use for MultibodyPlant model "
              "discretization.  0 uses the continuous version of the plant.");

using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::manipulation::kinova_jaco::JacoCommandReceiver;
using drake::manipulation::kinova_jaco::JacoStatusSender;
using drake::manipulation::kinova_jaco::kJacoDefaultArmNumJoints;
using drake::manipulation::kinova_jaco::kJacoDefaultArmNumFingers;
using drake::manipulation::kinova_jaco::kJacoLcmStatusPeriod;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::controllers::InverseDynamicsController;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

const char kUrdfPath[] =
    "drake/manipulation/models/jaco_description/urdf/"
    "j2s7s300_sphere_collision.urdf";

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>* scene_graph = builder.AddSystem<SceneGraph>();
  MultibodyPlant<double>* jaco_plant = builder.AddSystem<MultibodyPlant>(
      FLAGS_time_step);

  jaco_plant->RegisterAsSourceForSceneGraph(scene_graph);
  builder.Connect(
      jaco_plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(
          jaco_plant->get_source_id().value()));
  builder.Connect(
      scene_graph->get_query_output_port(),
      jaco_plant->get_geometry_query_input_port());

  const multibody::ModelInstanceIndex jaco_id =
      Parser(jaco_plant, scene_graph).AddModelFromFile(
          FindResourceOrThrow(kUrdfPath));
  jaco_plant->WeldFrames(jaco_plant->world_frame(),
                         jaco_plant->GetFrameByName("base"));
  jaco_plant->Finalize();

  // These gains are really just a guess.  Velocity limits are not enforced,
  // allowing much faster simulated movement than the actual robot.
  const int num_positions = jaco_plant->num_positions();
  VectorXd kp = VectorXd::Constant(num_positions, 100);
  VectorXd kd = 2.0 * kp.array().sqrt();
  VectorXd ki = VectorXd::Zero(num_positions);

  auto jaco_controller = builder.AddSystem<InverseDynamicsController>(
      *jaco_plant, kp, ki, kd, false);

  builder.Connect(jaco_plant->get_state_output_port(jaco_id),
                  jaco_controller->get_input_port_estimated_state());

  systems::lcm::LcmInterfaceSystem* lcm =
      builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_jaco_command>(
          "KINOVA_JACO_COMMAND", lcm));
  auto command_receiver = builder.AddSystem<JacoCommandReceiver>();
  builder.Connect(command_sub->get_output_port(),
                  command_receiver->get_input_port());
  builder.Connect(command_receiver->get_output_port(),
                  jaco_controller->get_input_port_desired_state());
  builder.Connect(jaco_controller->get_output_port_control(),
                  jaco_plant->get_actuation_input_port(jaco_id));

  auto status_pub =
      builder.AddSystem(
          systems::lcm::LcmPublisherSystem::Make<drake::lcmt_jaco_status>(
          "KINOVA_JACO_STATUS", lcm, kJacoLcmStatusPeriod));
  // TODO(sammy-tri) populate joint torque (and external torques).  External
  // torques might want to wait until after #12631 is fixed or it could slow
  // down the simulation significantly.
  auto status_sender = builder.AddSystem<JacoStatusSender>();
  builder.Connect(jaco_plant->get_state_output_port(jaco_id),
                  status_sender->get_state_input_port());
  builder.Connect(status_sender->get_output_port(),
                  status_pub->get_input_port());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& root_context = simulator.get_mutable_context();

  // Set the initial position to something similar to where the jaco moves to
  // when starting teleop.
  VectorXd initial_position = VectorXd::Zero(num_positions);
  initial_position(0) = 1.80;
  initial_position(1) = 3.44;
  initial_position(2) = 3.14;
  initial_position(3) = 0.76;
  initial_position(4) = 4.63;
  initial_position(5) = 4.49;
  initial_position(6) = 5.03;

  command_receiver->set_initial_position(
      &diagram->GetMutableSubsystemContext(*command_receiver, &root_context),
      initial_position);
  jaco_plant->SetPositions(
      &diagram->GetMutableSubsystemContext(*jaco_plant, &root_context),
      initial_position);

  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_sec);

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
