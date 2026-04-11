/// @file
///
/// This demo sets up a gravity compensated JACO arm within a MultibodyPlant
/// simulation controlled via LCM.

#include <limits>
#include <memory>
#include <vector>

#include <gflags/gflags.h>

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
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/visualization/visualization_config_functions.h"

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
using drake::manipulation::kinova_jaco::kJacoDefaultArmNumFingers;
using drake::manipulation::kinova_jaco::kJacoDefaultArmNumJoints;
using drake::manipulation::kinova_jaco::kJacoLcmStatusPeriod;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::systems::Demultiplexer;
using drake::systems::controllers::InverseDynamicsController;
using drake::visualization::AddDefaultVisualization;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

const char kUrdfUrl[] =
    "package://drake_models/jaco_description/urdf/"
    "j2s7s300_sphere_collision.urdf";

int DoMain() {
  systems::DiagramBuilder<double> builder;

  auto [jaco_plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);

  const multibody::ModelInstanceIndex jaco_id =
      Parser(&builder).AddModelsFromUrl(kUrdfUrl).at(0);
  jaco_plant.WeldFrames(jaco_plant.world_frame(),
                        jaco_plant.GetFrameByName("base"));
  jaco_plant.Finalize();

  // These gains are really just a guess.  Velocity limits are not enforced,
  // allowing much faster simulated movement than the actual robot.
  const int num_positions = jaco_plant.num_positions();
  VectorXd kp = VectorXd::Constant(num_positions, 100);
  VectorXd kd = 2.0 * kp.array().sqrt();
  VectorXd ki = VectorXd::Zero(num_positions);

  auto jaco_controller = builder.AddSystem<InverseDynamicsController>(
      jaco_plant, kp, ki, kd, false);

  builder.Connect(jaco_plant.get_state_output_port(jaco_id),
                  jaco_controller->get_input_port_estimated_state());

  AddDefaultVisualization(&builder);

  systems::lcm::LcmInterfaceSystem* lcm =
      builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_jaco_command>(
          "KINOVA_JACO_COMMAND", lcm));
  auto command_receiver = builder.AddSystem<JacoCommandReceiver>();
  builder.Connect(command_sub->get_output_port(),
                  command_receiver->get_message_input_port());

  auto mux = builder.AddSystem<systems::Multiplexer>(
      std::vector<int>({kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers,
                        kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers}));
  builder.Connect(command_receiver->get_commanded_position_output_port(),
                  mux->get_input_port(0));
  builder.Connect(command_receiver->get_commanded_velocity_output_port(),
                  mux->get_input_port(1));
  builder.Connect(mux->get_output_port(),
                  jaco_controller->get_input_port_desired_state());
  builder.Connect(jaco_controller->get_output_port_control(),
                  jaco_plant.get_actuation_input_port(jaco_id));

  auto status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_jaco_status>(
          "KINOVA_JACO_STATUS", lcm, kJacoLcmStatusPeriod));
  // TODO(sammy-tri) populate joint torque (and external torques).  External
  // torques might want to wait until after #12631 is fixed or it could slow
  // down the simulation significantly.
  auto status_sender = builder.AddSystem<JacoStatusSender>();
  auto demux = builder.AddSystem<systems::Demultiplexer>(
      std::vector<int>({kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers,
                        kJacoDefaultArmNumJoints + kJacoDefaultArmNumFingers}));
  builder.Connect(jaco_plant.get_state_output_port(jaco_id),
                  demux->get_input_port());
  builder.Connect(demux->get_output_port(0),
                  status_sender->get_position_input_port());
  builder.Connect(demux->get_output_port(0),
                  command_receiver->get_position_measured_input_port());
  builder.Connect(demux->get_output_port(1),
                  status_sender->get_velocity_input_port());
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

  jaco_plant.SetPositions(
      &diagram->GetMutableSubsystemContext(jaco_plant, &root_context),
      initial_position);

  simulator.Initialize();
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
