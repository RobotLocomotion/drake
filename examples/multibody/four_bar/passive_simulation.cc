#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using Eigen::Vector3d;

using geometry::SceneGraph;
using multibody::Frame;
using multibody::LinearBushingRollPitchYaw;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RevoluteJoint;
using systems::Context;
using systems::DiagramBuilder;
using systems::Simulator;

namespace examples {
namespace multibody {
namespace four_bar {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(force_stiffness, 30000,
              "Desired force stiffness value for kx, ky, and kz of the "
              "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(force_damping, 800,
              "Desired force damping value for dx, dy, and dz of the "
              "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(torque_stiffness, 30000,
              "Desired torque stiffness value for k₀, k₁, and k₂ of the "
              "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(torque_damping, 800,
              "Desired torque damping value for d₀, d₁, and d₂ of the "
              "LinearBushingRollPitchYaw ForceElement.");

int do_main() {
  // Build a generic MultibodyPlant and SceneGraph.
  DiagramBuilder<double> builder;

  auto pair = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(0.0));

  MultibodyPlant<double>& four_bar = pair.plant;
  SceneGraph<double>& scene_graph = pair.scene_graph;

  // Make and add the four_bar model from an SDF model.
  const std::string relative_name =
      "drake/examples/multibody/four_bar/four_bar.sdf";
  const std::string full_name = FindResourceOrThrow(relative_name);

  Parser parser(&four_bar, &scene_graph);
  parser.AddModelFromFile(full_name);

  // Grab the two coincident frames at the midpoint of the coupler link
  // One is attached to the Crank side, one to the Rocker side.
  // The two frames will be welded together with a LinearBushingRollPitchYaw
  // force element to approximate a closed-loop kinematic chain.

  // Grab the two frames, one attached to the end of the link B; the other
  // attached to the end of link C. The two frames will be welded together
  // with one rotational degree of freedom along the z axis of BC_Bushing.
  const Frame<double>& bc_bushing = four_bar.GetFrameByName("BC_Bushing");

  const Frame<double>& cb_bushing = four_bar.GetFrameByName("CB_Bushing");

  // TODO(joemasterjohn) come up with a way to estimate correct parameters
  //  for stiffness and damping constants. These are hand tuned to fit the
  //  linkages used in this example and make the simulation "look right".
  const double k_xyz = FLAGS_force_stiffness;
  const double d_xyz = FLAGS_force_damping;
  const double k_012 = FLAGS_torque_stiffness;
  const double d_012 = FLAGS_torque_damping;

  // See the documentation for LinearBushingRollPitchYaw.
  // This particular choice of parameters models a z-axis revolute joint.
  const Vector3d torque_stiffness_constants{k_xyz, k_xyz, 0};
  const Vector3d torque_damping_constants{d_xyz, k_xyz, 0};
  const Vector3d force_stiffness_constants{k_012, k_012, k_012};
  const Vector3d force_damping_constants{d_012, d_012, d_012};

  // Add a bushing force element where the joint between link B and link C
  // should be in an ideal 4-bar linkage.
  four_bar.AddForceElement<LinearBushingRollPitchYaw>(
      bc_bushing, cb_bushing, torque_stiffness_constants,
      torque_damping_constants, force_stiffness_constants,
      force_damping_constants);

  // We are done defining the model. Finalize and build the diagram.
  four_bar.Finalize();

  ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system and sub-context for the four bar system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& four_bar_context =
      diagram->GetMutableSubsystemContext(four_bar, diagram_context.get());

  // A constant source for a zero applied torque at the Crank joint.
  four_bar.get_actuation_input_port().FixValue(&four_bar_context, 0.0);

  // Set initial conditions so the model will have some motion
  RevoluteJoint<double>& WA_joint =
      four_bar.GetMutableJointByName<RevoluteJoint>("q_WA");
  RevoluteJoint<double>& WC_joint =
      four_bar.GetMutableJointByName<RevoluteJoint>("q_WC");
  RevoluteJoint<double>& AB_joint =
      four_bar.GetMutableJointByName<RevoluteJoint>("q_AB");

  // See the README for an explanation of these angles.
  const double qA = atan2(sqrt(15.0), 1.0);
  const double qB = M_PI - qA;
  const double qC = qB;

  WA_joint.set_angle(&four_bar_context, qA);
  AB_joint.set_angle(&four_bar_context, qB);
  WC_joint.set_angle(&four_bar_context, qC);

  // Set the velocity of qA so the model has some motion.
  WA_joint.set_angular_rate(&four_bar_context, 3.0);

  // Create a simulator and run the simulation
  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  simulator->set_publish_every_time_step(false);
  simulator->set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator->Initialize();
  simulator->AdvanceTo(FLAGS_simulation_time);

  // Print some useful statistics
  PrintSimulatorStatistics(*simulator);

  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace four_bar
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A four bar linkage demo demonstrating the use of a linear bushing  as "
      "a way to model kinematic loop. Launch drake-visualizer before running "
      "this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::four_bar::do_main();
}
