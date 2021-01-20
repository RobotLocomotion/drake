/* @file
A four bar linkage demo demonstrating the use of a linear bushing as
a way to model a kinematic loop. It shows:
  - How to model a four bar linkage in SDF.
  - Use the `multibody::Parser` to load a model from an SDF file into a
    MultibodyPlant.
  - Model a revolute joint with a `multibody::LinearBushingRollPitchYaw` to
    model a closed kinematic chain.

  Refer to README.md for more details.
*/
#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
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

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");

DEFINE_double(
    force_stiffness, 30000,
    "Force (translational) stiffness value for kx, ky, kz in N/m of the "
    "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(
    force_damping, 1500,
    "Force (translational) damping value for dx, dy, dz in N·s/m of the "
    "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(torque_stiffness, 30000,
              "Torque (rotational) stiffness value for k₀, k₁, k₂ in N·m/rad "
              "of the LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(
    torque_damping, 1500,
    "Torque (rotational) damping value for d₀, d₁, and d₂ in N·m·s/rad of "
    "the LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(applied_torque, 0.0,
              "Constant torque applied to joint_WA, denoted Tᴀ in the README.");

DEFINE_double(
    initial_velocity, 3.0,
    "Initial velocity, q̇A, of joint_WA. Default set to 3 radians/second ≈ "
    "171.88 degrees/second so that the model has some motion.");

int do_main() {
  // Build a generic MultibodyPlant and SceneGraph.
  DiagramBuilder<double> builder;

  auto [four_bar, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(0.0));

  // Make and add the four_bar model from an SDF model.
  const std::string relative_name =
      "drake/examples/multibody/four_bar/four_bar.sdf";
  const std::string full_name = FindResourceOrThrow(relative_name);

  Parser parser(&four_bar);
  parser.AddModelFromFile(full_name);

  // Get the two frames that define the bushing, namely frame Bc that is
  // welded to the end of link B and frame Cb that is welded to the end of
  // link C. Although the bushing allows 6 degrees-of-freedom between frames
  // Bc and Cb, the stiffness and damping constants are chosen to approximate
  // the connection between frames Bc and Cb as having only one rotational
  // degree of freedom along the bushing's z-axis.
  const Frame<double>& bc_bushing = four_bar.GetFrameByName("Bc_bushing");
  const Frame<double>& cb_bushing = four_bar.GetFrameByName("Cb_bushing");

  // See the README for a discussion of how these parameters were selected
  // for this particular example.
  const double k_xyz = FLAGS_force_stiffness;
  const double d_xyz = FLAGS_force_damping;
  const double k_012 = FLAGS_torque_stiffness;
  const double d_012 = FLAGS_torque_damping;

  // See the documentation for LinearBushingRollPitchYaw.
  // This particular choice of parameters models a z-axis revolute joint.
  // Note: since each link is constrained to rigid motion in the world X-Z
  // plane (X-Y plane of the bushing) by the revolute joints specified in the
  // SDF, it is unnecessary for the bushing to have non-zero values for: k_z,
  // d_z, k_0, d_0, k_1, d_1. They are left here as an example of how to
  // parameterize a general z-axis revolute joint without other constraints.
  const Vector3d force_stiffness_constants{k_xyz, k_xyz, k_xyz};  // N/m
  const Vector3d force_damping_constants{d_xyz, d_xyz, d_xyz};    // N·s/m
  const Vector3d torque_stiffness_constants{k_012, k_012, 0};     // N·m/rad
  const Vector3d torque_damping_constants{d_012, d_012, 0};       // N·m·s/rad

  // Add a bushing force element where the joint between link B and link C
  // should be in an ideal 4-bar linkage.
  four_bar.AddForceElement<LinearBushingRollPitchYaw>(
      bc_bushing, cb_bushing, torque_stiffness_constants,
      torque_damping_constants, force_stiffness_constants,
      force_damping_constants);

  // We are done defining the model. Finalize and build the diagram.
  four_bar.Finalize();

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system and sub-context for the four bar system.
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& four_bar_context =
      four_bar.GetMyMutableContextFromRoot(diagram_context.get());

  // A constant source for applied torque (Tᴀ) at joint_WA.
  four_bar.get_actuation_input_port().FixValue(&four_bar_context,
                                               FLAGS_applied_torque);

  // Set initial conditions so the model will have some motion
  const RevoluteJoint<double>& joint_WA =
      four_bar.GetJointByName<RevoluteJoint>("joint_WA");
  const RevoluteJoint<double>& joint_WC =
      four_bar.GetJointByName<RevoluteJoint>("joint_WC");
  const RevoluteJoint<double>& joint_AB =
      four_bar.GetJointByName<RevoluteJoint>("joint_AB");

  // See the README for an explanation of these angles.
  const double qA = atan2(sqrt(15.0), 1.0);  // about 75.52°
  const double qB = M_PI - qA;               // about 104.48°
  const double qC = qB;                      // about 104.48°

  joint_WA.set_angle(&four_bar_context, qA);
  joint_AB.set_angle(&four_bar_context, qB);
  joint_WC.set_angle(&four_bar_context, qC);

  // Set q̇A,the rate of change in radians/second of the angle qA.
  joint_WA.set_angular_rate(&four_bar_context, FLAGS_initial_velocity);

  // Create a simulator and run the simulation
  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  simulator->AdvanceTo(FLAGS_simulation_time);

  // Print some useful statistics
  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace four_bar
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A four bar linkage demo demonstrating the use of a linear bushing as "
      "a way to model a kinematic loop. Launch drake-visualizer before running "
      "this example.");
  // Changes the default realtime rate to 1.0, so the visualization looks
  // realistic. Otherwise, it finishes so fast that we can't appreciate the
  // motion. Users can still change it on command-line, e.g. "
  // --simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::four_bar::do_main();
}
