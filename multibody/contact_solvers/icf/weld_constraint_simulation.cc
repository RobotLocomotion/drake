/// @file weld_constraint_simulation.cc
///
/// A simulation demonstrating the ICF weld constraint.
///
/// Setup:
///   - box1: a gray cube welded to the world at z=0.5 (no joint in MJCF).
///   - box2: an orange cube, free body (6 DOF), connected to box1 via
///     AddWeldConstraint. The constraint places box2's bottom face against
///     box1's top face. box2 starts slightly displaced so the ICF constraint
///     correction is visible.
///   - floor: a green slab welded to the world for visual context.
///
/// The CENIC integrator (using the ICF solver) enforces the weld constraint
/// each time step, pulling box2 into alignment with box1 despite the initial
/// displacement error and gravity.
///
/// Run:
///   bazel run //multibody/contact_solvers/icf:weld_constraint_simulation
///
/// Then open http://localhost:7000 in your browser to see Meshcat.

#include <iostream>
#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");
DEFINE_double(realtime_rate, 1.0, "Target realtime rate for visualization.");
DEFINE_double(max_step_size, 0.1, "Maximum integrator step size in seconds.");
DEFINE_double(target_accuracy, 1e-3, "CENIC target integration accuracy.");
DEFINE_double(initial_gap, 0.0, "Weld constraint error to be overcome.");
DEFINE_bool(fixed_step, false,
            "If true, run the CENIC integrator in fixed-step mode (disables "
            "error control). Recovers the 'Lagged' discrete-time ICF variant.");

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using multibody::CenicIntegrator;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

// MJCF model defining two boxes and a floor.
//
// box1: half-extents 0.15m, welded to the world (no joint) at z=0.5.
// box2: half-extents 0.10m, free floating body (freejoint). The weld
//       constraint will be added programmatically below.
// floor: half-extents 1m x 1m x 0.05m, welded to world with top face at z=0.
constexpr char kMjcf[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="weld_constraint_demo">
    <worldbody>
      <!-- box1: welded to world at z=1.0 (no joint means fixed) -->
      <body name="box1" pos="0 0 1.0">
        <inertial mass="2" diaginertia="0.015 0.015 0.015"/>
        <geom type="box" size="0.15 0.15 0.15" rgba="0.5 0.5 0.5 1"/>
      </body>
      <!-- box2: free body, to be weld-constrained to box1 -->
      <body name="box2" pos="0 0 0.75">
        <joint name="box2_free" type="free"/>
        <inertial mass="1" diaginertia="0.0067 0.0067 0.0067"/>
        <geom type="box" size="0.1 0.1 0.1" rgba="0.9 0.4 0.1 1"/>
      </body>
      <!-- floor for visual context -->
      <geom name="floor" type="box" pos="0 0 -0.05" size="1 1 0.05"
            rgba="0.2 0.8 0.3 1"/>
    </worldbody>
  </mujoco>
)""";

int do_main() {
  DiagramBuilder<double> builder;

  // Continuous-time plant is required for the CENIC integrator.
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.0;
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  Parser(&plant).AddModelsFromString(kMjcf, "xml");

  const auto& box1 = plant.GetBodyByName("box1");
  const auto& box2 = plant.GetBodyByName("box2");

  // Weld constraint: frame P on box1 at the center of its top face (+z),
  // frame Q on box2 at the center of its bottom face (-z).
  // When satisfied, box2 sits on top of box1:
  //   box1 center at z=1.0, top face at z=1.15 → box2 center at z=1.25.
  const RigidTransformd X_box1_P(Vector3d(0.0, 0.0, -0.15));
  const RigidTransformd X_box2_Q(Vector3d(0.0, 0.0, 0.1));
  plant.AddWeldConstraint(box1, X_box1_P, box2, X_box2_Q);

  plant.Finalize();

  // Add Meshcat visualization.
  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();

  // Set initial conditions: box2 starts at its exact constrained position
  // (center of box2 at (0, 0, 0.75), sitting on top of box1).
  // The weld constraint resists gravity to hold box2 in place. Without the
  // constraint, box2 would fall freely under gravity.
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
  // @sherm move the lower box down, to say 0.7 and you'll see the bug.
  // 0.75 is the exact constrained position.
  plant.SetFloatingBaseBodyPoseInWorldFrame(
      &plant_context, box2,
      RigidTransformd(Vector3d(0.0, 0.0, 0.75 - FLAGS_initial_gap)));

  // Set up the simulator with the CENIC integrator.
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  auto& integrator = simulator->reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(FLAGS_max_step_size);
  integrator.set_fixed_step_mode(FLAGS_fixed_step);
  if (!FLAGS_fixed_step) {
    integrator.set_target_accuracy(FLAGS_target_accuracy);
  }

  simulator->set_target_realtime_rate(FLAGS_realtime_rate);
  simulator->Initialize();
  std::cout << "Simulating for " << FLAGS_simulation_time
            << " seconds. Enter to continue ..." << std::endl;
  std::cin.get();  // Wait for user input before starting the simulation.
  simulator->AdvanceTo(FLAGS_simulation_time);

  systems::PrintSimulatorStatistics(*simulator);
  return 0;
}

}  // namespace
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Simulation demonstrating the ICF weld constraint.\n"
      "\n"
      "  box1 (gray)   is welded to the world (no joint in MJCF).\n"
      "  box2 (orange) is a free body connected to box1 via\n"
      "                AddWeldConstraint (ICF enforcement).\n"
      "\n"
      "box2 starts displaced from box1's top face. The CENIC integrator\n"
      "(ICF solver) enforces the weld constraint each step, drawing box2\n"
      "back into alignment with box1 against gravity.\n"
      "\n"
      "Open http://localhost:7000 in your browser for Meshcat visualization.\n"
      "\n"
      "Use --fixed_step to disable error control and run at a fixed step "
      "size,\n"
      "recovering the 'Lagged' discrete-time ICF variant.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::contact_solvers::icf::do_main();
}
