// A simple clutter example loaded from an MJCF file, which we can use to
// compare with MuJoCo and other simulators.

#include <chrono>
#include <fstream>
#include <iostream>

#include <gflags/gflags.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/convex_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace examples {
namespace {

// Simulation parameters.
DEFINE_double(simulation_time, 10.0, "Simulation duration in seconds");
DEFINE_double(
    mbp_time_step, 0.0,
    "If mbp_time_step > 0, the fixed-time step period (in seconds) of discrete "
    "updates for the plant (modeled as a discrete system). "
    "If mbp_time_step = 0, the plant is modeled as a continuous system "
    "and no contact forces are displayed.  mbp_time_step must be >= 0.");
DEFINE_double(stiction_tolerance, 1e-4, "Stiction velocity (m/s). ");
DEFINE_double(use_hydro, false, "If true, use hydro. Otherwise point contact.");
DEFINE_double(hc_dissipation, 50.0, "Hunt & Crossley dissipation (s/m). ");
DEFINE_double(point_stiffness, 1.0e6, "Point contact stiffness (N/m).");

// Visualization.
DEFINE_bool(visualize, true, "Whether to visualize (true) or not (false).");
DEFINE_bool(visualize_forces, false,
            "Whether to visualize forces (true) or not (false).");
DEFINE_double(viz_period, std::numeric_limits<double>::infinity(),
              "Viz period.");

// Logging and verbosity for the convex integrator.
DEFINE_bool(log_solver_stats, false,
            "Whether to log convex integrator statistics to a CSV file.");
DEFINE_bool(print_solver_stats, false,
            "Whether to print convex integrator statistics to the console.");

// Convex integrator parameters.
DEFINE_bool(enable_hessian_reuse, true,
            "Whether to reuse the Hessian factorization between iterations.");
DEFINE_int32(k_max, 10,
             "Maximum number of iterations before re-computing the Hessian.");
DEFINE_double(kappa, 0.05,
              "Scaling factor for the relaxed convergence check (θ method of "
              "Hairer 1996) used to exit early under loose accuracies.");
DEFINE_double(
    alpha_max, 1.0,
    "Maximum line search step size for the convex integrator (α_max).");
DEFINE_double(
    ls_tolerance, 1e-6,
    "Tolerance for the exact line search performed by the convex integrator.");
DEFINE_double(tolerance, 1e-8,
              "Convergence tolerance for the convex integrator.");

using geometry::SceneGraphConfig;
using multibody::AddMultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using systems::Context;
using systems::ConvexIntegrator;
using systems::ConvexIntegratorSolverParameters;
using systems::DiagramBuilder;
using visualization::ApplyVisualizationConfig;
using visualization::VisualizationConfig;
using clock = std::chrono::steady_clock;

// MJCF model to simulate
const char mjcf[] = R"""(
<?xml version="1.0"?>
<mujoco model="clutter">

  <option cone="elliptic" />

  <default>
    <geom solimp="0.9 1.0 0.001" solref="0.001 100"/>
  </default>

  <worldbody>
    <geom name="base" type="box" pos="0.0 0.0 0.0" size="0.5 0.5 0.05" rgba="0.5 0.5 0.5 0.3"/>
    <geom name="left" type="box" pos="0.5 0.0 0.5" size="0.05 0.5 0.5" rgba="0.5 0.5 0.5 0.3"/>
    <geom name="right" type="box" pos="-0.5 0.0 0.5" size="0.05 0.5 0.5" rgba="0.5 0.5 0.5 0.3"/>
    <geom name="front" type="box" pos="0.0 0.5 0.5" size="0.5 0.05 0.5" rgba="0.5 0.5 0.5 0.3"/>
    <geom name="back" type="box" pos="0.0 -0.5 0.5" size="0.5 0.05 0.5" rgba="0.5 0.5 0.5 0.3"/>
    <body>
      <joint type="free"/>
      <geom name="ball1" type="sphere" pos="0.0 0.0 0.5" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball2" type="sphere" pos="0.0001 0.0 0.7" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball3" type="sphere" pos="0.0 0.0001 0.9" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball4" type="sphere" pos="-0.0001 0.0 1.1" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball5" type="sphere" pos="0.0 -0.0001 1.3" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball6" type="sphere" pos="0.0 0.0 1.5" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball7" type="sphere" pos="0.0001 0.0 1.7" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball8" type="sphere" pos="0.0 0.0001 1.9" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball9" type="sphere" pos="-0.0001 0.0 2.1" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    <body>
      <joint type="free"/>
      <geom name="ball10" type="sphere" pos="0.0 -0.0001 2.3" size="0.1" rgba="1.0 1.0 1.0 1.0"/>
    </body>
  </worldbody>
</mujoco>
)""";

int do_main() {
  DiagramBuilder<double> builder;

  // Add the plant model
  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_time_step;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  plant_config.contact_model = FLAGS_use_hydro ? "hydroelastic" : "point";
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);
  Parser(&plant, &scene_graph).AddModelsFromString(mjcf, "xml");

  SceneGraphConfig sg_config;
  sg_config.default_proximity_properties.hunt_crossley_dissipation =
      FLAGS_hc_dissipation;
  sg_config.default_proximity_properties.point_stiffness =
      FLAGS_point_stiffness;
  scene_graph.set_config(sg_config);

  plant.Finalize();

  fmt::print("Num positions: {:d}\n", plant.num_positions());
  fmt::print("Num velocities: {:d}\n", plant.num_velocities());

  // Set up the visualizer
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  if (FLAGS_visualize) {
    VisualizationConfig vis_config;
    vis_config.publish_period = FLAGS_viz_period;
    vis_config.publish_contacts = FLAGS_visualize_forces;
    ApplyVisualizationConfig(vis_config, &builder, nullptr, &plant,
                             &scene_graph, meshcat);
  }

  // Build the diagram
  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  // Set up the simulator
  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  // Use the convex integrator
  if (FLAGS_mbp_time_step == 0.0) {
    ConvexIntegrator<double>& ci =
        simulator->reset_integrator<ConvexIntegrator<double>>();
    ci.set_plant(&plant);
    ci.set_maximum_step_size(FLAGS_simulator_max_time_step);
    ci.set_fixed_step_mode(!FLAGS_simulator_use_error_control);
    ci.set_target_accuracy(FLAGS_simulator_accuracy);
    ci.set_log_solver_stats(FLAGS_log_solver_stats);
    ci.set_print_solver_stats(FLAGS_print_solver_stats);

    ConvexIntegratorSolverParameters ci_params;
    ci_params.enable_hessian_reuse = FLAGS_enable_hessian_reuse;
    ci_params.max_iterations_for_hessian_reuse = FLAGS_k_max;
    ci_params.kappa = FLAGS_kappa;
    ci_params.alpha_max = FLAGS_alpha_max;
    ci_params.ls_tolerance = FLAGS_ls_tolerance;
    ci_params.tolerance = FLAGS_tolerance;
    ci.set_solver_parameters(ci_params);
  }

  simulator->set_publish_every_time_step(true);
  simulator->Initialize();
  if (FLAGS_visualize) {
    // Wait for meshcat to load
    std::cout << "Press [ENTER] to continue ...\n";
    getchar();
  }

  const double recording_frames_per_second =
      FLAGS_mbp_time_step == 0 ? 32 : 1.0 / FLAGS_mbp_time_step;
  meshcat->StartRecording(recording_frames_per_second);
  clock::time_point sim_start_time = clock::now();
  simulator->AdvanceTo(FLAGS_simulation_time);
  clock::time_point sim_end_time = clock::now();
  const double sim_time =
      std::chrono::duration<double>(sim_end_time - sim_start_time).count();
  std::cout << "AdvanceTo() time [sec]: " << sim_time << std::endl;
  meshcat->StopRecording();
  meshcat->PublishRecording();

  PrintSimulatorStatistics(*simulator);

  if (FLAGS_visualize) {
    // Wait for meshcat to finish rendering.
    std::cout << "Press [ENTER] to quit ...\n";
    getchar();
  }

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::do_main();
}
