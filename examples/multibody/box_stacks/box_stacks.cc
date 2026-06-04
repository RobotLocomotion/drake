/* @file
A demonstration of the CenicIntegrator on a scene that decomposes into several
independent constraint "islands". The scene is a set of well-separated vertical
stacks of boxes: the boxes within a stack are in contact (and form one island),
while the stacks are far enough apart that they never interact. CENIC can solve
the islands independently and, optionally, in parallel (see --num_threads).

The example exposes the main CENIC knobs: Meshcat visualization,
error-controlled vs. fixed-step integration, and the target accuracy. It prints
simulator and CENIC solver statistics, along with the wall-clock simulation
time, at the end. */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace multibody {
namespace box_stacks {
namespace {

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::CenicIntegrator;
using drake::multibody::Parser;
using drake::systems::Simulator;

DEFINE_double(simulation_time, 2.0, "Duration of the simulation [s].");

DEFINE_bool(
    fixed_step, false,
    "If true, run CENIC in fixed-step mode using --max_step_size as the "
    "step (no error control). If false (the default), use "
    "error-controlled integration with target --accuracy.");

DEFINE_double(accuracy, 1.0e-3,
              "Target accuracy for error-controlled integration. Smaller is "
              "more accurate (and slower). Ignored in --fixed_step mode.");

DEFINE_double(max_step_size, 0.1,
              "Maximum integration step [s]. In --fixed_step mode this is the "
              "exact (constant) step size.");

DEFINE_int32(num_stacks, 8,
             "Number of well-separated box stacks. Each stack is an "
             "independent constraint island.");

DEFINE_int32(boxes_per_stack, 10, "Number of boxes in each stack.");

DEFINE_int32(
    num_threads, 1,
    "Number of threads CENIC uses to solve constraint islands in "
    "parallel. 1 (the default) is serial; 0 uses the maximum available "
    "(Parallelism::Max()). Results are independent of this value.");

DEFINE_bool(meshcat, false,
            "If true, start a Meshcat server and visualize the simulation. You "
            "will probably also want --target_realtime_rate=1 to watch it at a "
            "natural speed.");

DEFINE_double(target_realtime_rate, 0.0,
              "Desired rate relative to real time. 0 runs as fast as possible "
              "(best for timing); use 1.0 to watch the simulation in Meshcat.");

/* Builds an MJCF description of `num_stacks` vertical stacks of
`boxes_per_stack` free boxes resting on a large floor. Stacks are spaced far
apart along x so that they never come into contact with one another. */
std::string MakeBoxStacksXml(int num_stacks, int boxes_per_stack) {
  const double box = 0.05;           // Box half-size [m].
  const double stack_spacing = 5.0;  // Distance between stacks [m].
  std::string bodies;
  for (int s = 0; s < num_stacks; ++s) {
    for (int b = 0; b < boxes_per_stack; ++b) {
      const double x = s * stack_spacing;
      const double z = box + b * (2 * box + 0.001);
      bodies += fmt::format(
          "    <body name=\"s{}_b{}\" pos=\"{} 0 {}\">\n"
          "      <joint type=\"free\"/>\n"
          "      <geom type=\"box\" size=\"{} {} {}\"/>\n"
          "    </body>\n",
          s, b, x, z, box, box, box);
    }
  }
  return fmt::format(
      "<?xml version=\"1.0\"?>\n"
      "<mujoco model=\"box_stacks\">\n"
      "  <worldbody>\n"
      "    <geom name=\"floor\" type=\"box\" pos=\"0 0 -0.1\" "
      "size=\"500 500 0.1\"/>\n"
      "{}"
      "  </worldbody>\n"
      "</mujoco>\n",
      bodies);
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  // CENIC integrates a continuous-time plant, so use a zero discrete time step.
  auto& plant = AddMultibodyPlantSceneGraph(&builder, 0.0).plant;
  Parser(&plant).AddModelsFromString(
      MakeBoxStacksXml(FLAGS_num_stacks, FLAGS_boxes_per_stack), "xml");
  plant.Finalize();

  std::shared_ptr<geometry::Meshcat> meshcat;
  if (FLAGS_meshcat) {
    meshcat = std::make_shared<geometry::Meshcat>();
    visualization::AddDefaultVisualization(&builder, meshcat);
  }

  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  auto& integrator = simulator.reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(FLAGS_max_step_size);
  if (FLAGS_fixed_step) {
    integrator.set_fixed_step_mode(true);
  } else {
    integrator.set_target_accuracy(FLAGS_accuracy);
  }
  const Parallelism parallelism = (FLAGS_num_threads == 0)
                                      ? Parallelism::Max()
                                      : Parallelism(FLAGS_num_threads);
  integrator.set_parallelism(parallelism);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  fmt::print(
      "Simulating {} box stacks of {} boxes ({} bodies) for {} s with CENIC.\n",
      FLAGS_num_stacks, FLAGS_boxes_per_stack,
      FLAGS_num_stacks * FLAGS_boxes_per_stack, FLAGS_simulation_time);
  fmt::print("Integration: {}; island solver threads: {}.\n",
             FLAGS_fixed_step
                 ? fmt::format("fixed step ({} s)", FLAGS_max_step_size)
                 : fmt::format("error control (accuracy {})", FLAGS_accuracy),
             parallelism.num_threads());

  const auto start = std::chrono::steady_clock::now();
  simulator.AdvanceTo(FLAGS_simulation_time);
  const auto stop = std::chrono::steady_clock::now();
  const double wall_clock = std::chrono::duration<double>(stop - start).count();

  // Print the standard simulator/integrator statistics. For CENIC these include
  // the convex-solver counters (cenic_total_solver_iterations, etc.) in the
  // JSON block. Follow with the measured wall-clock time.
  systems::PrintSimulatorStatistics(simulator);
  fmt::print("\nWall-clock simulation time: {:.4f} s.\n", wall_clock);

  if (meshcat != nullptr) {
    fmt::print("\nVisualization is live at {}.\nPress Enter to exit.\n",
               meshcat->web_url());
    std::cin.get();
  }

  return 0;
}

}  // namespace
}  // namespace box_stacks
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Simulates several independent stacks of boxes with Drake's CENIC "
      "integrator, demonstrating optional per-island parallelism. See the "
      "flags for Meshcat visualization, fixed-step vs. error-controlled "
      "integration, accuracy, and the number of solver threads.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::box_stacks::do_main();
}
