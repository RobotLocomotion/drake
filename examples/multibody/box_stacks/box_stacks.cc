/* @file
A demonstration of the CenicIntegrator on a scene that decomposes into several
independent constraint "islands". The scene is a grid of well-separated vertical
stacks of objects (boxes or spheres): the objects within a stack are in contact
(forming one island), while the stacks are far enough apart that they never
interact. CENIC can solve the islands independently and, optionally, in parallel
(see --num_threads).

The example exposes the main CENIC knobs along with a discrete-MultibodyPlant
baseline:
 - --meshcat records the simulation and replays it as an animation at the end;
 - --time_step selects a continuous plant integrated by CENIC (0, the default)
   or a discrete plant using its built-in contact solver (> 0) as a baseline;
 - --fixed_step / --accuracy choose CENIC's integration mode;
 - --hydroelastic switches from point to hydroelastic contact;
 - --spheres simulates spheres instead of boxes.
It prints simulator and CENIC solver statistics, along with the wall-clock
simulation time, at the end. */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/cenic/cenic_integrator.h"
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

using drake::geometry::Box;
using drake::geometry::HalfSpace;
using drake::geometry::ProximityProperties;
using drake::geometry::Shape;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::CenicIntegrator;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::systems::Simulator;
using Eigen::Vector3d;
using Eigen::Vector4d;

DEFINE_double(simulation_time, 2.0, "Duration of the simulation [s].");

DEFINE_double(
    time_step, 0.0,
    "Discrete period of the MultibodyPlant [s]. 0 (the default) uses a "
    "continuous plant integrated by CENIC/ICF; a value > 0 uses a discrete "
    "plant with its built-in contact solver as a baseline, in which case the "
    "CENIC-specific flags (--fixed_step, --accuracy, --max_step_size, "
    "--num_threads) are ignored.");

DEFINE_bool(spheres, false, "If true, simulate spheres instead of boxes.");

DEFINE_bool(hydroelastic, false,
            "If true, use hydroelastic contact instead of point contact.");

DEFINE_bool(
    fixed_step, false,
    "CENIC only: if true, run in fixed-step mode using --max_step_size as the "
    "step (no error control). If false (the default), use error-controlled "
    "integration with target --accuracy.");

DEFINE_double(accuracy, 1.0e-3,
              "CENIC only: target accuracy for error-controlled integration. "
              "Smaller is more accurate (and slower). Ignored in --fixed_step "
              "mode.");

DEFINE_double(max_step_size, 0.1,
              "CENIC only: maximum integration step [s]. In --fixed_step mode "
              "this is the exact (constant) step size.");

DEFINE_int32(num_stacks, 8,
             "Number of object stacks, arranged in a roughly square grid "
             "centered on the origin. Each stack is an independent island.");

DEFINE_int32(boxes_per_stack, 10, "Number of objects in each stack.");

DEFINE_double(stack_spacing, 2.0,
              "Distance between neighboring stacks in the grid [m]. Must be "
              "large enough that stacks remain independent (non-touching).");

DEFINE_int32(
    num_threads, 1,
    "CENIC only: number of threads used to solve constraint islands in "
    "parallel. 1 (the default) is serial; 0 uses the maximum available "
    "(Parallelism::Max()). Results are independent of this value.");

DEFINE_bool(
    meshcat, false,
    "If true, start a Meshcat server, record the simulation, and replay the "
    "recorded animation at a natural speed when the run finishes (regardless "
    "of --target_realtime_rate).");

DEFINE_double(target_realtime_rate, 0.0,
              "Desired rate relative to real time. 0 runs as fast as possible. "
              "With --meshcat the animation is recorded and replayed at a "
              "natural speed afterward, so 0 is fine for visualization too.");

/* Populates `plant` with a grid of --num_stacks vertical stacks of
--boxes_per_stack free bodies (boxes or spheres) resting on a half-space floor.
Stacks are centered on the origin and spaced --stack_spacing apart so that, when
sufficiently separated, each stack is an independent constraint island. */
void BuildScene(MultibodyPlant<double>* plant) {
  const double half = 0.05;             // Box half-size or sphere radius [m].
  const double density = 1000.0;        // [kg/m^3].
  const double gap = 0.001;             // Initial vertical gap between objects.
  const double resolution_hint = 0.05;  // Hydroelastic mesh refinement [m].
  const double hydroelastic_modulus = 1.0e6;  // [Pa].
  const CoulombFriction<double> friction(0.5, 0.5);

  if (FLAGS_hydroelastic) {
    plant->set_contact_model(ContactModel::kHydroelasticWithFallback);
  }

  // Floor: a rigid half-space anchored to the world.
  ProximityProperties floor_props;
  geometry::AddContactMaterial({}, {}, friction, &floor_props);
  if (FLAGS_hydroelastic) {
    geometry::AddRigidHydroelasticProperties(&floor_props);
  }
  plant->RegisterCollisionGeometry(plant->world_body(), RigidTransformd(),
                                   HalfSpace(), "floor_collision", floor_props);
  plant->RegisterVisualGeometry(plant->world_body(), RigidTransformd(),
                                HalfSpace(), "floor_visual",
                                Vector4d(0.7, 0.7, 0.7, 1.0));

  // Per-object proximity properties (compliant when using hydroelastic).
  ProximityProperties body_props;
  geometry::AddContactMaterial({}, {}, friction, &body_props);
  if (FLAGS_hydroelastic) {
    geometry::AddCompliantHydroelasticProperties(
        resolution_hint, hydroelastic_modulus, &body_props);
  }

  // Object shape and inertia.
  const Box box(2 * half, 2 * half, 2 * half);
  const Sphere sphere(half);
  const Shape& shape = FLAGS_spheres ? static_cast<const Shape&>(sphere)
                                     : static_cast<const Shape&>(box);
  const SpatialInertia<double> inertia =
      FLAGS_spheres
          ? SpatialInertia<double>::SolidSphereWithDensity(density, half)
          : SpatialInertia<double>::SolidBoxWithDensity(density, 2 * half,
                                                        2 * half, 2 * half);

  // Lay the stacks out on a roughly square grid centered on the origin.
  const int grid = static_cast<int>(std::ceil(std::sqrt(FLAGS_num_stacks)));
  const double center = (grid - 1) / 2.0;
  for (int k = 0; k < FLAGS_num_stacks; ++k) {
    const int i = k % grid;
    const int j = k / grid;
    const double x = (i - center) * FLAGS_stack_spacing;
    const double y = (j - center) * FLAGS_stack_spacing;
    // A 4-color checkerboard makes neighboring (independent) stacks distinct.
    const Vector4d color((i % 2 == 0) ? 0.9 : 0.2, (j % 2 == 0) ? 0.7 : 0.3,
                         0.3, 1.0);
    for (int b = 0; b < FLAGS_boxes_per_stack; ++b) {
      const std::string name = fmt::format("stack{}_body{}", k, b);
      const auto& body = plant->AddRigidBody(name, inertia);
      plant->RegisterCollisionGeometry(body, RigidTransformd(), shape,
                                       name + "_collision", body_props);
      plant->RegisterVisualGeometry(body, RigidTransformd(), shape,
                                    name + "_visual", color);
      const double z = half + b * (2 * half + gap);
      plant->SetDefaultFloatingBaseBodyPose(body,
                                            RigidTransformd(Vector3d(x, y, z)));
    }
  }
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  // A zero --time_step builds a continuous plant (integrated by CENIC); a
  // positive --time_step builds a discrete plant that uses its own solver.
  auto& plant = AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step).plant;
  BuildScene(&plant);
  plant.Finalize();

  std::shared_ptr<geometry::Meshcat> meshcat;
  if (FLAGS_meshcat) {
    meshcat = std::make_shared<geometry::Meshcat>();
    visualization::AddDefaultVisualization(&builder, meshcat);
  }

  auto diagram = builder.Build();

  const bool use_cenic = (FLAGS_time_step == 0.0);

  Simulator<double> simulator(*diagram);
  if (use_cenic) {
    // Continuous plant: integrate with CENIC and its (optional) per-island
    // parallel solver.
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
    fmt::print("Integrator: CENIC, {}; island solver threads: {}.\n",
               FLAGS_fixed_step
                   ? fmt::format("fixed step ({} s)", FLAGS_max_step_size)
                   : fmt::format("error control (accuracy {})", FLAGS_accuracy),
               parallelism.num_threads());
  } else {
    // Discrete plant: the Simulator advances it with the plant's built-in
    // contact solver. This is the discrete-time baseline.
    fmt::print(
        "Integrator: discrete MultibodyPlant baseline (time step {} s).\n",
        FLAGS_time_step);
  }

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  fmt::print(
      "Simulating {} stacks of {} {} ({} bodies) for {} s; {} contact.\n",
      FLAGS_num_stacks, FLAGS_boxes_per_stack,
      FLAGS_spheres ? "spheres" : "boxes",
      FLAGS_num_stacks * FLAGS_boxes_per_stack, FLAGS_simulation_time,
      FLAGS_hydroelastic ? "hydroelastic" : "point");

  // Record the trajectory into a Meshcat animation so it can be replayed at a
  // natural speed after the (possibly much faster than real-time) simulation.
  if (meshcat != nullptr) {
    meshcat->StartRecording();
  }

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
    // Publish the recorded animation; it plays back automatically in the
    // browser. Keep the process (and thus the Meshcat server) alive so the
    // user can watch and replay it.
    meshcat->StopRecording();
    meshcat->PublishRecording();
    fmt::print(
        "\nAnimation published to Meshcat at {} and now playing back.\n"
        "Use the controls panel to replay it. Press Enter to exit.\n",
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
      "Simulates a grid of independent stacks of boxes or spheres with Drake's "
      "CENIC integrator (or a discrete-MultibodyPlant baseline), demonstrating "
      "optional per-island parallelism. See the flags for Meshcat "
      "visualization, point vs. hydroelastic contact, fixed-step vs. "
      "error-controlled integration, accuracy, and the number of solver "
      "threads.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::box_stacks::do_main();
}
