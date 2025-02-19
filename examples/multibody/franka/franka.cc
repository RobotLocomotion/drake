#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/convex_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace examples {
namespace {

constexpr double kHuge = 1.0e40;

// Simulation parameters.
DEFINE_double(simulation_time, 10.0, "Simulation duration in seconds");
DEFINE_double(
    mbp_time_step, 1.0E-2,
    "If mbp_time_step > 0, the fixed-time step period (in seconds) of discrete "
    "updates for the plant (modeled as a discrete system). "
    "If mbp_time_step = 0, the plant is modeled as a continuous system "
    "and no contact forces are displayed.  mbp_time_step must be >= 0.");

// Physical parameters.
DEFINE_double(density, 1000.0, "The density of all objects, in kg/m³.");
DEFINE_double(friction_coefficient, 1.0,
              "All friction coefficients have this value.");
DEFINE_double(box_stiffness, 1.0e5, "Box point contact stiffness in N/m.");
DEFINE_double(sphere_stiffness, 1.0e5,
              "Sphere point contact stiffness in N/m.");
DEFINE_bool(use_hydro, false, "If true, use hydro. Otherwise point contact.");
DEFINE_double(sphere_resolution, 0.02, "Resolution hint for the sphere");
DEFINE_double(dissipation_time_constant, 0.01,
              "Dissipation time constant in seconds.");
DEFINE_double(hc_dissipation, 10.0, "Hunt & Crossley dissipation [s/m].");
DEFINE_double(stiction_tolerance, 1.0e-4, "Stiction tolerance [m/s].");

// Contact geometry parameters.
DEFINE_bool(
    emulate_box_multicontact, true,
    "Emulate multicontact by adding spheres to the faces of box geometries.");
DEFINE_int32(
    num_spheres_per_face, 3,
    "Multi-contact emulation. We place num_sphere x num_spheres_per_face on "
    "each box face, when emulate_box_multicontact = true.");
DEFINE_bool(enable_box_box_collision, false, "Enable box vs. box contact.");
DEFINE_bool(add_box_corners, false,
            "Adds collision points at the corners of each box.");

// Scenario parameters.
DEFINE_int32(objects_per_pile, 5, "Number of objects per pile.");
DEFINE_double(dz, 0.15, "Initial distance between objects in the pile.");
DEFINE_double(scale_factor, 1.0, "Multiplicative factor to generate the pile.");
DEFINE_bool(add_sink_walls, true, "Adds wall of a sink model.");
DEFINE_bool(enable_boxes, false, "Make some of the objects boxes.");

// Visualization.
DEFINE_bool(visualize, true, "Whether to visualize (true) or not (false).");
DEFINE_bool(visualize_forces, false,
            "Whether to visualize forces (true) or not (false).");
DEFINE_double(viz_period, 1.0 / 60.0, "Viz period.");

// Discrete contact solver.
DEFINE_string(discrete_contact_approximation, "sap",
              "Discrete contact solver. Options are: 'tamsi', 'sap', 'lagged', "
              "'similar'.");
DEFINE_double(near_rigid_threshold, 1.0, "SAP near rigid threshold.");

// Continuous integration parameters
DEFINE_string(
    integrator_jacobian_scheme, "forward",
    "Jacobian computation scheme: 'forward', 'central', 'automatic'.");
DEFINE_bool(full_newton, false, "Update Jacobian every iteration.");
DEFINE_bool(save_csv, false, "Save CSV data for the convex integrator.");
DEFINE_bool(trapezoid, false, "Implicit trapezoid rule for error estimation.");

using drake::geometry::CollisionFilterDeclaration;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::ContactResults;
using drake::multibody::MultibodyPlant;
using drake::systems::ConvexIntegrator;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::IntegratorBase;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using clock = std::chrono::steady_clock;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;

int do_main() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_time_step;
  plant_config.discrete_contact_approximation =
      FLAGS_discrete_contact_approximation;
  plant_config.sap_near_rigid_threshold = FLAGS_near_rigid_threshold;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  plant_config.contact_model = FLAGS_use_hydro ? "hydroelastic" : "point";
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);

  // Add the franka arm model
  multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake_models/franka_description/urdf/panda_arm_hand.urdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

  // TODO(vincekurtz): Add a manipuland
  plant.Finalize();

  fmt::print("Num positions: {:d}\n", plant.num_positions());
  fmt::print("Num velocities: {:d}\n", plant.num_velocities());

  // Set up the meshcat visualizer
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  if (FLAGS_visualize) {
    VisualizationConfig vis_config;
    vis_config.publish_period = FLAGS_viz_period;
    vis_config.publish_contacts = FLAGS_visualize_forces;
    ApplyVisualizationConfig(vis_config, &builder, nullptr, &plant,
                             &scene_graph, meshcat);
  }

  // TODO(vincekurtz): Add a PID controller
  VectorXd q_nom(9);
  q_nom << 0, 0, 0, -M_PI_2, 0, M_PI_2, 0, 0.01, 0.01;

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  plant.SetDefaultContext(&plant_context);

  // Set initial conditions
  plant.SetPositions(&plant_context, q_nom);

  // Set up the simulator with the specified integration scheme.
  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  drake::systems::IntegratorBase<double>& integrator =
      simulator->get_mutable_integrator();
  if (FLAGS_simulator_integration_scheme == "implicit_euler") {
    auto& ie = dynamic_cast<ImplicitEulerIntegrator<double>&>(integrator);
    using JacobianComputationScheme =
        ImplicitEulerIntegrator<double>::JacobianComputationScheme;
    if (FLAGS_integrator_jacobian_scheme == "forward") {
      ie.set_jacobian_computation_scheme(
          JacobianComputationScheme::kForwardDifference);
    } else if (FLAGS_integrator_jacobian_scheme == "central") {
      ie.set_jacobian_computation_scheme(
          JacobianComputationScheme::kCentralDifference);
    } else if (FLAGS_integrator_jacobian_scheme == "automatic") {
      ie.set_jacobian_computation_scheme(JacobianComputationScheme::kAutomatic);
    } else {
      throw std::logic_error("Invalid jacobian scheme");
    }
    ie.set_use_full_newton(FLAGS_full_newton);
    ie.set_use_implicit_trapezoid_error_estimation(FLAGS_trapezoid);
  }
  if (FLAGS_simulator_integration_scheme == "convex") {
    auto& ci = dynamic_cast<ConvexIntegrator<double>&>(integrator);
    ci.set_use_full_newton(FLAGS_full_newton);
    ci.set_write_to_csv(FLAGS_save_csv);
    ci.set_use_implicit_trapezoid_error_estimation(FLAGS_trapezoid);
  }

  simulator->set_publish_every_time_step(true);
  simulator->Initialize();
  if (FLAGS_visualize) {
    std::cout << "Press any key to continue ...\n";
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

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("\nSimulation of a franka arm with PID control.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::do_main();
}
