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
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace examples {
namespace {

// Simulation parameters.
DEFINE_double(simulation_time, 10.0, "Simulation duration in seconds");
DEFINE_double(
    mbp_time_step, 1.0E-2,
    "If mbp_time_step > 0, the fixed-time step period (in seconds) of discrete "
    "updates for the plant (modeled as a discrete system). "
    "If mbp_time_step = 0, the plant is modeled as a continuous system "
    "and no contact forces are displayed.  mbp_time_step must be >= 0.");

// Physical parameters.
DEFINE_bool(use_hydro, true, "If true, use hydro. Otherwise point contact.");
DEFINE_double(stiction_tolerance, 1.0e-4, "Stiction tolerance [m/s].");

// Visualization.
DEFINE_bool(visualize, true, "Whether to visualize (true) or not (false).");
DEFINE_bool(visualize_forces, true,
            "Whether to visualize forces (true) or not (false).");
DEFINE_bool(visualize_proximity, false,
            "Whether to visualize collision geometry (true) or not (false).");
DEFINE_double(viz_period, 1.0 / 60.0, "Viz period.");

// Discrete contact solver.
DEFINE_string(discrete_contact_approximation, "lagged",
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

// PID controller parameters
DEFINE_double(kp, 500, "Proportional gain for PID controller.");
DEFINE_double(kd, 50, "Derivative gain for PID controller.");
DEFINE_double(ki, 10, "Integral gain for PID controller.");

// Whether to include a box to hold
DEFINE_bool(manipuland, false, "Include a manipuland in the simulation.");

using drake::geometry::CollisionFilterDeclaration;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::ContactResults;
using drake::multibody::MultibodyPlant;
using drake::systems::ConstantVectorSource;
using drake::systems::ConvexIntegrator;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::IntegratorBase;
using drake::systems::controllers::PidController;
using Eigen::MatrixXd;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using clock = std::chrono::steady_clock;
using drake::geometry::SceneGraphConfig;
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

  // Enable hydroelastic contact
  if (FLAGS_use_hydro) {
    SceneGraphConfig sg_config;
    sg_config.default_proximity_properties.compliance_type = "compliant";
    sg_config.default_proximity_properties.resolution_hint = 0.01;
    scene_graph.set_config(sg_config);
  }

  // Add the franka arm model
  multibody::Parser parser(&plant);
  parser.AddModelsFromUrl(
      "package://drake_models/franka_description/urdf/panda_arm_hand.urdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

  // Add an object to hold, if requested
  if (FLAGS_manipuland) {
    parser.AddModelsFromUrl("package://drake_models/ycb/003_cracker_box.sdf");
  }

  plant.Finalize();

  fmt::print("Num positions: {:d}\n", plant.num_positions());
  fmt::print("Num velocities: {:d}\n", plant.num_velocities());

  // Set up the meshcat visualizer
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();
  if (FLAGS_visualize) {
    VisualizationConfig vis_config;
    vis_config.publish_period = FLAGS_viz_period;
    vis_config.publish_contacts = FLAGS_visualize_forces;
    vis_config.publish_proximity = FLAGS_visualize_proximity;
    ApplyVisualizationConfig(vis_config, &builder, nullptr, &plant,
                             &scene_graph, meshcat);
  }

  // Define a target state
  VectorXd q_nom(plant.num_positions());
  VectorXd x_nom(plant.num_multibody_states());

  if (FLAGS_manipuland) {
    q_nom << 0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4, 0.01, 0.01,  // robot
        0.707, 0.707, 0, 0, 0.55, 0.0, 0.43;                   // box
  } else {
    q_nom << 0, 0, 0, -M_PI_2, 0, M_PI_2, 0, 0.01, 0.01;
  }
  x_nom << q_nom, VectorXd::Zero(plant.num_velocities());

  // Add a PID controller
  MatrixXd Px = MatrixXd::Identity(plant.num_multibody_states(),
                                   plant.num_multibody_states());

  if (FLAGS_manipuland) {
    Px.resize(18, 31);
    Px.setZero();
    Px.block<9, 9>(0, 0) = MatrixXd::Identity(9, 9);
    Px.block<9, 9>(9, 16) = MatrixXd::Identity(9, 9);
  }

  VectorXd Kp(9), Kd(9), Ki(9);
  Kp << FLAGS_kp, FLAGS_kp, FLAGS_kp, FLAGS_kp, FLAGS_kp, FLAGS_kp, FLAGS_kp,
      0.2 * FLAGS_kp, 0.2 * FLAGS_kp;
  Kd << FLAGS_kd, FLAGS_kd, FLAGS_kd, FLAGS_kd, FLAGS_kd, FLAGS_kd, FLAGS_kd,
      0.2 * FLAGS_kd, 0.2 * FLAGS_kd;
  Ki << FLAGS_ki, FLAGS_ki, FLAGS_ki, FLAGS_ki, FLAGS_ki, FLAGS_ki, FLAGS_ki,
      0.2 * FLAGS_ki, 0.2 * FLAGS_ki;

  auto pid_controller = builder.AddSystem<PidController>(Px, Kp, Ki, Kd);
  auto nominal_state_source =
      builder.AddSystem<ConstantVectorSource>(Px * x_nom);

  builder.Connect(nominal_state_source->get_output_port(),
                  pid_controller->get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(pid_controller->get_output_port_control(),
                  plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  plant.SetDefaultContext(&plant_context);

  // Set initial condition. The gripper is open a bit wider than in q_nom
  VectorXd q_init = q_nom;
  q_init(7) = 0.035;
  q_init(8) = 0.035;
  plant.SetPositions(&plant_context, q_init);

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

  // Wait for user input again b/c meshcat is too slow to publish the recording
  if (FLAGS_visualize) {
    std::cout << "Press any key to quit ...\n";
    getchar();
  }

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
