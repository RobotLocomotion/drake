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
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/convex_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/joint_stiffness_controller.h"
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
    mbp_time_step, 0.0,
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
DEFINE_double(viz_period, std::numeric_limits<double>::infinity(),
              "Viz period.");

// Discrete contact solver.
DEFINE_string(discrete_contact_approximation, "lagged",
              "Discrete contact solver. Options are: 'tamsi', 'sap', 'lagged', "
              "'similar'.");
DEFINE_double(near_rigid_threshold, 1.0, "SAP near rigid threshold.");
DEFINE_bool(dense_algebra, false, "Whether to use dense algebra in SAP.");

// Continuous integration parameters
DEFINE_string(
    integrator_jacobian_scheme, "forward",
    "Jacobian computation scheme: 'forward', 'central', 'automatic'.");
DEFINE_bool(full_newton, false, "Update Jacobian every iteration.");
DEFINE_bool(save_csv, false, "Save CSV data for the convex integrator.");
DEFINE_bool(trapezoid, false, "Implicit trapezoid rule for error estimation.");

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
DEFINE_double(tolerance, 1e-8, "Main solver tolerance");

// Which controller to use
DEFINE_string(
    controller, "joint_stiffness",
    "Controller type: 'pid', 'inverse_dynamics', or 'joint_stiffness'.");

// PID controller gains
DEFINE_double(kp, 1e3, "Proportional gain for PID controller.");
DEFINE_double(kd, 1e3, "Derivative gain for PID controller.");
DEFINE_double(ki, 0, "Integral gain for PID controller.");

// Whether to include a box to hold
DEFINE_bool(manipuland, true, "Include a manipuland in the simulation.");

using drake::geometry::CollisionFilterDeclaration;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::ContactResults;
using drake::multibody::MultibodyPlant;
using drake::systems::ConstantVectorSource;
using drake::systems::ConvexIntegrator;
using drake::systems::ConvexIntegratorSolverParameters;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::IntegratorBase;
using drake::systems::controllers::InverseDynamicsController;
using drake::systems::controllers::JointStiffnessController;
using drake::systems::controllers::PidController;
using Eigen::MatrixXd;
using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using clock = std::chrono::steady_clock;
using drake::geometry::SceneGraphConfig;
using drake::multibody::contact_solvers::internal::SapHessianFactorizationType;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::internal::CompliantContactManager;
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
  const std::string panda_url =
      "package://drake_models/franka_description/urdf/panda_arm_hand.urdf";
  multibody::ModelInstanceIndex panda = parser.AddModelsFromUrl(panda_url)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

  // Add an object to hold, if requested
  if (FLAGS_manipuland) {
    parser.AddModelsFromUrl("package://drake_models/ycb/003_cracker_box.sdf");
  }

  // Set up implicit PD controller gains if we're doing discrete-time sim
  if (plant.time_step() > 0.0) {
    for (JointActuatorIndex actuator_index : plant.GetJointActuatorIndices()) {
      JointActuator<double>& actuator =
          plant.get_mutable_joint_actuator(actuator_index);
      actuator.set_controller_gains({FLAGS_kp, FLAGS_kd});
    }
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
    q_nom << 0, -0.4, 0.5, -M_PI_2, 0, M_PI_2, M_PI_4, 0.01, 0.01,  // robot
        0.707, 0.707, 0, 0, 0.55, 0.0, 0.43;                        // box
  } else {
    q_nom << 0, -0.4, 0.5, -M_PI_2, 0, M_PI_2, 0, 0.01, 0.01;
  }
  x_nom << q_nom, VectorXd::Zero(plant.num_velocities());

  // Set controller gains
  MatrixXd Px = MatrixXd::Identity(plant.num_multibody_states(),
                                   plant.num_multibody_states());

  if (FLAGS_manipuland) {
    Px.resize(18, 31);
    Px.setZero();
    Px.block<9, 9>(0, 0) = MatrixXd::Identity(9, 9);
    Px.block<9, 9>(9, 16) = MatrixXd::Identity(9, 9);
  }

  VectorXd Kp(9), Kd(9), Ki(9);
  Kp.fill(FLAGS_kp);
  Kd.fill(FLAGS_kd);
  Ki.fill(FLAGS_ki);

  // System than outputs desired joint positions and velocities
  auto nominal_state_source =
      builder.AddSystem<ConstantVectorSource>(Px * x_nom);

  // Set up a simplified plant model for the controller, which does not include
  // the manipuland.
  MultibodyPlant<double> control_plant(1.0);
  Parser(&control_plant).AddModelsFromUrl(panda_url);
  control_plant.WeldFrames(control_plant.world_frame(),
                           control_plant.GetFrameByName("panda_link0"));
  control_plant.Finalize();

  // Select the model instance for the franka itself, which we'll use to get the
  // state of the robot but not the manipuland.
  ModelInstanceIndex robot = plant.GetModelInstanceByName("panda");

  // Create and connect the controller
  if (plant.time_step() > 0.0) {
    // We'll use a discrete-time implicit PID controller
    if (FLAGS_controller != "pid") {
      throw std::runtime_error(
          "Discrete-time simulation only supports an implicit PID controller.");
    }
    builder.Connect(nominal_state_source->get_output_port(),
                    plant.get_desired_state_input_port(robot));

    // And force SAP to use dense algebra
    SapSolverParameters sap_parameters;
    sap_parameters.linear_solver_type = SapHessianFactorizationType::kDense;

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    CompliantContactManager<double>* contact_manager =
        owned_contact_manager.get();
    plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    contact_manager->set_sap_solver_parameters(sap_parameters);
  } else {
    if (FLAGS_controller == "pid") {
      auto controller = builder.AddSystem<PidController>(Px, Kp, Ki, Kd);

      builder.Connect(nominal_state_source->get_output_port(),
                      controller->get_input_port_desired_state());
      builder.Connect(plant.get_state_output_port(),
                      controller->get_input_port_estimated_state());
      builder.Connect(controller->get_output_port_control(),
                      plant.get_actuation_input_port(panda));

    } else if (FLAGS_controller == "inverse_dynamics") {
      auto controller = builder.AddSystem<InverseDynamicsController>(
          control_plant, Kp, Ki, Kd, false);

      builder.Connect(nominal_state_source->get_output_port(),
                      controller->get_input_port_desired_state());
      builder.Connect(plant.get_state_output_port(robot),
                      controller->get_input_port_estimated_state());
      builder.Connect(controller->get_output_port_control(),
                      plant.get_actuation_input_port());

    } else if (FLAGS_controller == "joint_stiffness") {
      auto controller =
          builder.AddSystem<JointStiffnessController>(control_plant, Kp, Kd);

      builder.Connect(nominal_state_source->get_output_port(),
                      controller->get_input_port_desired_state());
      builder.Connect(plant.get_state_output_port(robot),
                      controller->get_input_port_estimated_state());
      builder.Connect(controller->get_output_port_actuation(),
                      plant.get_actuation_input_port());

    } else {
      throw std::runtime_error(
          "Unknown controller, options are 'pid', 'inverse_dynamics', and "
          "'joint_stiffness'");
    }
  }

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
  q_init(1) = 0.0;
  q_init(2) = 0.0;
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
    ci.set_plant(&plant);

    ConvexIntegratorSolverParameters ci_params;
    ci_params.enable_hessian_reuse = FLAGS_enable_hessian_reuse;
    ci_params.max_iterations_for_hessian_reuse = FLAGS_k_max;
    ci_params.kappa = FLAGS_kappa;
    ci_params.alpha_max = FLAGS_alpha_max;
    ci_params.ls_tolerance = FLAGS_ls_tolerance;
    ci_params.tolerance = FLAGS_tolerance;
    ci_params.log_solver_stats = FLAGS_log_solver_stats;
    ci_params.print_solver_stats = FLAGS_print_solver_stats;
    ci_params.use_dense_algebra = FLAGS_dense_algebra;
    ci.set_solver_parameters(ci_params);
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

  // Set some reasonable defaults for the simulator options (these can be
  // overridden from the command line).
  FLAGS_simulator_integration_scheme = "convex";
  FLAGS_simulator_accuracy = 0.1;
  FLAGS_simulator_max_time_step = 0.01;
  FLAGS_simulator_use_error_control = true;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::do_main();
}
