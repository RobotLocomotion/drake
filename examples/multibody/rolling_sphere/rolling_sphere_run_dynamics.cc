#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(target_realtime_rate, 0.2,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'radau1', 'implicit_euler' (ec), 'velocity_implicit_euler' (ec), 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");

// Integration parameters.
DEFINE_double(simulation_time, 2.0,
              "Desired duration of the simulation in seconds.");
DEFINE_double(accuracy, 1.0e-3, "The integration accuracy.");
DEFINE_double(max_time_step, 1.0e-3,
              "The maximum time step the integrator is allowed to take, [s].");
DEFINE_bool(error_control, false, "If 'true', integrator uses error control.");

// Contact model parameters.
DEFINE_string(contact_model, "point",
              "Contact model. Options are: 'point', 'hydroelastic'.");
DEFINE_double(elastic_modulus, 5.0e4,
              "For hydroelastics, elastic modulus, [Pa].");
DEFINE_double(dissipation, 5.0,
              "For hydroelastics, Hunt & Crossley dissipation, [s/m].");
DEFINE_double(friction_coefficient, 0.3, "friction coefficient.");

// Sphere's spatial velocity.
DEFINE_double(vx, 1.5,
              "Sphere's initial translational velocity in the x-axis in m/s.");
DEFINE_double(vy, 0.0,
              "Sphere's initial translational velocity in the y-axis in m/s.");
DEFINE_double(vz, 0.0,
              "Sphere's initial translational velocity in the z-axis in m/s.");
DEFINE_double(wx, 0.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wy, -360.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");

// Sphere's pose.
DEFINE_double(roll, 0.0, "Sphere's initial roll in degrees.");
DEFINE_double(pitch, 0.0, "Sphere's initial pitch in degrees");
DEFINE_double(yaw, 0.0, "Sphere's initial yaw in degrees.");
DEFINE_double(z0, 0.05, "Sphere's initial position in the z-axis.");

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using drake::geometry::SceneGraph;
using drake::geometry::SourceId;
using drake::lcm::DrakeLcm;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialVelocity;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::RungeKutta2Integrator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::SemiExplicitEulerIntegrator;

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = FLAGS_accuracy;

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const double z0 = FLAGS_z0;        // Initial height.
  const CoulombFriction<double> coulomb_friction(
      FLAGS_friction_coefficient /* static friction */,
      FLAGS_friction_coefficient /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeBouncingBallPlant(
      radius, mass, FLAGS_elastic_modulus, FLAGS_dissipation, coulomb_friction,
      -g * Vector3d::UnitZ(), &scene_graph));

  // Set contact model and parameters.
  if (FLAGS_contact_model == "hydroelastic") {
    plant.set_contact_model(ContactModel::kHydroelasticsOnly);
    plant.Finalize();
  } else if (FLAGS_contact_model == "point") {
    // Plant must be finalized before setting the penetration allowance.
    plant.Finalize();
    // Set how much penetration (in meters) we are willing to accept.
    plant.set_penetration_allowance(0.001);
  } else {
    throw std::runtime_error("Invalid contact model '" + FLAGS_contact_model +
                             "'.");
  }

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  ConnectContactResultsToDrakeVisualizer(&builder, plant);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the sphere's initial pose.
  math::RotationMatrixd R_WB(math::RollPitchYawd(
      M_PI / 180.0 * Vector3<double>(FLAGS_roll, FLAGS_pitch, FLAGS_yaw)));
  math::RigidTransformd X_WB(R_WB, Vector3d(0.0, 0.0, z0));
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName("Ball"), X_WB);

  const SpatialVelocity<double> V_WB(Vector3d(FLAGS_wx, FLAGS_wy, FLAGS_wz),
                                     Vector3d(FLAGS_vx, FLAGS_vy, FLAGS_vz));
  plant.SetFreeBodySpatialVelocity(
      &plant_context, plant.GetBodyByName("Ball"), V_WB);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  // for VIE statistics
  bool vie = false;
  bool implicit = false;
  systems::VelocityImplicitEulerIntegrator<double>* vie_integrator{nullptr};
  systems::ImplicitIntegrator<double>* implicit_integrator{nullptr};
  
  if (FLAGS_integration_scheme == "implicit_euler") {
    implicit_integrator =
        simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    implicit = true;
    integrator = implicit_integrator;
    integrator->set_target_accuracy(target_accuracy);
    integrator->set_fixed_step_mode(!FLAGS_error_control);
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "velocity_implicit_euler") {
    vie_integrator =
        simulator.reset_integrator<systems::VelocityImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    vie = true;
    implicit_integrator = vie_integrator;
    implicit = true;
    integrator = implicit_integrator;
    integrator->set_target_accuracy(target_accuracy);
    integrator->set_fixed_step_mode(!FLAGS_error_control);
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
  integrator->set_maximum_step_size(FLAGS_max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);
  
  integrator->set_fixed_step_mode(!FLAGS_error_control);

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();


  simulator.AdvanceTo(FLAGS_simulation_time);


  
  fmt::print("Stats for integrator {}:\n", FLAGS_integration_scheme);
  fmt::print("Number of time steps taken = {:d}\n",
              vie ? vie_integrator->get_vie_num_steps_taken() : integrator->get_num_steps_taken());
  if (!integrator->get_fixed_step_mode()) {
    fmt::print("Initial time step taken = {:10.6g} s\n",
                vie ? vie_integrator->get_vie_actual_initial_step_size_taken() : 
                integrator->get_actual_initial_step_size_taken());
    fmt::print("Largest time step taken = {:10.6g} s\n",
                vie ? vie_integrator->get_vie_largest_step_size_taken() : 
                integrator->get_largest_step_size_taken());
    fmt::print("Smallest adapted step size = {:10.6g} s\n",
                vie ? vie_integrator->get_vie_smallest_adapted_step_size_taken() :
                integrator->get_smallest_adapted_step_size_taken());
    fmt::print("Number of steps shrunk due to error control = {:d}\n",
                integrator->get_num_step_shrinkages_from_error_control());
    fmt::print("Number of steps shrunk due to convergence-based failure = {:d}\n",
                integrator->get_num_step_shrinkages_from_substep_failures());
    fmt::print("Number of convergence-based step failures (should match) = {:d}\n",
                integrator->get_num_substep_failures());
  }
  if(implicit)
  {
    if(vie)
      fmt::print("Implicit Integrator Statistics (total, half-size-steps):\n");
    else
      fmt::print("Implicit Integrator Statistics (total, error estimator):\n");
    fmt::print("Number of Derivative Evaluations = {:d}, {:d} \n",
                implicit_integrator->get_num_derivative_evaluations(),
                implicit_integrator->get_num_error_estimator_derivative_evaluations());
    fmt::print("Number of Jacobian Computations = {:d}, {:d} \n",
                implicit_integrator->get_num_jacobian_evaluations(),
                implicit_integrator->get_num_error_estimator_jacobian_evaluations());
    fmt::print("Number of Derivative Evaluations for Jacobians = {:d}, {:d} \n",
                implicit_integrator->get_num_derivative_evaluations_for_jacobian(),
                implicit_integrator->get_num_error_estimator_derivative_evaluations_for_jacobian());
    fmt::print("Number of Iteration Matrix Factorizations = {:d}, {:d} \n",
                implicit_integrator->get_num_iteration_matrix_factorizations(),
                implicit_integrator->get_num_error_estimator_iteration_matrix_factorizations());
    fmt::print("Number of Newton-Raphson Iterations = {:d}, {:d} \n",
                implicit_integrator->get_num_newton_raphson_iterations(),
                implicit_integrator->get_num_error_estimator_newton_raphson_iterations());
    fmt::print("Number of Newton-Raphson Iterations That Lead to Failure = {:d}, {:d} \n",
                implicit_integrator->get_num_newton_raphson_iterations_that_end_in_failure(),
                implicit_integrator->get_num_error_estimator_newton_raphson_iterations_that_end_in_failure());
    fmt::print("Number of Newton-Raphson Failures = {:d}, {:d} \n",
                implicit_integrator->get_num_newton_raphson_failures(),
                implicit_integrator->get_num_error_estimator_newton_raphson_failures());
  }
  
  
  // Some sanity checks:
  if (FLAGS_integration_scheme == "semi_explicit_euler") {
    DRAKE_DEMAND(integrator->get_fixed_step_mode() == true);
  }

  // Checks for variable time step integrators.
  if (!integrator->get_fixed_step_mode()) {
    // From IntegratorBase::set_maximum_step_size():
    // "The integrator may stretch the maximum step size by as much as 1% to
    // reach discrete event." Thus the 1.01 factor in this DRAKE_DEMAND.
    DRAKE_DEMAND(integrator->get_largest_step_size_taken() <=
                 1.01 * FLAGS_max_time_step);
    DRAKE_DEMAND(integrator->get_smallest_adapted_step_size_taken() <=
                 integrator->get_largest_step_size_taken());
    DRAKE_DEMAND(integrator->get_num_steps_taken() >=
                 FLAGS_simulation_time / FLAGS_max_time_step);
  }

  // Checks for fixed time step integrators.
  if (integrator->get_fixed_step_mode()) {
    ///const int kNumEvaluationsPerStep =
     //   FLAGS_integration_scheme == "runge_kutta2"? 2 : 1;
   // DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
    //    integrator->get_num_steps_taken() * kNumEvaluationsPerStep);
    DRAKE_DEMAND(
        integrator->get_num_step_shrinkages_from_error_control() == 0);
  }

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bouncing_ball::do_main();
}
