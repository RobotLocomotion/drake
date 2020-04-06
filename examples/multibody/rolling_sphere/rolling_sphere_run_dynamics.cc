#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/geometry_instance.h"
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
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(target_realtime_rate, 0.2,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'.");

// Integration parameters.
DEFINE_double(simulation_time, 2.0,
              "Desired duration of the simulation in seconds.");
DEFINE_double(accuracy, 1.0e-3, "The integration accuracy.");
DEFINE_double(max_time_step, 1.0e-3,
              "The maximum time step the integrator is allowed to take, [s].");

// Contact model parameters.
DEFINE_string(contact_model, "point",
              "Contact model. Options are: 'point', 'hydroelastic', 'hybrid'.");
DEFINE_double(elastic_modulus, 5.0e4,
              "For hydroelastic (and hybrid) contact, elastic modulus, [Pa].");
DEFINE_double(dissipation, 5.0,
              "For hydroelastic (and hybrid) contact, Hunt & Crossley "
              "dissipation, [s/m].");
DEFINE_double(friction_coefficient, 0.3, "friction coefficient.");
DEFINE_bool(rigid_ball, false,
            "If true, the ball is given a rigid hydroelastic representation "
            "(instead of the default soft value). Make sure you have the right "
            "contact model to support this representation.");
DEFINE_bool(soft_ground, false,
            "If true, the ground is given a soft hydroelastic representation "
            "(instead of the default rigid value). Make sure you have the "
            "right contact model to support this representation.");
DEFINE_bool(add_wall, false,
            "If true, adds a wall with soft hydroelastic representation in the "
            "path of the default ball trajectory. This will cause the "
            "simulation to throw when the soft ball hits the wall with the "
            "'hydroelastic' model; use the 'hybrid' or 'point' contact model "
            "to simulate beyond this contact.");

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
DEFINE_double(pitch, 0.0, "Sphere's initial pitch in degrees.");
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
using Eigen::Vector4d;
using drake::geometry::SceneGraph;
using drake::geometry::SourceId;
using drake::lcm::DrakeLcm;
using drake::math::RigidTransformd;
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
      -g * Vector3d::UnitZ(), FLAGS_rigid_ball, FLAGS_soft_ground,
      &scene_graph));

  if (FLAGS_add_wall) {
    geometry::Box wall{0.2, 4, 0.4};
    const RigidTransformd X_WB(Vector3d{-0.5, 0, 0});
    geometry::ProximityProperties prox_prop;
    geometry::AddContactMaterial(1e8, {}, CoulombFriction<double>(),
                                 &prox_prop);
    geometry::AddSoftHydroelasticProperties(0.1, &prox_prop);
    plant.RegisterCollisionGeometry(plant.world_body(), X_WB, wall,
                                    "wall_collision", std::move(prox_prop));

    geometry::IllustrationProperties illus_prop;
    illus_prop.AddProperty("phong", "diffuse", Vector4d(0.7, 0.5, 0.4, 0.5));
    plant.RegisterVisualGeometry(plant.world_body(), X_WB, wall, "wall_visual",
                                 std::move(illus_prop));
  }

  // Set contact model and parameters.
  if (FLAGS_contact_model == "hydroelastic") {
    plant.set_contact_model(ContactModel::kHydroelasticsOnly);
    plant.Finalize();
  } else if (FLAGS_contact_model == "point") {
    // Plant must be finalized before setting the penetration allowance.
    plant.Finalize();
    // Set how much penetration (in meters) we are willing to accept.
    plant.set_penetration_allowance(0.001);
  } else if (FLAGS_contact_model == "hybrid") {
    plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
    plant.Finalize();
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
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        &simulator.reset_integrator<ImplicitEulerIntegrator<double>>();
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator = &simulator.reset_integrator<RungeKutta2Integrator<double>>(
        FLAGS_max_time_step);
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        &simulator.reset_integrator<RungeKutta3Integrator<double>>();
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        &simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            FLAGS_max_time_step);
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
        "' not supported for this example.");
  }
  integrator->set_maximum_step_size(FLAGS_max_time_step);

  // Error control is only supported for variable time step integrators.
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

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
    const int kNumEvaluationsPerStep =
        FLAGS_integration_scheme == "runge_kutta2"? 2 : 1;
    DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
        integrator->get_num_steps_taken() * kNumEvaluationsPerStep);
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
