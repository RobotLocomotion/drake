#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix2d;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::GeometrySystem;
using multibody::benchmarks::inclined_plane::MakeInclinedPlanePlant;
using systems::Context;
using systems::DiagramBuilder;

namespace multibody {
namespace multibody_plant {
namespace {

// This test creates a simple model for an acrobot using MultibodyPlant and
// verifies a number of invariants such as that body and joint models were
// properly added and the model sizes.
GTEST_TEST(MultibodyPlant, RollingSphereTest) {
  systems::DiagramBuilder<double> builder;

  GeometrySystem<double>& geometry_system =
      *builder.AddSystem<GeometrySystem>();
  geometry_system.set_name("geometry_system");

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 1.0e-6;

  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double radius = 0.05;   // Rolling sphere radius, [m]
  const double mass = 0.1;      // Rolling sphere mass, [kg]
  const double g = 9.81;        // Acceleration of gravity, [m/s²]
  const double slope = 15.0 / 180 * M_PI;  // Inclined plane's slope, [rad]
  const CoulombFrictionCoefficients surface_friction(
      1.0 /* static friction */, 0.5 /* dynamic friction */);

  // Contact parameters. Results converge to the analytical solution as the
  // penetration allowance and the stiction tolerance go to zero.
  const double penetration_allowance = 0.001;  // [m]
  // Stribeck approximation stiction velocity tolerance, [m/s].
  const double stiction_tolerance = 0.001;

  MultibodyPlant<double>& plant =
      *builder.AddSystem(MakeInclinedPlanePlant(
          radius, mass, slope, surface_friction, g, &geometry_system));
  const MultibodyTree<double>& model = plant.model();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(penetration_allowance);
  plant.set_stiction_tolerance(stiction_tolerance);
  const RigidBody<double>& ball = model.GetRigidBodyByName("Ball");

  // Hint the integrator's time step based on the contact time scale.
  // A fraction of this time scale is used which is chosen so that the fixed
  // time step integrators are stable.
  const double max_time_step =
      plant.get_contact_penalty_method_time_scale() / 30;

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_ids_output_port(),
      geometry_system.get_source_frame_id_port(
          plant.get_source_id().value()));
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      geometry_system.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(geometry_system.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // This will set a default initial condition with the sphere located at
  // p_WBcm = (0; 0; 0) and zero spatial velocity.
  model.SetDefaultContext(&plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>* integrator =
      simulator.get_mutable_integrator();
  integrator->set_maximum_step_size(max_time_step);
  integrator->set_target_accuracy(target_accuracy);
  simulator.set_publish_every_time_step(true);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // Compute the kinetic energy of B (in frame W) from V_WB.
  const SpatialVelocity<double>& V_WB =
      model.EvalBodySpatialVelocityInWorld(plant_context, ball);
  const SpatialInertia<double> M_BBo_B = ball.default_spatial_inertia();
  const Isometry3<double>& X_WB =
      model.EvalBodyPoseInWorld(plant_context, ball);
  const SpatialInertia<double> M_BBo_W = M_BBo_B.ReExpress(X_WB.linear());
  const double ke_WB = 0.5 * V_WB.dot(M_BBo_W * V_WB);
  const double speed = V_WB.translational().norm();
  const double angular_velocity = V_WB.rotational().y();

  // Height traveled by the sphere.
  const double h = std::abs(X_WB.translation().z());
  // This change in height must have been transferred into kinetic energy.
  const double ke_WB_expected = mass * g * h;

  // Sphere's unit inertia.
  const double G_Bcm = 0.4 * radius * radius;
  const double speed_expected =
      std::sqrt(2 * g * h / (1.0 + G_Bcm / radius / radius));
  // Expected angular velocity for when there is no slipping.
  const double angular_velocity_expected = speed_expected / radius;

  // Verify the plant's potential energy matches the analytical calculation.
  const double Ve = model.CalcPotentialEnergy(plant_context);
  EXPECT_NEAR(-Ve, ke_WB_expected, std::numeric_limits<double>::epsilon());

  // Verify the relative errors are below 1%. For this case errors are dominated
  // by the penetration allowance and the stiction tolerance since the
  // integrator's accuracy was set to a relatively small value.
  EXPECT_TRUE(std::abs(ke_WB - ke_WB_expected) / ke_WB < 0.01);
  EXPECT_TRUE(std::abs(speed - speed_expected) / speed_expected < 0.01);
  EXPECT_TRUE(std::abs(angular_velocity - angular_velocity_expected)
                  / angular_velocity_expected < 0.01);

#if 0
  PRINT_VAR("energy");
  PRINT_VAR(ke_WB);
  PRINT_VAR(ke_WB_expected);
  PRINT_VAR(Ve);
  PRINT_VAR(std::abs(ke_WB - ke_WB_expected) / ke_WB);

  PRINT_VAR("Velocity");
  PRINT_VAR(speed_expected);
  PRINT_VAR(V_WB.translational().norm());
  PRINT_VAR(std::abs(speed_expected - V_WB.translational().norm()) / speed_expected);

  PRINT_VAR("Angular velocity");
  PRINT_VAR(angular_velocity_expected);
  PRINT_VAR(V_WB.rotational().y());
  PRINT_VAR(std::abs(angular_velocity_expected - V_WB.rotational().y()) / angular_velocity_expected);

  PRINT_VAR("More stuff");
  PRINT_VAR(X_WB.translation().transpose());
  PRINT_VAR(V_WB);
#endif
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

