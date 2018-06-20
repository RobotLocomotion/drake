#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;
//#define PRINT_VAR(a) (void)a;
//#define PRINT_VARn(a) (void)a;

namespace drake {

using geometry::SceneGraph;
using multibody::benchmarks::inclined_plane::MakeInclinedPlanePlant;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::IntegratorBase;
using systems::Simulator;

namespace multibody {
namespace multibody_plant {
namespace {

class InclinedPlaneTest : public ::testing::TestWithParam<bool> {
 public:
  void SetUp() override {

  }
 protected:

};

// This test creates a simple multibody model of a sphere rolling down an
// inclined plane. After simulating the model for a given length of time, this
// test verifies the numerical solution against analytical results obtained from
// an energy conservation analysis.
TEST_P(InclinedPlaneTest, RollingSphereTest) {
  DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

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
  const bool time_stepping = GetParam();
  const double time_step = time_stepping ? 5.0e-4 : 0.0;
  const CoulombFriction<double> surface_friction(
      1.0 /* static friction */, 0.5 /* dynamic friction */);

  // Contact parameters. Results converge to the analytical solution as the
  // penetration allowance and the stiction tolerance go to zero.
  const double penetration_allowance = time_stepping ? 1.0e-6 : 1.0e-3;  // [m]
  // Stribeck approximation stiction velocity tolerance, [m/s].
  const double stiction_tolerance = time_stepping ? 1.0e-5 : 1.0e-3;

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeInclinedPlanePlant(
      radius, mass, slope, surface_friction, g, time_step, &scene_graph));
  const MultibodyTree<double>& model = plant.model();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(penetration_allowance);
  plant.set_stiction_tolerance(stiction_tolerance);
  const RigidBody<double>& ball = model.GetRigidBodyByName("Ball");

  // Hint the integrator's time step based on the contact time scale.
  // A fraction of this time scale is used which is chosen so that the fixed
  // time step integrators are stable.
  //const double max_time_step =
    //  plant.get_contact_penalty_method_time_scale() / 30;

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  // And build the Diagram:
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // This will set a default initial condition with the sphere located at
  // p_WBcm = (0; 0; 0) and zero spatial velocity.
  model.SetDefaultContext(&plant_context);

  Simulator<double> simulator(*diagram, std::move(diagram_context));
  IntegratorBase<double>* integrator = simulator.get_mutable_integrator();
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

  PRINT_VAR(ke_WB);
  PRINT_VAR(ke_WB_expected);
  PRINT_VAR(std::abs(ke_WB - ke_WB_expected) / ke_WB);

  PRINT_VAR(speed);
  PRINT_VAR(speed_expected);
  PRINT_VAR(std::abs(speed - speed_expected) / speed_expected);

  PRINT_VAR(angular_velocity);
  PRINT_VAR(angular_velocity_expected);
  PRINT_VAR(std::abs(angular_velocity - angular_velocity_expected)
                / angular_velocity_expected);
}

// Instantiate the tests.
INSTANTIATE_TEST_CASE_P(ContinuousAndTimeSteppingTest, InclinedPlaneTest,
                        ::testing::Bool());

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

