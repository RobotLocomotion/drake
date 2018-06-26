#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
//#define PRINT_VAR(a) (void) a;

namespace drake {

using geometry::Box;
using geometry::SceneGraph;
using multibody::parsing::AddModelFromSdfFile;
using systems::AbstractValue;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::IntegratorBase;
using systems::Simulator;

namespace multibody {
namespace multibody_plant {
namespace {

// This test creates a simple multibody model of a sphere rolling down an
// inclined plane. After simulating the model for a given length of time, this
// test verifies the numerical solution against analytical results obtained from
// an energy conservation analysis.
GTEST_TEST(Box, UnderStiction) {
  DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Length of the simulation, in seconds.
  const double simulation_time = 2.0;

  // Plant's parameters.
  //const double length = 1.0;   // Box size, [m]
  const double mass = 1.0;     // Box mass, [kg]
  const double g = 10.0;       // Acceleration of gravity, [m/s²]
  const CoulombFriction<double> surface_friction(
      1.0 /* static friction */, 1.0 /* dynamic friction */);

  // Simulation parameters.
  const double time_step = 1.0e-3;  // in seconds.
  const double penetration_allowance = 1.0e-4;  // in meters.
  const double stiction_tolerance = 1.0e-4;  // in meters per second.
  const double applied_force = 5.0;  // Force in Newtons.

  // Tolerance used to verify the results.
  const double kTolerance = 1.0e-12;

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/multibody_tree/multibody_plant/test/box.sdf");
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  AddModelFromSdfFile(full_name, &plant, &scene_graph);

  // Add collision geometry to the ground.
  plant.RegisterCollisionGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-y plane.
      geometry::HalfSpace::MakePose(
          Vector3<double>::UnitZ(), Vector3<double>::Zero()),
      geometry::HalfSpace(), surface_friction, &scene_graph);

  // Add gravity to the model.
  plant.AddForceElement<UniformGravityFieldElement>(
      -g * Vector3<double>::UnitZ());

  plant.Finalize(&scene_graph);  // Done creating the model.

#if 0
  // Add a slider to grant the box one DOF along the x axis.
  const PrismaticJoint<double>& x_slider = plant.AddJoint<PrismaticJoint>(
      "SliderInX", plant.world_body(), {}, box, {}, Vector3<double>::UnitX());
  // We add actuation along the x axis so that we can apply an external force.
  plant.AddJointActuator("ActuationInX", x_slider);

  // Add a slider to grant the box one DOF along the z axis. This allows the box
  // to "fall" in the z direction generating contact forces against the ground.
  const PrismaticJoint<double>& z_slider = plant.AddJoint<PrismaticJoint>(
      "SliderInZ", plant.world_body(), {}, box, {}, Vector3<double>::UnitX());
#endif

  const MultibodyTree<double>& model = plant.model();
  // Set contact parameters.
  plant.set_penetration_allowance(penetration_allowance);
  plant.set_stiction_tolerance(stiction_tolerance);

  // Sanity check for the model's size.
  EXPECT_EQ(plant.num_velocities(), 2);
  EXPECT_EQ(plant.num_positions(), 2);
  EXPECT_EQ(plant.num_actuators(), 1);
  EXPECT_EQ(plant.num_actuated_dofs(), 1);

  // Two input ports: actuation + geometric queries.
  EXPECT_EQ(plant.get_num_input_ports(), 2);

  // Sanity check on the availability of the optional source id before using it.
  ASSERT_TRUE(!!plant.get_source_id());

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

  plant_context.FixInputPort(0, Vector1<double>::Constant(applied_force));

  Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // We evaluate the contact results output port in order to verify the
  // simulation results.
  std::unique_ptr<AbstractValue> contact_results_value =
      plant.get_contact_results_output_port().Allocate();
  EXPECT_NO_THROW(
      contact_results_value->GetValueOrThrow<ContactResults<double>>());
  const ContactResults<double>& contact_results =
      contact_results_value->GetValueOrThrow<ContactResults<double>>();
  // Compute the poses for each geometry in the model.
  plant.get_contact_results_output_port().Calc(
      plant_context, contact_results_value.get());

  PRINT_VAR(contact_results.num_contacts());

  const PointPairContactInfo<double>& contact_info =
      contact_results.contact_info(0);

  const RigidBody<double>& box = model.GetRigidBodyByName("box");
  EXPECT_TRUE(
      (contact_info.bodyA_index() == box.index() &&
       contact_info.bodyB_index() == plant.world_body().index()) ||
      (contact_info.bodyB_index() == box.index() &&
       contact_info.bodyA_index() == plant.world_body().index()));

  // direction = +1 indicates the normal is the +z axis pointing into the box
  // (i.e. the outward normal to the ground plane).
  // direction = -1 indicates the outward normal to the box (i.e. pointing down
  // in the -z dirction).
  double direction = contact_info.bodyA_index() == box.index() ? 1.0 : -1.0;

  // The expected value of the contact force applied on body with index
  // contact_info.bodyB_index() at the contact point C.
  const Vector3<double> f_Bc_W(-applied_force, 0.0, mass * g * direction);
  EXPECT_TRUE(CompareMatrices(
      contact_info.contact_force(), f_Bc_W,
      kTolerance, MatrixCompareType::relative));

  // Upper limit on the x displacement computed using the maximum possible
  // velocity under stiction, i.e. the stiction tolerance.
  const double x_upper_limit = stiction_tolerance * simulation_time;
  EXPECT_LT


  PRINT_VAR(contact_info.bodyA_index());
  PRINT_VAR(contact_info.bodyB_index());
  PRINT_VAR(contact_info.contact_force().transpose());
  PRINT_VAR(contact_info.contact_point().transpose());

  PRINT_VAR(contact_info.point_pair().depth);
  PRINT_VAR(contact_info.point_pair().nhat_BA_W.transpose());

  const SpatialVelocity<double>& V_WB =
      model.EvalBodySpatialVelocityInWorld(plant_context, box);
  PRINT_VAR(V_WB);
  const Isometry3<double>& X_WB =
      model.EvalBodyPoseInWorld(plant_context, box);
  PRINT_VAR(X_WB.matrix());

#if 0
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

  // Verify the relative errors. For the continuous model of the plant errors
  // are dominated by the penetration allowance and the stiction tolerance since
  // the integrator's accuracy was set to a relatively tight value.
  // For the time stepping model of the plant, errors are dominated by the
  // finite time step, given that we are able to use very tight penetration
  // allowance and stiction tolerance.
  // Notice that given the kinematic relationship between linear and angular
  // velocity v_WBcm = radius * w_WB at rolling, relative errors
  // in v_WBcm and w_WB have the same order of magnitude. Moreover, since the
  // kinetic energy scales with the velocities (translational and angular)
  // squared, standard error propagation shows that the relative error in the
  // kinetic energy is expected to be twice that in the velocities. Thus the
  // factor of two used below for the relative error in kinetic energy.
  EXPECT_TRUE(
      std::abs(ke_WB - ke_WB_expected) / ke_WB < 2 * relative_tolerance_);
  EXPECT_TRUE(
      std::abs(speed - speed_expected) / speed_expected < relative_tolerance_);
  EXPECT_TRUE(std::abs(angular_velocity - angular_velocity_expected)
                  / angular_velocity_expected < relative_tolerance_);
#endif
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

