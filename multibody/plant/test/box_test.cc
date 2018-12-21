#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::Box;
using geometry::SceneGraph;
using multibody::Parser;
using systems::AbstractValue;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::IntegratorBase;
using systems::Simulator;

namespace multibody {
namespace {

// This unit test loads a model for a box on a flat plane. The plane is the x-y
// plane with the z axis pointing up and gravity pointing down in the negative z
// direction.
// In this test we apply an external horizontal force for which the box is in
// stiction.
// The test then verifies the computed values of contact results.
GTEST_TEST(Box, UnderStiction) {
  DiagramBuilder<double> builder;

  // Length of the simulation, in seconds.
  const double simulation_time = 2.0;

  // Plant's parameters.
  const double mass = 1.0;     // Box mass, [kg]
  // We use g = 10 m/s² so that numbers are simpler.
  const double g = 10.0;       // Acceleration of gravity, [m/s²]
  const CoulombFriction<double> surface_friction(
      0.9 /* static friction */, 0.9 /* dynamic friction */);

  // Simulation parameters.
  const double time_step = 1.0e-3;  // in seconds.
  const double penetration_allowance = 1.0e-4;  // in meters.
  const double stiction_tolerance = 1.0e-4;  // in meters per second.
  const double applied_force = 5.0;  // Force in Newtons.

  // Tolerance used to verify the results.
  // Since the contact solver uses a continuous ODE to model Coulomb friction
  // (a modified Stribeck model), we simulate for a long enough time to reach a
  // "steady state". Therefore the precision of the results in these tests is
  // dominated for "how well we reached steady state".
  const double kTolerance = 1.0e-12;

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/plant/test/box.sdf");
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(time_step));
  Parser(&plant).AddModelFromFile(full_name);

  // Add gravity to the model.
  plant.AddForceElement<UniformGravityFieldElement>(
      -g * Vector3<double>::UnitZ());

  plant.Finalize();  // Done creating the model.

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

  // And build the Diagram:
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  plant_context.FixInputPort(
      plant.get_actuation_input_port().get_index(),
      Vector1<double>::Constant(applied_force));

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

  ASSERT_EQ(contact_results.num_contacts(), 1);  // only one contact pair.
  const PointPairContactInfo<double>& contact_info =
      contact_results.contact_info(0);

  // Verify the bodies referenced by the contact info.
  const Body<double>& ground = plant.GetBodyByName("ground");
  const Body<double>& box = plant.GetBodyByName("box");
  EXPECT_TRUE(
      (contact_info.bodyA_index() == box.index() &&
       contact_info.bodyB_index() == ground.index()) ||
      (contact_info.bodyB_index() == box.index() &&
       contact_info.bodyA_index() == ground.index()));

  // Whether the normal points up or down depends on the order in which the
  // geometry engine orders bodies in the contact pair.
  // direction = +1 indicates the normal is the +z axis pointing into the box
  // (i.e. the outward normal to the ground plane).
  // direction = -1 indicates the outward normal to the box (i.e. pointing down
  // in the -z direction).
  double direction = contact_info.bodyA_index() == box.index() ? 1.0 : -1.0;

  // The expected value of the contact force applied on the box at the contact
  // point C.
  const Vector3<double> f_Bc_W(-applied_force, 0.0, mass * g);
  EXPECT_TRUE(CompareMatrices(
      contact_info.contact_force(), f_Bc_W * direction,
      kTolerance, MatrixCompareType::relative));

  // Upper limit on the x displacement computed using the maximum possible
  // velocity under stiction, i.e. the stiction tolerance.
  const double x_upper_limit = stiction_tolerance * simulation_time;
  EXPECT_LT(contact_info.contact_point().x(), x_upper_limit);

  // The penetration allowance is just an estimate. Therefore we only expect
  // the penetration depth to be close enough to it.
  EXPECT_NEAR(contact_info.point_pair().depth, penetration_allowance, 1.0e-3);

  // Whether the normal points up or down depends on the order in which
  // scene graph orders bodies in the contact pair.
  const Vector3<double> expected_normal = Vector3<double>::UnitZ() * direction;
  EXPECT_TRUE(CompareMatrices(
      contact_info.point_pair().nhat_BA_W, expected_normal,
      kTolerance, MatrixCompareType::relative));

  // If we are in stiction, the slip speed should be smaller than the specified
  // stiction tolerance.
  EXPECT_LT(contact_info.slip_speed(), stiction_tolerance);

  // There should not be motion in the normal direction.
  EXPECT_NEAR(contact_info.separation_speed(), 0.0, kTolerance);
}

}  // namespace
}  // namespace multibody
}  // namespace drake

