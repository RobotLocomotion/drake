#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {

using systems::Context;
using systems::Simulator;

namespace multibody {
namespace multibody_plant {
namespace {

// This test creates a simple multibody model of a sphere rolling down an
// inclined plane. After simulating the model for a given length of time, this
// test verifies the numerical solution against analytical results obtained from
// an energy conservation analysis.
GTEST_TEST(JointLimitsTest, PrismaticJoint) {
  const double time_step = 1.0e-3;

  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double mass = 1.0;      // Mass of the body, [kg]
  const double box_size = 0.3;  // The size of the box shaped body, [m].

  MultibodyPlant<double> plant(time_step);
  const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
          mass, Vector3<double>::Zero(),
          mass * UnitInertia<double>::SolidBox(box_size, box_size, box_size));
  const RigidBody<double>& body = plant.AddRigidBody("Body", M_B);
  const PrismaticJoint<double>& slider = plant.AddJoint<PrismaticJoint>(
      "Slider", plant.world_body(), {}, body, {}, Vector3<double>::UnitZ(),
      0.0 /* damping */, 0.0 /* lower limit */, 0.1 /* upper limit */);
  plant.AddJointActuator("ForceAlongZ", slider);
  plant.Finalize();

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 1);
  DRAKE_DEMAND(plant.num_positions() == 1);

  Simulator<double> simulator(plant);
  Context<double>& context = simulator.get_mutable_context();
  context.FixInputPort(0, Vector1<double>::Constant(-10.0));
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // For this test we expect the steady state to be within 1 mm of the limit.
  EXPECT_NEAR(slider.get_translation(context), slider.lower_limit(), 1.1e-3);

  // After a second of simulation we expect the slider to be at rest.
  EXPECT_NEAR(slider.get_translation_rate(context), 0.0, 1.0e-12);

  // Set the force to be positive and re-start the simulation.
  context.FixInputPort(0, Vector1<double>::Constant(10.0));
  context.set_time(0.0);
  simulator.StepTo(simulation_time);

  // Verify we are at rest near the upper limit.
  EXPECT_NEAR(slider.get_translation(context), slider.upper_limit(), 1.1e-3);
  EXPECT_NEAR(slider.get_translation_rate(context), 0.0, 1.0e-12);
}

GTEST_TEST(JointLimitsTest, RevoluteJoint) {
  const double time_step = 1.0e-3;

  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double mass = 1.0;         // Mass of the rod, [kg]
  const double rod_length = 0.3;   // The length of the rod, [m].
  const double rod_radius = 0.01;  // The radius of the rod, [m].

  MultibodyPlant<double> plant(time_step);
  // The COM of the rod is right at its center, though we place the body frame B
  // on the left end of the rod to connect it to the world with a revolute
  // joint.
  const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
      mass, Vector3<double>(rod_length / 2.0, 0.0, 0.0),
      mass * UnitInertia<double>::SolidCylinder(
          rod_radius, rod_length, Vector3<double>::UnitX()));
  const RigidBody<double>& body = plant.AddRigidBody("Body", M_B);
  const RevoluteJoint<double>& pin = plant.AddJoint<RevoluteJoint>(
      "Pin", plant.world_body(), {}, body, {}, Vector3<double>::UnitZ(),
      0.0 /* damping */,
      -M_PI / 5.0 /* lower limit */, M_PI / 3.0 /* upper limit */);
  plant.AddJointActuator("TorqueAboutZ", pin);
  plant.Finalize();

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 1);
  DRAKE_DEMAND(plant.num_positions() == 1);

  Simulator<double> simulator(plant);
  Context<double>& context = simulator.get_mutable_context();
  context.FixInputPort(0, Vector1<double>::Constant(1.5));
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // For this test we expect the steady state to be within 5e-3 radians of the
  // limit.
  EXPECT_NEAR(pin.get_angle(context), pin.upper_limit(), 5.1e-3);
  // After a second of simulation we expect the pint to be at rest.
  EXPECT_NEAR(pin.get_angular_rate(context), 0.0, 1.0e-12);

  // Set the torque to be negative and re-start the simulation.
  context.FixInputPort(0, Vector1<double>::Constant(-1.5));
  context.set_time(0.0);
  simulator.StepTo(simulation_time);

  // Verify we are at rest near the lower limit.
  EXPECT_NEAR(pin.get_angle(context), pin.lower_limit(), 5.1e-3);
  EXPECT_NEAR(pin.get_angular_rate(context), 0.0, 1.0e-12);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

