#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;
using systems::Simulator;

namespace multibody {
namespace multibody_plant {
namespace {

// These unit tests verify the convergence of the joint limits as the time step
// is decreased. MultibodyPlant uses a characteristic "numerical stiffness" time
// scale proportional to the time step of the discrete model. This time scale is
// used to estimate the largest stiffness penalty parameter used to enforce
// joint limits that still guarantees the stability of the time stepping
// approach. This stiffness parameter is shown to be proportional to the inverse
// of the time step squared, i.e. k ∝ 1/δt².
// Since, at steady state, the violation of the joint limit is inversely
// proportional to the stiffness parameter, this violation turns out being
// proportional to the time step squared, that is, Δq ∝ δt².
// Therefore the convergence of the joint limit violation is expected to be
// quadratic with the time step. These unit tests verifies this.
// In addition, the joint velocities are expected to go to zero within a time
// interval proportional to this numerical stiffness time scale. This is also
// verified.
// Moreover, these unit tests validate the stability of the method as the time
// step is decreased, verifying that the solution is stable even though the
// joint stiffness increases as we decrease the time step.


// This unit test verifies the convergence of joint limits for a prismatic
// joint. Refer to the documentation at the top of this file for further
// details.
// The test consists of a box shaped body of a given mass attached to the world
// by a prismatic joint granting a single degree of freedom to move along the
// z axis. An actuator is added to this joint so that we can apply a force
// that pushes the body against the joint's limits. We then verify the joint
// limits are satisfied within the expected precision for the given time step.
GTEST_TEST(JointLimitsTest, PrismaticJointConvergenceTest) {
  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double mass = 1.0;      // Mass of the body, [kg]
  const double box_size = 0.3;  // The size of the box shaped body, [m].

  // At steady state after one second of simulation, we expect the velocity to
  // be zero within this absolute tolerance.
  const double kVelocityTolerance = 1.0e-12;  // in m/s.

  for (double time_step : {2.5e-4, 5.0e-4, 1.0e-3}) {
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

    // We expect a second order convergence with the time step. That is, we
    // expect the error to be lower than:
    const double expected_error = 1100 * time_step * time_step;
    // where the constant 1100 is simply obtained through previous runs of this
    // test. See description of this test at the top.

    EXPECT_NEAR(
        slider.get_translation(context), slider.lower_limit(), expected_error);
    // After a second of simulation we expect the slider to be at rest.
    EXPECT_NEAR(slider.get_translation_rate(context), 0.0, kVelocityTolerance);

    // Set the force to be positive and re-start the simulation.
    context.FixInputPort(0, Vector1<double>::Constant(10.0));
    context.set_time(0.0);
    simulator.StepTo(simulation_time);

    // Verify we are at rest near the upper limit.
    EXPECT_NEAR(
        slider.get_translation(context), slider.upper_limit(), expected_error);
    EXPECT_NEAR(slider.get_translation_rate(context), 0.0, kVelocityTolerance);
  }
}

// This unit test verifies the convergence of joint limits for a revolute
// joint. Refer to the documentation at the top of this file for further
// details.
// The test consists of a rod body of a given mass attached to the world
// by a revolute joint at one of the rod's ends, granting a single degree of
// freedom to rotate about the z axis. An actuator is added to this joint so
// that we can apply a torque that pushes the rod against the joint's limits.
// We then verify the joint limits are satisfied within the expected precision
// for the given time step.
GTEST_TEST(JointLimitsTest, RevoluteJoint) {
  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double mass = 1.0;         // Mass of the rod, [kg]
  const double rod_length = 0.3;   // The length of the rod, [m].
  const double rod_radius = 0.01;  // The radius of the rod, [m].

  // At steady state after one second of simulation, we expect the velocity to
  // be zero within this absolute tolerance.
  const double kVelocityTolerance = 1.0e-12;  // in rad/s.

  for (double time_step : {2.5e-4, 5.0e-4, 1.0e-3}) {
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

    // We expect a second order convergence with the time step. That is, we
    // expect the error to be lower than:
    const double expected_error = 5100 * time_step * time_step;
    // where the constant 5100 is simply obtained through previous runs of this
    // test. See description of this test at the top.

    EXPECT_NEAR(pin.get_angle(context), pin.upper_limit(), expected_error);
    // After a second of simulation we expect the pint to be at rest.
    EXPECT_NEAR(pin.get_angular_rate(context), 0.0, kVelocityTolerance);

    // Set the torque to be negative and re-start the simulation.
    context.FixInputPort(0, Vector1<double>::Constant(-1.5));
    context.set_time(0.0);
    simulator.StepTo(simulation_time);

    // Verify we are at rest near the lower limit.
    EXPECT_NEAR(pin.get_angle(context), pin.lower_limit(), expected_error);
    EXPECT_NEAR(pin.get_angular_rate(context), 0.0, kVelocityTolerance);
  }
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

