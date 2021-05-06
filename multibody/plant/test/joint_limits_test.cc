#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;
using systems::Simulator;
using multibody::Parser;

namespace multibody {
namespace {

const char kIiwaFilePath[] =
    "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";

// These unit tests verify the convergence of the joint limits as the time step
// is decreased. A constant force is applied at the joint to drive its state to
// the upper/lower limit.
// MultibodyPlant uses a characteristic "numerical stiffness" time
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
    plant.mutable_gravity_field().set_gravity_vector(
        Vector3<double>::Zero());
    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        mass, Vector3<double>::Zero(),
        mass * UnitInertia<double>::SolidBox(box_size, box_size, box_size));
    const RigidBody<double>& body = plant.AddRigidBody("Body", M_B);
    const PrismaticJoint<double>& slider = plant.AddJoint<PrismaticJoint>(
        "Slider", plant.world_body(), std::nullopt, body, std::nullopt,
        Vector3<double>::UnitZ(), 0.0 /* lower limit */, 0.1 /* upper limit */,
        0.0 /* damping */);
    plant.AddJointActuator("ForceAlongZ", slider);
    plant.Finalize();

    // Sanity check for the model's size.
    DRAKE_DEMAND(plant.num_velocities() == 1);
    DRAKE_DEMAND(plant.num_positions() == 1);

    Simulator<double> simulator(plant);
    Context<double>& context = simulator.get_mutable_context();
    plant.get_actuation_input_port().FixValue(&context, -10.0);
    simulator.Initialize();
    simulator.AdvanceTo(simulation_time);

    // We expect a second order convergence with the time step. That is, we
    // expect the error to be lower than:
    const double expected_error = 1100 * time_step * time_step;
    // where the constant 1100 is simply obtained through previous runs of this
    // test. See description of this test at the top.

    EXPECT_NEAR(slider.get_translation(context), slider.position_lower_limit(),
                expected_error);
    // After a second of simulation we expect the slider to be at rest.
    EXPECT_NEAR(slider.get_translation_rate(context), 0.0, kVelocityTolerance);

    // Set the force to be positive and re-start the simulation.
    plant.get_actuation_input_port().FixValue(&context, 10.0);
    context.SetTime(0.0);
    simulator.Initialize();
    simulator.AdvanceTo(simulation_time);

    // Verify we are at rest near the upper limit.
    EXPECT_NEAR(slider.get_translation(context), slider.position_upper_limit(),
                expected_error);
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
    // The COM of the rod is right at its center, though we place the body frame
    // B on the left end of the rod to connect it to the world with a revolute
    // joint.
    const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
        mass, Vector3<double>(rod_length / 2.0, 0.0, 0.0),
        mass * UnitInertia<double>::SolidCylinder(
            rod_radius, rod_length, Vector3<double>::UnitX()));
    const RigidBody<double>& body = plant.AddRigidBody("Body", M_B);
    const RevoluteJoint<double>& pin = plant.AddJoint<RevoluteJoint>(
        "Pin", plant.world_body(), std::nullopt, body, std::nullopt,
        Vector3<double>::UnitZ(), -M_PI / 5.0 /* lower limit */,
        M_PI / 3.0 /* upper limit */, 0.0 /* damping */);
    plant.AddJointActuator("TorqueAboutZ", pin);
    plant.Finalize();

    // Sanity check for the model's size.
    DRAKE_DEMAND(plant.num_velocities() == 1);
    DRAKE_DEMAND(plant.num_positions() == 1);

    Simulator<double> simulator(plant);
    Context<double>& context = simulator.get_mutable_context();
    plant.get_actuation_input_port().FixValue(&context, 1.5);
    simulator.Initialize();
    simulator.AdvanceTo(simulation_time);

    // We expect a second order convergence with the time step. That is, we
    // expect the error to be lower than:
    const double expected_error = 5100 * time_step * time_step;
    // where the constant 5100 is simply obtained through previous runs of this
    // test. See description of this test at the top.

    EXPECT_NEAR(pin.get_angle(context), pin.position_upper_limit(),
                expected_error);
    // After a second of simulation we expect the pint to be at rest.
    EXPECT_NEAR(pin.get_angular_rate(context), 0.0, kVelocityTolerance);

    // Set the torque to be negative and re-start the simulation.
    plant.get_actuation_input_port().FixValue(&context, -1.5);
    context.SetTime(0.0);
    simulator.Initialize();
    simulator.AdvanceTo(simulation_time);

    // Verify we are at rest near the lower limit.
    EXPECT_NEAR(pin.get_angle(context), pin.position_lower_limit(),
                expected_error);
    EXPECT_NEAR(pin.get_angular_rate(context), 0.0, kVelocityTolerance);
  }
}

VectorX<double> KukaPositionLowerLimits() {
  VectorX<double> lower_limits(7);
  lower_limits
      << -2.96706, -2.0944, -2.96706, -2.0944, -2.96706, -2.0944, -3.05433;
  return lower_limits;
}

VectorX<double> KukaPositionUpperLimits() {
  return -KukaPositionLowerLimits();
}

// We test joint limits for the case of a Kuka arm model. In order to reach
// the joint limits we drive the joints by applying a constant torque for a
// given length of simulation time.
// Tolerances are rather loose given we use a relatively large time step and
// a short simulation time to keep wall clock times low, specially for debug
// builds.
GTEST_TEST(JointLimitsTest, KukaArm) {
  const double time_step = 2.0e-3;
  const double simulation_time = 35;

  // At steady state after one second of simulation, we expect the velocity to
  // be zero within this absolute tolerance.
  const double kVelocityTolerance = 5.0e-4;

  // Expected relative tolerance for the joint limits. This number is chosen
  // so that the verifications performed below pass for the time step used in
  // this test. A smaller time step would lead to smaller violations (with
  // quadratic convergence in the time step) and therefore we could make
  // kRelativePositionTolerance even smaller. However there is a trade off
  // between what we want to test and the computational cost of this unit test.
  const double kRelativePositionTolerance = 0.055;

  MultibodyPlant<double> plant(time_step);
  Parser(&plant).AddModelFromFile(FindResourceOrThrow(kIiwaFilePath));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("iiwa_link_0"));
  plant.mutable_gravity_field().set_gravity_vector(
      Vector3<double>::Zero());
  plant.Finalize();

  // Some sanity check on model sizes.
  const int nq = plant.num_positions();
  ASSERT_EQ(plant.num_positions(), nq);
  ASSERT_EQ(plant.num_velocities(), nq);
  ASSERT_EQ(plant.num_actuators(), nq);
  ASSERT_EQ(plant.num_actuated_dofs(), nq);

  // Verify the joint limits were correctly parsed.
  const VectorX<double> lower_limits_expected = KukaPositionLowerLimits();
  const VectorX<double> upper_limits_expected = KukaPositionUpperLimits();
  EXPECT_TRUE(CompareMatrices(lower_limits_expected,
                              plant.GetPositionLowerLimits()));
  EXPECT_TRUE(CompareMatrices(upper_limits_expected,
                              plant.GetPositionUpperLimits()));

  // Duplicate checks for `GetPosition*Limits`, but using joint names.
  for (int joint_number = 1; joint_number <= nq; ++joint_number) {
    const std::string joint_name = "iiwa_joint_" + std::to_string(joint_number);
    const auto& joint = plant.GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_NEAR(joint.position_lower_limit(),
                lower_limits_expected(joint_number - 1),
                std::numeric_limits<double>::epsilon());
    EXPECT_NEAR(joint.position_upper_limit(),
                upper_limits_expected(joint_number - 1),
                std::numeric_limits<double>::epsilon());
  }

  Simulator<double> simulator(plant);
  Context<double>& context = simulator.get_mutable_context();

  // Drive all the joints to their upper limit by applying a positive torque.
  plant.get_actuation_input_port().FixValue(&context,
                                            VectorX<double>::Constant(nq, 0.4));
  simulator.Initialize();
  simulator.AdvanceTo(simulation_time);

  for (int joint_number = 1; joint_number <= nq; ++joint_number) {
    const std::string joint_name = "iiwa_joint_" + std::to_string(joint_number);
    const auto& joint = plant.GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_LT(
        std::abs((joint.get_angle(context) - joint.position_upper_limit()) /
                 joint.position_upper_limit()),
        kRelativePositionTolerance);
    EXPECT_NEAR(joint.get_angular_rate(context), 0.0, kVelocityTolerance);
  }

  // Drive all the joints to their lower limit by applying a negative torque.
  plant.get_actuation_input_port().FixValue(
      &context, VectorX<double>::Constant(nq, -0.4));
  plant.SetDefaultContext(&context);
  context.SetTime(0.0);
  simulator.Initialize();
  simulator.AdvanceTo(simulation_time);
  for (int joint_number = 1; joint_number <= nq; ++joint_number) {
    const std::string joint_name = "iiwa_joint_" + std::to_string(joint_number);
    const auto& joint = plant.GetJointByName<RevoluteJoint>(joint_name);
    EXPECT_LT(
        std::abs((joint.get_angle(context) - joint.position_lower_limit()) /
                 joint.position_lower_limit()),
        kRelativePositionTolerance);
    EXPECT_NEAR(joint.get_angular_rate(context), 0.0, kVelocityTolerance);
  }
}

GTEST_TEST(JointLimitsTest, KukaArmFloating) {
  // Check limits for a model with a floating base.
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(FindResourceOrThrow(kIiwaFilePath));
  plant.Finalize();
  const int nq = 14;
  const int nq_floating = 7;
  const int nq_arm = 7;
  EXPECT_EQ(plant.num_positions(), nq);
  const double inf = std::numeric_limits<double>::infinity();
  VectorX<double> lower_limits_expected(nq);
  lower_limits_expected.head(nq_floating).setConstant(-inf);
  lower_limits_expected.tail(nq_arm) = KukaPositionLowerLimits();
  EXPECT_TRUE(CompareMatrices(lower_limits_expected,
                              plant.GetPositionLowerLimits()));
  VectorX<double> upper_limits_expected(nq);
  upper_limits_expected.head(nq_floating).setConstant(inf);
  upper_limits_expected.tail(nq_arm) = KukaPositionUpperLimits();
  EXPECT_TRUE(CompareMatrices(upper_limits_expected,
                              plant.GetPositionUpperLimits()));
}

// Invokes a method that putatively uses joint limits, but which are ignored
// for a continuous plant. This is merely to confirm that nothing crashes.
GTEST_TEST(JointLimitsTest, ContinuousLimitsDoNotFault) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(FindResourceOrThrow(kIiwaFilePath));
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  plant.get_actuation_input_port().FixValue(context.get(),
      Eigen::VectorXd::Zero(7));

  // The reaction forces putatively use joint limits, and will log a warning.
  plant.get_reaction_forces_output_port().Eval<AbstractValue>(*context);

  // A second attempt also does not crash, and will not log a warning.
  context->DisableCaching();
  plant.get_reaction_forces_output_port().Eval<AbstractValue>(*context);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
