#include "drake/systems/analysis/semi_explicit_euler_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

// Tests context-related operations, including determining whether or not
// system can be integrated without a context (it can't).
GTEST_TEST(IntegratorTest, ContextAccess) {
  // Create the mass spring system.
  SpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup the integration step size.
  const double dt = 1e-3;

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  SemiExplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);\
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  EXPECT_THROW(integrator.StepOnceAtMost(dt, dt, dt), std::logic_error);
}

// Verifies error estimation is unsupported.
GTEST_TEST(IntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  const double dt = 1e-3;
  auto context = spring_mass.CreateDefaultContext();
  SemiExplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());

  EXPECT_EQ(integrator.get_error_estimate_order(), 0);
  EXPECT_EQ(integrator.supports_error_estimation(), false);
  EXPECT_THROW(integrator.set_target_accuracy(1e-1), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(dt),
               std::logic_error);
}

// Tests accuracy when generalized velocity is not the time derivative of
// generalized configuration (using a rigid body).
GTEST_TEST(IntegratorTest, RigidBody) {
  // Instantiates a Multibody Dynamics (MBD) model of the world.
  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Add a single free body with a quaternion base.
  RigidBody<double>* body;
  tree->add_rigid_body(
      std::unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
  body->set_name("free_body");

  // Sets body to have a non-zero spatial inertia. Otherwise the body gets
  // welded by a fixed joint to the world by RigidBodyTree::compile().
  body->set_mass(1.0);
  body->set_spatial_inertia(Matrix6<double>::Identity());
  body->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
      "base", Eigen::Isometry3d::Identity()));

  tree->compile();

  // Instantiates a RigidBodyPlant from the  MBD model.
  RigidBodyPlant<double> plant(move(tree));
  auto context = plant.CreateDefaultContext();

  // Set free_body to have zero translation, zero rotation, and zero velocity.
  plant.SetDefaultState(*context, context->get_mutable_state());

  Eigen::Vector3d v0(1, 2, 3);    // Linear velocity in body's frame.
  Eigen::Vector3d w0(-4, 5, 6);  // Angular velocity in body's frame.
  BasicVector<double> generalized_velocities(plant.get_num_velocities());

  // NOTE: the functionality of this code is dependent upon the rigid body
  // velocity variables being ordering as [ angular, linear ].
  generalized_velocities.get_mutable_value() << w0, v0;

  // Set the linear and angular velocity.
  for (int i=0; i< plant.get_num_velocities(); ++i)
    plant.set_velocity(context.get(), i, generalized_velocities[i]);

  // Integrate for one second of virtual time using explicit Euler integrator.
  const double small_dt = 1e-4;
  ExplicitEulerIntegrator<double> ee(plant, small_dt, context.get());
  ee.Initialize();
  const double t_final = 1.0;
  double t_remaining = t_final - context->get_time();
  do {
    ee.StepOnceAtMost(t_remaining, t_remaining, t_remaining);
    t_remaining = t_final - context->get_time();
  } while (t_remaining > 0.0);

  // Get the final state.
  VectorX<double> x_final_ee = context->get_continuous_state_vector().
      CopyToVector();

  // Re-integrate with semi-explicit Euler.
  context->set_time(0.);
  plant.SetDefaultState(*context, context->get_mutable_state());
  for (int i=0; i< plant.get_num_velocities(); ++i)
    plant.set_velocity(context.get(), i, generalized_velocities[i]);
  SemiExplicitEulerIntegrator<double> see(plant, small_dt, context.get());
  see.Initialize();

  // Integrate for one second.
  t_remaining = t_final - context->get_time();
  do {
    see.StepOnceAtMost(t_remaining, t_remaining, t_remaining);
    t_remaining = t_final - context->get_time();
  } while (t_remaining > 0.0);

  // Verify that the final states are "close".
  VectorX<double> x_final_see = context->get_continuous_state_vector().
      CopyToVector();
  const double tol = 5e-3;
  for (int i=0; i< x_final_see.size(); ++i)
    EXPECT_NEAR(x_final_ee[i], x_final_see[i], tol);
}

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
GTEST_TEST(IntegratorTest, SpringMassStep) {
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg

  // Create the spring-mass system.
  SpringMassSystem<double> spring_mass(kSpring, kMass, 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Setup the integration size and infinity.
  const double dt = 1e-6;
  const double inf = std::numeric_limits<double>::infinity();

  // Create the integrator.
  SemiExplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  // Setup the initial position and initial velocity.
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.01;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition.
  spring_mass.set_position(context.get(), kInitialPosition);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double c1 = kInitialPosition;
  const double c2 = kInitialVelocity / kOmega;

  // Integrate for 1 second.
  const double kTFinal = 1.0;
  double t;
  for (t = 0.0; std::abs(t - kTFinal) > dt; t += dt)
    integrator.StepOnceAtMost(inf, inf, dt);

  EXPECT_NEAR(context->get_time(), t, dt);  // Should be exact.

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(c1 * std::cos(kOmega * t) + c2 * std::sin(kOmega * t), x_final,
              5e-3);

  // Verify that integrator statistics are valid
  EXPECT_GT(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GT(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GT(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
