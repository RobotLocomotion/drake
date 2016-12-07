#include "drake/systems/analysis/explicit_euler_integrator.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(IntegratorTest, MiscAPI) {
  typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AScalar;
  SpringMassSystem<double> spring_mass_dbl(1., 1., 0.);
  SpringMassSystem<AScalar> spring_mass_ad(1., 1., 0.);

  // Setup the integration step size.
  const double dt = 1e-3;

  // Create a context.
  auto context_dbl = spring_mass_dbl.CreateDefaultContext();
  auto context_ad = spring_mass_ad.CreateDefaultContext();

  // Create the integrator as a double and as an autodiff type
  ExplicitEulerIntegrator<double> int_dbl(spring_mass_dbl, dt,
                                          context_dbl.get());
  ExplicitEulerIntegrator<AScalar> int_ad(spring_mass_ad, dt, context_ad.get());

  // Test that setting the target accuracy or initial step size target fails.
  EXPECT_THROW(int_dbl.set_target_accuracy(1.0), std::logic_error);
  EXPECT_THROW(int_dbl.request_initial_step_size_target(1.0), std::logic_error);
}

GTEST_TEST(IntegratorTest, ContextAccess) {
  // Create the mass spring system.
  SpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup the integration step size.
  const double dt = 1e-3;

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  ExplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);\
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  EXPECT_THROW(integrator.StepOnceAtMost(dt, dt), std::logic_error);
}

/// Verifies error estimation is unsupported.
GTEST_TEST(IntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  const double dt = 1e-3;
  auto context = spring_mass.CreateDefaultContext();
  ExplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());

  EXPECT_EQ(integrator.get_error_estimate_order(), 0);
  EXPECT_EQ(integrator.supports_error_estimation(), false);
  EXPECT_THROW(integrator.set_target_accuracy(1e-1), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(dt),
               std::logic_error);
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
  ExplicitEulerIntegrator<double> integrator(
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
    integrator.StepOnceAtMost(inf, inf);

  EXPECT_NEAR(context->get_time(), t, dt);  // Should be exact.

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(c1 * std::cos(kOmega * t) + c2 * std::sin(kOmega * t), x_final,
              5e-3);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

GTEST_TEST(IntegratorTest, StepSize) {
  // Create the mass spring system.
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  // Set the maximum step size.
  const double max_dt = .01;
  // Create a context.
  auto context = spring_mass.CreateDefaultContext();
  context->set_time(0.0);
  double t = 0.0;
  // Create the integrator.
  ExplicitEulerIntegrator<double> integrator(
      spring_mass, max_dt, context.get());
  integrator.Initialize();

  // The step ends on the next publish time.
  {
    const double publish_dt = 0.005;
    const double update_dt = 0.007;
    typename IntegratorBase<double>::StepResult result =
        integrator.StepOnceAtMost(publish_dt, update_dt);
    EXPECT_EQ(IntegratorBase<double>::kReachedPublishTime, result);
    EXPECT_EQ(publish_dt, context->get_time());
    t = context->get_time();
  }

  // The step ends on the next update time.
  {
    const double publish_dt = 0.0013;
    const double update_dt = 0.0011;
    typename IntegratorBase<double>::StepResult result =
        integrator.StepOnceAtMost(publish_dt, update_dt);
    EXPECT_EQ(IntegratorBase<double>::kReachedUpdateTime, result);
    EXPECT_EQ(t + update_dt, context->get_time());
    t = context->get_time();
  }

  // The step ends on the max step time, because both the publish and update
  // times are too far in the future.
  {
    const double publish_dt = 0.17;
    const double update_dt = 0.19;
    typename IntegratorBase<double>::StepResult result =
        integrator.StepOnceAtMost(publish_dt, update_dt);
    EXPECT_EQ(IntegratorBase<double>::kTimeHasAdvanced, result);
    EXPECT_EQ(t + max_dt, context->get_time());
    t = context->get_time();
  }

  // The step ends on the next update time, even though it's a little larger
  // than the max step time, because the max step time stretches.
  {
    const double publish_dt = 42.0;
    const double update_dt = 0.01001;
    typename IntegratorBase<double>::StepResult result =
        integrator.StepOnceAtMost(publish_dt, update_dt);
    EXPECT_EQ(IntegratorBase<double>::kReachedUpdateTime, result);
    EXPECT_EQ(t + update_dt, context->get_time());
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
