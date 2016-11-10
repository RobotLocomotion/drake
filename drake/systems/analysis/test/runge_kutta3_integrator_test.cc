#include "drake/systems/analysis/runge_kutta3_integrator.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

class RK3IntegratorTest : public ::testing::Test {
 public:
  RK3IntegratorTest() {
    // Create a mass-spring-system with update rate=0.
    spring_mass_ = std::make_unique<analysis_test::MySpringMassSystem<double>>(
        kSpring, kMass, 0.);
    context_ = spring_mass_->CreateDefaultContext();
  }

  std::unique_ptr<analysis_test::MySpringMassSystem<double>> spring_mass_;
  std::unique_ptr<Context<double>> context_;
  const double DT = 1e-3;         // Integration step size.
  const double kSpring = 300.0;   // N/m
  const double kMass = 2.0;       // kg
};

TEST_F(RK3IntegratorTest, MiscAPI) {

  // Create the integrator.
  RungeKutta3Integrator<double> integrator(*spring_mass_, context_.get());

  // Set the accuracy.
  integrator.request_initial_step_size_target(DT);

  EXPECT_EQ(integrator.get_initial_step_size_target(), DT);
}

TEST_F(RK3IntegratorTest, ContextAccess) {
  // Create the integrator
  RungeKutta3Integrator<double> integrator(
      *spring_mass_, context_.get());  // Use default Context.

  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context_->get_time(), 3.);

  // Reset the context time.
  integrator.get_mutable_context()->set_time(0.);
}

/// Verifies error estimation is supported.
TEST_F(RK3IntegratorTest, ErrorEst) {
  // Spring-mass system is necessary only to setup the problem.
  RungeKutta3Integrator<double> integrator(*spring_mass_, context_.get());

  EXPECT_GE(integrator.get_error_estimate_order(), 1);
  EXPECT_EQ(integrator.supports_error_estimation(), true);
  EXPECT_NO_THROW(integrator.set_target_accuracy(1e-1));
  EXPECT_NO_THROW(integrator.request_initial_step_size_target(DT));
}

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(RK3IntegratorTest, SpringMassStep) {
  // Create the integrator.
  RungeKutta3Integrator<double> integrator(
      *spring_mass_, context_.get());  // Use default Context.

  // Set integrator parameters: do no error control.
  integrator.set_maximum_step_size(DT);
  integrator.set_minimum_step_size(DT);
  integrator.set_target_accuracy(1.0);

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator.get_mutable_context(), kInitialPosition);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double C1 = kInitialPosition;
  const double C2 = kInitialVelocity / kOmega;

  // StepOnceAtFixedSize for 1 second.
  const double T_FINAL = 1.0;
  for (double t = 0.0; t < T_FINAL; t += DT) integrator.StepOnceAtMost(DT, DT);

  // Get the final position.
  const double kXFinal =
      context_->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(C1 * std::cos(kOmega * T_FINAL) + C2 * std::sin(kOmega * T_FINAL),
              kXFinal, 1e-5);
}

// Test scaling vectors
TEST_F(RK3IntegratorTest, Scaling) {
  // Create and initialize the integrator.
  RungeKutta3Integrator<double> integrator(
      *spring_mass_, context_.get());  // Use default Context.
  integrator.Initialize();

  // Test scaling
  EXPECT_EQ(integrator.get_mutable_generalized_state_weight_vector().size(), 1);
  EXPECT_EQ(integrator.get_mutable_generalized_state_weight_vector()
                .lpNorm<Eigen::Infinity>(),
            1);
  EXPECT_EQ(integrator.get_misc_state_weight_vector().size(), 1);
  EXPECT_EQ(integrator.get_mutable_misc_state_weight_vector()
                .lpNorm<Eigen::Infinity>(),
            1);
}

// Integrate a purely continuous system with no sampling using error control.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(RK3IntegratorTest, SpringMassStepEC) {
  // Create a new context
  auto context = spring_mass_->CreateDefaultContext();

  // Create the integrator.
  RungeKutta3Integrator<double> integrator(
      *spring_mass_, context.get());  // Use default Context.

  // Set reasonable integrator parameters.
  integrator.set_maximum_step_size(0.1);
  integrator.set_minimum_step_size(1e-6);
  integrator.set_target_accuracy(1e-5);

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator.get_mutable_context(), kInitialPosition);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double C1 = kInitialPosition;
  const double C2 = kInitialVelocity / kOmega;

  // StepOnceAtFixedSize for 1 second.
  const double T_FINAL = 1.0;
  double t_remaining = T_FINAL - context->get_time();
  do {
    integrator.StepOnceAtMost(t_remaining, t_remaining);
    t_remaining = T_FINAL - context->get_time();
  } while (t_remaining > 0.0);

  // Get the final position.
  const double kXFinal =
      context->get_state().get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(C1 * std::cos(kOmega * T_FINAL) + C2 * std::sin(kOmega * T_FINAL),
              kXFinal, 1e-5);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_smallest_adapted_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_NE(integrator.get_error_estimate(), nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
