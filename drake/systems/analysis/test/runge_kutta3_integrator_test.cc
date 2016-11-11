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

    // Create and initialize the integrator.
    integrator_ = std::make_unique<RungeKutta3Integrator<double>>(
        *spring_mass_, context_.get());  // Use default Context.
    integrator_->Initialize();
  }

  std::unique_ptr<analysis_test::MySpringMassSystem<double>> spring_mass_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<RungeKutta3Integrator<double>> integrator_;
  const double DT = 1e-3;        // Integration step size.
  const double kSpring = 300.0;  // N/m
  const double kMass = 2.0;      // kg
};

TEST_F(RK3IntegratorTest, ReqAccuracy) {
  // Set the accuracy.
  integrator_->request_initial_step_size_target(DT);

  EXPECT_EQ(integrator_->get_initial_step_size_target(), DT);
}

TEST_F(RK3IntegratorTest, ContextAccess) {
  integrator_->get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator_->get_context().get_time(), 3.);
  EXPECT_EQ(context_->get_time(), 3.);

  // Reset the context time.
  integrator_->get_mutable_context()->set_time(0.);
}

/// Verifies error estimation is supported.
TEST_F(RK3IntegratorTest, ErrorEst) {
  EXPECT_GE(integrator_->get_error_estimate_order(), 1);
  EXPECT_EQ(integrator_->supports_error_estimation(), true);
  EXPECT_NO_THROW(integrator_->set_target_accuracy(1e-1));
  EXPECT_NO_THROW(integrator_->request_initial_step_size_target(DT));
}

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(RK3IntegratorTest, SpringMassStep) {
  // Set integrator parameters: do no error control.
  integrator_->set_maximum_step_size(DT);
  integrator_->set_minimum_step_size(DT);
  integrator_->set_target_accuracy(1.0);

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             kInitialPosition);

  // Setup c1 and c2 for ODE constants.
  const double kC1 = kInitialPosition;
  const double kC2 = kInitialVelocity / kOmega;

  // StepOnceAtFixedSize for 1 second.
  const double kTFinal = 1.0;
  for (double t = 0.0; t < kTFinal; t += DT)
    integrator_->StepOnceAtMost(DT, DT);

  // Get the final position.
  const double kXFinal =
      context_->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(
      kC1 * std::cos(kOmega * kTFinal) + kC2 * std::sin(kOmega * kTFinal),
      kXFinal, 1e-5);
}

// Test scaling vectors
TEST_F(RK3IntegratorTest, Scaling) {
  // Test scaling
  EXPECT_EQ(integrator_->get_mutable_generalized_state_weight_vector().size(),
            1);
  EXPECT_EQ(integrator_->get_mutable_generalized_state_weight_vector()
                .lpNorm<Eigen::Infinity>(),
            1);
  EXPECT_EQ(integrator_->get_misc_state_weight_vector().size(), 1);
  EXPECT_EQ(integrator_->get_mutable_misc_state_weight_vector()
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

  // Reset the integrator context
  integrator_->reset_context(context_.get());

  // Set reasonable integrator parameters.
  integrator_->set_maximum_step_size(0.1);
  integrator_->set_minimum_step_size(1e-6);
  integrator_->set_target_accuracy(1e-5);

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             kInitialPosition);

  // Setup c1 and c2 for ODE constants.
  const double kC1 = kInitialPosition;
  const double kC2 = kInitialVelocity / kOmega;

  // StepOnceAtFixedSize for 1 second.
  const double kTFinal = 1.0;
  double t_remaining = kTFinal - context->get_time();
  do {
    integrator_->StepOnceAtMost(t_remaining, t_remaining);
    t_remaining = kTFinal - context->get_time();
  } while (t_remaining > 0.0);

  // Get the final position.
  const double kXFinal =
      context->get_state().get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(
      kC1 * std::cos(kOmega * kTFinal) + kC2 * std::sin(kOmega * kTFinal),
      kXFinal, 1e-5);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator_->get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator_->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator_->get_smallest_adapted_step_size_taken(), 0.0);
  EXPECT_GE(integrator_->get_num_steps_taken(), 0);
  EXPECT_NE(integrator_->get_error_estimate(), nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
