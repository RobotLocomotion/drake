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
  }

  std::unique_ptr<analysis_test::MySpringMassSystem<double>> spring_mass_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<RungeKutta3Integrator<double>> integrator_;
  const double DT = 1e-3;        // Integration step size.
  const double kBigDT = 1e-1;    // Big integration step size.
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

// Test scaling vectors
TEST_F(RK3IntegratorTest, Scaling) {
  // Initialize the integrator to set weight vector sizes.
  integrator_->Initialize();

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

// Tests the error estimation capabilities.
TEST_F(RK3IntegratorTest, ErrEst) {
  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             kInitialPosition);
  spring_mass_->set_velocity(integrator_->get_mutable_context(),
                             kInitialVelocity);

  // Setup c1 and c2 for ODE constants.
  const double kC1 = kInitialPosition;
  const double kC2 = kInitialVelocity / kOmega;

  // Set integrator parameters: do no error control.
  integrator_->set_maximum_step_size(kBigDT);
  integrator_->set_minimum_step_size(kBigDT);
  integrator_->set_target_accuracy(10.0);

  // Initialize the integrator.
  integrator_->Initialize();

  // Take a single step of size DT.
  integrator_->StepOnceAtMost(kBigDT, kBigDT);

  // Verify that a step of DT was taken.
  EXPECT_NEAR(context_->get_time(), kBigDT,
              std::numeric_limits<double>::epsilon());

  // Get the true solution
  const double kXTrue =  kC1 * std::cos(kOmega * kBigDT) +
      kC2 * std::sin(kOmega * kBigDT);

  // Get the integrator's solution
  const double kXApprox =
      context_->get_state().get_continuous_state()->get_vector().GetAtIndex(0);

  // Get the error estimate
  const double err_est = integrator_->get_error_estimate()->
      get_vector().GetAtIndex(0);

  // Verify that difference between integration result and true result is
  // captured by the error estimate. The 0.2 below indicates that the error
  // estimate is quite conservative.
  EXPECT_NEAR(kXApprox, kXTrue, err_est*0.2);
}

// Integrate a purely continuous system with no sampling using error control.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(RK3IntegratorTest, SpringMassStepEC) {

  // Set integrator parameters: do no error control.
  integrator_->set_maximum_step_size(DT);
  integrator_->set_minimum_step_size(DT);
  integrator_->set_target_accuracy(1.0);

  // Initialize the integrator.
  integrator_->Initialize();

  // Setup the initial position and initial velocity
  const double kInitialPosition = 0.1;
  const double kInitialVelocity = 0.0;
  const double kOmega = std::sqrt(kSpring / kMass);

  // Set initial condition using the Simulator's internal Context.
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             kInitialPosition);
  spring_mass_->set_velocity(integrator_->get_mutable_context(),
                           kInitialVelocity);

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

  // Store the number of integration steps
  int fixed_steps = integrator_->get_num_steps_taken();

  // Check the solution.
  EXPECT_NEAR(
      kC1 * std::cos(kOmega * kTFinal) + kC2 * std::sin(kOmega * kTFinal),
      kXFinal, 1e-5);

  // Reset the integrator and set reasonable parameters for integration with
  // error control.
  integrator_->Reset();
  integrator_->set_maximum_step_size(0.1);
  integrator_->set_minimum_step_size(1e-6);
  integrator_->set_target_accuracy(1e-3);

  // Re-initialize the integrator.
  integrator_->Initialize();

  // Set initial condition using the Simulator's internal Context.
  integrator_->get_mutable_context()->set_time(0.);
  spring_mass_->set_position(integrator_->get_mutable_context(),
                             kInitialPosition);
  spring_mass_->set_velocity(integrator_->get_mutable_context(),
                             kInitialVelocity);

  // StepOnceAtFixedSize for 1 second.
  double t_remaining = kTFinal - context_->get_time();
  do {
    integrator_->StepOnceAtMost(t_remaining, t_remaining);
    t_remaining = kTFinal - context_->get_time();
  } while (t_remaining > 0.0);

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

  // Verify that less computation was performed compared to the fixed step
  // integrator.
  EXPECT_LT(integrator_->get_num_steps_taken(), fixed_steps);
}

}  // namespace
}  // namespace systems
}  // namespace drake
