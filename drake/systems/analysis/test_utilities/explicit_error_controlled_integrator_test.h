#pragma once

#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// T is the integrator type (e.g., RungeKutta3Integrator<double>).
template <class T>
struct ExplicitErrorControlledIntegratorTest : public ::testing::Test {
 public:
  ExplicitErrorControlledIntegratorTest() {
    // Create a mass-spring-system with update rate=0.
    spring_mass = std::make_unique<analysis_test::MySpringMassSystem<double>>(
        kSpringK, kMass, 0.);
    context = spring_mass->CreateDefaultContext();

    // Create and initialize the integrator.
    integrator = std::make_unique<T>(*spring_mass, context.get());
  }

  std::unique_ptr<analysis_test::MySpringMassSystem<double>> spring_mass;
  std::unique_ptr<Context<double>> context;
  std::unique_ptr<IntegratorBase<double>> integrator;
  const double kDt = 1e-3;         // Integration step size.
  const double kBigDt = 1e-1;     // Big integration step size.
  const double kSpringK = 300.0;  // N/m
  const double kMass = 2.0;        // kg
};

TYPED_TEST_CASE_P(ExplicitErrorControlledIntegratorTest);

TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ReqInitialStepTarget) {
  // Set the requested initial step size.
  this->integrator->request_initial_step_size_target(this->kDt);
  EXPECT_EQ(this->integrator->get_initial_step_size_target(), this->kDt);
}

TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ContextAccess) {
  this->integrator->get_mutable_context()->set_time(3.);
  EXPECT_EQ(this->integrator->get_context().get_time(), 3.);
  EXPECT_EQ(this->context->get_time(), 3.);
}

// Verifies error estimation is supported.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ErrorEstSupport) {
  EXPECT_GE(this->integrator->get_error_estimate_order(), 1);
  EXPECT_EQ(this->integrator->supports_error_estimation(), true);
  EXPECT_NO_THROW(this->integrator->set_target_accuracy(1e-1));
  EXPECT_NO_THROW(this->integrator->request_initial_step_size_target(
      this->kDt));
}

// Verifies that the stepping works with relatively small
// magnitude step sizes.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, MagDisparity) {
  // Set a unit magnitude time.
  this->context->set_time(1.0);

  // Set integrator parameters.
  this->integrator->set_maximum_step_size(0.1);
  this->integrator->set_requested_minimum_step_size(1e-40);
  this->integrator->set_target_accuracy(1e-3);

  // Take all the defaults.
  this->integrator->Initialize();

  // Attempt to take a variable step- should not throw an exception.
  EXPECT_NO_THROW(this->integrator->IntegrateWithMultipleSteps(1e-40));
}

// Test scaling vectors
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, Scaling) {
  // Setting maximum integrator step size is necessary to prevent integrator
  // from throwing an exception.
  this->integrator->set_maximum_step_size(this->kBigDt);

  // Initialize the integrator to set weight vector sizes.
  this->integrator->Initialize();

  // Test scaling
  EXPECT_EQ(this->integrator->get_mutable_generalized_state_weight_vector().
            size(), 1);
  EXPECT_EQ(this->integrator->get_mutable_generalized_state_weight_vector().
            template lpNorm<Eigen::Infinity>(), 1);
  EXPECT_EQ(this->integrator->get_misc_state_weight_vector().size(), 1);
  EXPECT_EQ(this->integrator->get_mutable_misc_state_weight_vector().
            template lpNorm<Eigen::Infinity>(), 1);
}

// Tests the ability to setup the integrator robustly (i.e., with minimal
// user knowledge); in other words, if the user fails to set some aspect of the
// integrator properly, will NaN values make it run forever?
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, BulletProofSetup) {
  // Setup the initial position and initial velocity
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(this->kSpringK / this->kMass);

  // Set the initial conditions.
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Attempt to initialize the integrator: should throw logic error because
  // neither maximum step size nor target accuracy has been set.
  EXPECT_THROW(this->integrator->Initialize(), std::logic_error);

  // Attempt to initialize the integrator: should throw logic error because
  // maximum step size smaller than minimum step size.
  this->integrator->set_maximum_step_size(this->kDt);
  this->integrator->set_requested_minimum_step_size(this->kBigDt);
  EXPECT_THROW(this->integrator->Initialize(), std::logic_error);

  // Set step sizes to cogent values and try to initialize again but now using
  // bad requested initial step sizes.
  this->integrator->set_requested_minimum_step_size(1e-8);
  this->integrator->set_maximum_step_size(this->kBigDt);
  this->integrator->request_initial_step_size_target(1e-10);
  EXPECT_THROW(this->integrator->Initialize(), std::logic_error);
  this->integrator->request_initial_step_size_target(this->kBigDt*2.0);

  // Set the accuracy to something too loose, set the maximum step size and
  // try again. Integrator should now silently adjust the target accuracy to
  // the in-use accuracy.
  this->integrator->request_initial_step_size_target(this->kDt);
  this->integrator->set_target_accuracy(10.0);
  this->integrator->Initialize();
  EXPECT_LE(this->integrator->get_accuracy_in_use(),
            this->integrator->get_target_accuracy());

  // Integrate for 1 second using variable stepping.
  const double t_final = 1.0;
  double t_remaining = t_final - this->context->get_time();
  do {
    this->integrator->IntegrateAtMost(t_remaining, t_remaining, t_remaining);
    t_remaining = t_final - this->context->get_time();
  } while (t_remaining > 0.0);

  // Get the final position.
  const double x_final =
      this->context->get_continuous_state().get_vector().GetAtIndex(0);

  // Check the solution. We're not really looking for accuracy here, just
  // want to make sure that the value is finite.
  EXPECT_NEAR(
      c1 * std::cos(omega * t_final) + c2 * std::sin(omega * t_final),
      x_final, 1e0);
}

// Tests the error estimation capabilities.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ErrEst) {
  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(this->kSpringK / this->kMass);

  // Set initial conditions.
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Set integrator parameters: do no error control.
  this->integrator->set_maximum_step_size(this->kBigDt);
  this->integrator->set_fixed_step_mode(true);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Take a single step of size this->kBigDt.
  this->integrator->IntegrateAtMost(this->kBigDt, this->kBigDt,
                                     this->kBigDt);

  // Verify that a step of this->kBigDt was taken.
  EXPECT_NEAR(this->context->get_time(), this->kBigDt,
              std::numeric_limits<double>::epsilon());

  // Get the true solution.
  const double x_true = c1 * std::cos(omega * this->kBigDt) +
      c2 * std::sin(omega * this->kBigDt);

  // Get the integrator's solution.
  const double kXApprox = this->context->get_continuous_state_vector().
      GetAtIndex(0);

  // Get the error estimate.
  const double err_est =
      this->integrator->get_error_estimate()->get_vector().GetAtIndex(0);

  // Verify that difference between integration result and true result is
  // captured by the error estimate. The 0.2 below indicates that the error
  // estimate is quite conservative.
  EXPECT_NEAR(kXApprox, x_true, err_est * 0.2);
}

// Integrate a purely continuous system with no sampling using error control.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, SpringMassStepEC) {
  // Set integrator parameters: do no error control.
  this->integrator->set_maximum_step_size(this->kDt);
  this->integrator->set_fixed_step_mode(true);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(this->kSpringK / this->kMass);

  // Set initial conditions.
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Step for 1 second.
  const double t_final = 1.0;
  for (double t = 0.0; t < t_final; t += this->kDt)
    this->integrator->IntegrateAtMost(this->kDt, this->kDt, this->kDt);

  // Get the final position.
  const double x_final =
      this->context->get_continuous_state().get_vector().GetAtIndex(0);

  // Store the number of integration steps.
  int fixed_steps = this->integrator->get_num_steps_taken();

  // Check the solution.
  EXPECT_NEAR(
      c1 * std::cos(omega * t_final) + c2 * std::sin(omega * t_final),
      x_final, 1e-5);

  // Reset the integrator and set reasonable parameters for integration with
  // error control.
  this->integrator->Reset();
  this->integrator->set_maximum_step_size(0.1);
  this->integrator->set_requested_minimum_step_size(1e-6);
  this->integrator->set_target_accuracy(1e-3);

  // Re-initialize the integrator.
  this->integrator->Initialize();

  // Set initial conditions.
  this->integrator->get_mutable_context()->set_time(0.);
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Step for 1 second.
  double t_remaining = t_final - this->context->get_time();
  do {
    this->integrator->IntegrateAtMost(t_remaining, t_remaining, t_remaining);
    t_remaining = t_final - this->context->get_time();
  } while (t_remaining > 0.0);

  // Check the solution.
  EXPECT_NEAR(
      c1 * std::cos(omega * t_final) + c2 * std::sin(omega * t_final),
      x_final, 1e-5);

  // Verify that integrator statistics are valid.
  EXPECT_GE(this->integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_GE(this->integrator->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(this->integrator->get_smallest_adapted_step_size_taken(), 0.0);
  EXPECT_GE(this->integrator->get_num_steps_taken(), 0);
  EXPECT_NE(this->integrator->get_error_estimate(), nullptr);
  EXPECT_GT(this->integrator->get_num_derivative_evaluations(), 0);

  // Verify that less computation was performed compared to the fixed step
  // integrator.
  EXPECT_LT(this->integrator->get_num_steps_taken(), fixed_steps);
}

// Verifies that the maximum step size taken is smaller than the integrator
// max.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, MaxStepSizeRespected) {
  // Set the initial position and initial velocity such that any step is
  // viable.
  const double initial_position = 0;
  const double initial_velocity = 0;
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Set reasonable parameters for integration with error control.
  const double max_step_size = 1e-2;
  this->integrator->Reset();
  this->integrator->set_maximum_step_size(max_step_size);
  this->integrator->set_requested_minimum_step_size(1e-6);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Step for 1/10 second.
  const double inf = std::numeric_limits<double>::infinity();
  const double eps = std::numeric_limits<double>::epsilon();
  const double t_final = 0.1;
  double t_remaining = t_final - this->context->get_time();
  do {
    // NOTE: this perfect storm of conditions (error controlled integration
    // can take the maximum step size, publish time larger than update time,
    // update time larger than directed step, directed step larger than maximum
    // step size) causes IntegratorBase::StepOnceErrorControlledAtMost() to
    // to hang *if* that method does not account for the maximum step size.
    this->integrator->IntegrateAtMost(inf, max_step_size + eps, t_remaining);
    t_remaining = t_final - this->context->get_time();
  } while (t_remaining > 0.0);

  // Verify the statistics.
  EXPECT_LE(this->integrator->get_largest_step_size_taken(),
            max_step_size * this->integrator->get_stretch_factor());
}

// Verify that attempting to take a step for a very large initial time throws
// an exception.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, MinTimeThrows) {
  // Set integrator parameters: do error control.
  const double dt = 1e-1;
  this->integrator->set_maximum_step_size(dt);
  this->integrator->set_fixed_step_mode(false);
  this->integrator->set_target_accuracy(1e-14);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Set the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Set the time to a really large value in the context.
  const double large_time = 1e20;
  this->integrator->get_mutable_context()->set_time(large_time);

  // It should throw if we try to integrate forward.
  EXPECT_THROW(this->integrator->IntegrateWithMultipleSteps(dt),
               std::runtime_error);

  // Set the requested minimum step size and try to integrate again; should
  // still throw an exception.
  this->integrator->set_maximum_step_size(dt * 10);
  this->integrator->get_mutable_context()->set_time(0);
  this->integrator->set_requested_minimum_step_size(1e-2);
  EXPECT_THROW(this->integrator->IntegrateWithMultipleSteps(dt),
               std::runtime_error);

  // Disable the throw and verify that the exception does not still occur.
  this->integrator->set_throw_on_minimum_step_size_violation(false);
  EXPECT_NO_THROW(this->integrator->IntegrateWithMultipleSteps(dt));
}

// Verify that attempting to take a single fixed step throws an exception.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, IllegalFixedStep) {
  // Set integrator parameters: do error control.
  this->integrator->set_maximum_step_size(this->kDt);
  this->integrator->set_fixed_step_mode(false);

  // Set accuracy to a really small value so that the step is guaranteed to be
  // small.
  this->integrator->set_target_accuracy(1e-8);

  // Initialize the integrator.
  this->integrator->Initialize();

  EXPECT_THROW(this->integrator->IntegrateWithSingleFixedStep(1e-8),
               std::logic_error);
}

// Verifies statistics validity for error controlled integrator.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, CheckStat) {
  // Set integrator parameters: do error control.
  this->integrator->set_maximum_step_size(this->kDt);
  this->integrator->set_fixed_step_mode(false);

  // Set accuracy to a really small value so that the step is guaranteed to be
  // small.
  this->integrator->set_target_accuracy(1e-8);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Set the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Integrate just one step.
  this->integrator->IntegrateAtMost(this->kDt, this->kDt, this->kDt);

  // Verify that integrator statistics are valid.
  EXPECT_GE(this->integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_LE(this->integrator->get_previous_integration_step_size(),
            this->kDt);
  EXPECT_LE(this->integrator->get_smallest_adapted_step_size_taken(),
            this->kDt);
}

REGISTER_TYPED_TEST_CASE_P(ExplicitErrorControlledIntegratorTest,
    ReqInitialStepTarget, ContextAccess, ErrorEstSupport, MagDisparity, Scaling,
    BulletProofSetup, ErrEst, SpringMassStepEC, MaxStepSizeRespected,
    MinTimeThrows, IllegalFixedStep, CheckStat);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
