#include "drake/common/test_utilities/expect_no_throw.h"
#pragma once

#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/unused.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/pleides_system.h"

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

TYPED_TEST_SUITE_P(ExplicitErrorControlledIntegratorTest);

TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ReqInitialStepTarget) {
  // Set the requested initial step size.
  this->integrator->request_initial_step_size_target(this->kDt);
  EXPECT_EQ(this->integrator->get_initial_step_size_target(), this->kDt);
}

TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ContextAccess) {
  this->integrator->get_mutable_context()->SetTime(3.);
  EXPECT_EQ(this->integrator->get_context().get_time(), 3.);
  EXPECT_EQ(this->context->get_time(), 3.);
}

// Verifies error estimation is supported.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ErrorEstSupport) {
  EXPECT_GE(this->integrator->get_error_estimate_order(), 1);
  EXPECT_EQ(this->integrator->supports_error_estimation(), true);
  DRAKE_EXPECT_NO_THROW(this->integrator->set_target_accuracy(1e-1));
  DRAKE_EXPECT_NO_THROW(this->integrator->request_initial_step_size_target(
      this->kDt));
}

// Verifies that the stepping works with relatively small
// magnitude step sizes.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, MagDisparity) {
  this->context->SetTime(0.0);

  // Set integrator parameters.
  this->integrator->set_maximum_step_size(0.1);
  this->integrator->set_requested_minimum_step_size(1e-40);
  this->integrator->set_target_accuracy(1e-3);

  // Take all the defaults.
  this->integrator->Initialize();

  // Attempt to take a variable step- should not throw an exception.
  DRAKE_EXPECT_NO_THROW(
      this->integrator->IntegrateWithMultipleStepsToTime(1e-40));
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
  do {
    this->integrator->IntegrateNoFurtherThanTime(t_final, t_final, t_final);
  } while (this->context->get_time() < t_final);

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
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, ErrEstOrder) {
  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(this->kSpringK / this->kMass);

  // Pick a step size that is much smaller than the period of vibration.
  const double period_of_vibration = 2.0 * M_PI / omega;
  const double h = period_of_vibration / 512.0;

  // Set initial conditions.
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Set integrator parameters: do no error control.
  this->integrator->set_maximum_step_size(h);
  this->integrator->set_fixed_step_mode(true);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Take a single step of size h.
  ASSERT_EQ(this->context->get_time(), 0.0);
  const double t_final = this->context->get_time() + h;
  ASSERT_TRUE(this->integrator->IntegrateWithSingleFixedStepToTime(t_final));

  // Verify that a step of h was taken.
  EXPECT_NEAR(this->context->get_time(), h,
              std::numeric_limits<double>::epsilon());

  // Get the true solution.
  const double x_true = c1 * std::cos(omega * h) + c2 * std::sin(omega * h);

  // Get the integrator's solution.
  const double x_approx_h = this->context->get_continuous_state_vector().
      GetAtIndex(0);

  // Get the error estimate and the error in the error estimate.
  const double err_est_h =
      this->integrator->get_error_estimate()->get_vector().GetAtIndex(0);
  const double err_est_h_err = std::abs(err_est_h - (x_true - x_approx_h));

  // Compute the same solution using two half-steps.
  this->context->SetTime(0);
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
      initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
      initial_velocity);
  this->integrator->Initialize();
  ASSERT_TRUE(this->integrator->IntegrateWithSingleFixedStepToTime(
      t_final / 2.0));
  ASSERT_TRUE(this->integrator->IntegrateWithSingleFixedStepToTime(t_final));
  EXPECT_NEAR(this->context->get_time(), h,
              std::numeric_limits<double>::epsilon());
  const double x_approx_2h_h = this->context->get_continuous_state_vector().
      GetAtIndex(0);
  const double err_est_2h_h =
      this->integrator->get_error_estimate()->get_vector().GetAtIndex(0);
  const double err_est_2h_h_err = std::abs(err_est_2h_h -
      (x_true - x_approx_2h_h));

  // Verify that the error in the error estimate dropped in accordance with the
  // order of the error estimator. Theory indicates that asymptotic error in
  // the estimate is bound by K*h^order, where K is some constant and h is
  // sufficiently small. We assume a value for K of 4.0 below, and we check that
  // the improvement in the error estimate is not as good as K*h^(order+1).
  // The K and h might need to be redetermined for a different problem or
  // for untested error-controlled integrators.
  const double K = 4.0;
  const int err_est_order = this->integrator->get_error_estimate_order();
  EXPECT_LE(err_est_2h_h_err, K * err_est_h_err / std::pow(2.0, err_est_order));
  EXPECT_GE(K * err_est_2h_h_err,
      err_est_h_err / std::pow(2.0, err_est_order + 1));
}

// Integrate a purely continuous system with no sampling using error control.
// d^2x/h^2 = -kx/m
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
  for (double t = this->kDt; t <= t_final; t += this->kDt)
    this->integrator->IntegrateNoFurtherThanTime(t, t, t);

  // At this point, the time will often be 0.999 plus some change. Step one last
  // time to take us to 1.0s. If the time happens to already be 1.0s, this
  // call will have no effect.
  this->integrator->IntegrateNoFurtherThanTime(t_final, t_final, t_final);

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
  this->integrator->get_mutable_context()->SetTime(0.);
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Step for 1 second.
  do {
    this->integrator->IntegrateNoFurtherThanTime(t_final, t_final, t_final);
  } while (this->context->get_time() < t_final);

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

// Verifies that the integrator does not alter the state when directed to step
// to the present time.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, StepToCurrentTimeNoOp) {
  // Set integrator parameters: do error control.
  this->integrator->set_maximum_step_size(this->kDt);
  this->integrator->set_fixed_step_mode(false);

  // Initialize the integrator.
  this->integrator->Initialize();

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;

  // Set initial conditions.
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Integrate to one second.
  const double t_final = 1.0;
  while (this->context->get_time() < t_final)
    this->integrator->IntegrateNoFurtherThanTime(t_final, t_final, t_final);
  ASSERT_EQ(this->context->get_time(), t_final);

  // Get the final state.
  const VectorX<double> x_final =
      this->context->get_continuous_state_vector().CopyToVector();

  // Call the various stepping methods, ensuring that the state and time do
  // not change.
  const double inf = std::numeric_limits<double>::infinity();
  this->integrator->IntegrateWithMultipleStepsToTime(t_final);
  EXPECT_EQ(this->context->get_time(), t_final);
  for (int i = 0; i < x_final.size(); ++i)
    EXPECT_EQ(x_final[i], this->context->get_continuous_state_vector()[i]);
  this->integrator->IntegrateNoFurtherThanTime(inf, inf, t_final);
  EXPECT_EQ(this->context->get_time(), t_final);
  for (int i = 0; i < x_final.size(); ++i)
    EXPECT_EQ(x_final[i], this->context->get_continuous_state_vector()[i]);

  // Must do fixed stepping for the last test.
  this->integrator->set_fixed_step_mode(true);
  ASSERT_TRUE(this->integrator->IntegrateWithSingleFixedStepToTime(t_final));
  EXPECT_EQ(this->context->get_time(), t_final);
  for (int i = 0; i < x_final.size(); ++i)
    EXPECT_EQ(x_final[i], this->context->get_continuous_state_vector()[i]);
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
  do {
    // NOTE: this perfect storm of conditions (error controlled integration
    // can take the maximum step size, publish time larger than update time,
    // update time larger than directed step, directed step larger than maximum
    // step size) causes IntegratorBase::StepOnceErrorControlledAtMost() to
    // to hang *if* that method does not account for the maximum step size.
    this->integrator->IntegrateNoFurtherThanTime(
        inf, this->context->get_time() + max_step_size + eps, t_final);
  } while (this->context->get_time() < t_final);

  // Verify the statistics.
  EXPECT_LE(this->integrator->get_largest_step_size_taken(),
            max_step_size * this->integrator->get_stretch_factor());
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

  ASSERT_EQ(this->context->get_time(), 0.0);
  EXPECT_THROW(unused(
      this->integrator->IntegrateWithSingleFixedStepToTime(1e-8)),
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
  const double t_final = this->context->get_time() + this->kDt;
  this->integrator->IntegrateNoFurtherThanTime(t_final, t_final, t_final);

  // Verify that integrator statistics are valid.
  EXPECT_GE(this->integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_LE(this->integrator->get_previous_integration_step_size(),
            this->kDt);
  EXPECT_LE(this->integrator->get_smallest_adapted_step_size_taken(),
            this->kDt);
}

// Verifies that dense integration works with error-controlled integration.
TYPED_TEST_P(ExplicitErrorControlledIntegratorTest, DenseOutput) {
  this->integrator->set_target_accuracy(1e-8);

  // Set an initial time step target that is too large, so that we have step
  // size "shrinkages".
  this->integrator->request_initial_step_size_target(3.);
  this->integrator->set_maximum_step_size(10.);

  // Initialize the integrator.
  this->integrator->Initialize();
  this->integrator->StartDenseIntegration();

  // Set the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  this->spring_mass->set_position(this->integrator->get_mutable_context(),
                             initial_position);
  this->spring_mass->set_velocity(this->integrator->get_mutable_context(),
                             initial_velocity);

  // Integrate and confirm that we had to shrink the steps.
  this->integrator->IntegrateWithMultipleStepsToTime(1.);
  EXPECT_GE(this->integrator->get_num_step_shrinkages_from_error_control(), 0);

  const std::unique_ptr<trajectories::PiecewisePolynomial<double>>
      dense_output = this->integrator->StopDenseIntegration();

  // Check the dense output.
  EXPECT_EQ(dense_output->start_time(), 0.0);
  EXPECT_EQ(dense_output->end_time(), 1.0);
  EXPECT_EQ(dense_output->get_number_of_segments(),
            this->integrator->get_num_steps_taken());
}

REGISTER_TYPED_TEST_SUITE_P(ExplicitErrorControlledIntegratorTest,
    ReqInitialStepTarget, ContextAccess, ErrorEstSupport, MagDisparity, Scaling,
    BulletProofSetup, ErrEstOrder, SpringMassStepEC, MaxStepSizeRespected,
    IllegalFixedStep, CheckStat, DenseOutput, StepToCurrentTimeNoOp);

// T is the integrator type (e.g., RungeKutta3Integrator<double>).
template <class T>
struct PleidesTest : public ::testing::Test {
 public:
  PleidesTest() {
    // Create the Pleides system.
    pleides = std::make_unique<analysis::test::PleidesSystem>();
    context = pleides->CreateDefaultContext();

    // Create the integrator.
    integrator = std::make_unique<T>(*pleides, context.get());
  }

  std::unique_ptr<analysis::test::PleidesSystem> pleides;
  std::unique_ptr<Context<double>> context;
  std::unique_ptr<IntegratorBase<double>> integrator;
};

TYPED_TEST_SUITE_P(PleidesTest);

// Verifies that the Pleides system can be integrated accurately.
TYPED_TEST_P(PleidesTest, Pleides) {
  // Set integrator to use variable-step (not fixed-step) with a tight accuracy
  // requirement for each variable step. Due to step size cutting, a variable
  // step can be substantially smaller than the initial step size. By default,
  // the initial step size is 1/10 of maximum step size (chosen below). We
  // request semi-tight accuracy, allowing us to use this test for various
  // error controlled integrators.
  this->integrator->set_maximum_step_size(0.1);
  this->integrator->set_fixed_step_mode(false);
  const double requested_local_accuracy = 1e-7;
  this->integrator->set_target_accuracy(requested_local_accuracy);

  // kTolerance = 100 is a heuristic derived from simulation experiments and
  // based on the fact that all tests pass within a tolerance of 25. The
  // extra factor of 4 (2 bits) helps ensure the tests also pass on
  // various compilers and computer architectures.
  const double kTolerance = 100 * requested_local_accuracy;

  // Initialize the integrator.
  this->integrator->Initialize();

  // Simulate to the designated time.
  const double t_final = this->pleides->get_end_time();
  this->integrator->IntegrateWithMultipleStepsToTime(t_final);

  // Check the result.
  const VectorX<double> q = this->context->get_continuous_state().
    get_generalized_position().CopyToVector();
  const VectorX<double> q_des = analysis::test::PleidesSystem::GetSolution(
        this->context->get_time());
  for (int i = 0; i < q.size(); ++i)
    EXPECT_NEAR(q[i], q_des[i], kTolerance) << i;
}

REGISTER_TYPED_TEST_SUITE_P(PleidesTest, Pleides);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
