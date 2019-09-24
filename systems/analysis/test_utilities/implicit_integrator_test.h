#pragma once

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/test_utilities/discontinuous_spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"
#include "drake/systems/analysis/test_utilities/spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"
#include "drake/systems/analysis/test_utilities/stiff_double_mass_spring_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

enum ReuseType { kNoReuse, kReuse };

template <typename T>
class ImplicitIntegratorTest : public ::testing::Test {
 public:
  ImplicitIntegratorTest() {
    // Create the spring-mass systems.
    spring_ = std::make_unique<SpringMassSystem<double>>(
        spring_k_, mass_, false /* no forcing */);

    spring_damper_ = std::make_unique<
        implicit_integrator_test::SpringMassDamperSystem<double>>(
        stiff_spring_k_, stiff_damping_b_, mass_);
    mod_spring_damper_ = std::make_unique<
        implicit_integrator_test::DiscontinuousSpringMassDamperSystem<double>>(
        stiff_spring_k_, damping_b_, mass_, constant_force_mag_);
    stiff_double_system_ =
        std::make_unique<analysis::test::StiffDoubleMassSpringSystem<double>>();

    // One context will be usable for three of the systems.
    context_ = spring_->CreateDefaultContext();

    // Separate context necessary for the double spring mass system.
    dspring_context_ = stiff_double_system_->CreateDefaultContext();
  }

  void MiscAPITest(ReuseType type) {
    // Create the integrator for a System<double>.
    T integrator(*spring_, context_.get());

    // Verifies set_reuse(flag) == get_reuse() == flag
    integrator.set_reuse(reuse_type_to_bool(type));
    EXPECT_EQ(integrator.get_reuse(), reuse_type_to_bool(type));

    // Verifies that calling Initialize without setting step size target or
    // maximum step size throws exception.
    EXPECT_THROW(integrator.Initialize(), std::logic_error);

    // Verify defaults match documentation.
    EXPECT_EQ(integrator.get_jacobian_computation_scheme(),
              T::JacobianComputationScheme::kForwardDifference);

    // Test that setting the target accuracy and initial step size target is
    // successful.
    integrator.set_maximum_step_size(dt_);
    integrator.set_target_accuracy(1.0);
    integrator.request_initial_step_size_target(dt_);
    integrator.Initialize();

    // Verifies that setting accuracy too loose (from above) makes the working
    // accuracy different than the target accuracy after initialization.
    EXPECT_NE(integrator.get_accuracy_in_use(),
              integrator.get_target_accuracy());
  }

  // Solve a stiff double spring-mass damper. This system has a very stiff
  // spring and damper connecting two point masses together, and one of the
  // point masses is connected to "the world" using a spring with no damper. The
  // solution of this system should approximate the solution of an undamped
  // spring connected to a mass equal to the sum of both point masses.
  void DoubleSpringMassDamperTest(ReuseType type) {
    // Clone the spring mass system's state.
    std::unique_ptr<State<double>> state_copy = dspring_context_->CloneState();

    // Designate the solution tolerance.
    const double sol_tol = 2e-2;

    // Set integrator parameters.
    T integrator(*stiff_double_system_, dspring_context_.get());
    integrator.set_maximum_step_size(large_dt_);
    integrator.request_initial_step_size_target(large_dt_);
    integrator.set_target_accuracy(1e-5);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Get the solution at the target time.
    const double t_final = 1.0;
    stiff_double_system_->GetSolution(
        *dspring_context_, t_final,
        &state_copy->get_mutable_continuous_state());

    // Take all the defaults.
    integrator.Initialize();

    // Integrate.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the solution.
    const VectorX<double> nsol = dspring_context_->get_continuous_state()
                                     .get_generalized_position()
                                     .CopyToVector();
    const VectorX<double> sol = state_copy->get_continuous_state()
                                    .get_generalized_position()
                                    .CopyToVector();

    for (int i = 0; i < nsol.size(); ++i) EXPECT_NEAR(sol(i), nsol(i), sol_tol);

    // Verify that integrator statistics are valid.
    CheckGeneralStatsValidity(&integrator);
  }

  // Integrate the mass-spring-damping system using huge stiffness and damping.
  // This equation should be stiff.
  void SpringMassDamperStiffTest(ReuseType type) {
    // Create the integrator.
    T integrator(*spring_damper_, context_.get());
    integrator.set_maximum_step_size(large_dt_);
    integrator.set_requested_minimum_step_size(small_dt_);
    integrator.set_throw_on_minimum_step_size_violation(false);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Set error controlled integration parameters.
    const double xtol = 1e-6;
    const double vtol = xtol * 100;
    integrator.set_target_accuracy(xtol);

    // Set the initial position and initial velocity.
    const double initial_position = 1;
    const double initial_velocity = 0.1;

    // Set initial condition.
    spring_damper_->set_position(context_.get(), initial_position);
    spring_damper_->set_velocity(context_.get(), initial_velocity);

    // Take all the defaults.
    integrator.Initialize();

    // Integrate for sufficient time for the spring to go to rest.
    const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
    const double t_final = 2.0;
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the time.
    EXPECT_NEAR(context_->get_time(), t_final, ttol);

    // Get the final position and velocity.
    const VectorBase<double>& xc_final =
        context_->get_continuous_state().get_vector();
    double x_final = xc_final.GetAtIndex(0);
    double v_final = xc_final.GetAtIndex(1);

    // Get the closed form solution.
    double x_final_true, v_final_true;
    spring_damper_->GetClosedFormSolution(initial_position, initial_velocity,
                                          t_final, &x_final_true,
                                          &v_final_true);

    // Check the solution.
    EXPECT_NEAR(x_final_true, x_final, xtol);
    EXPECT_NEAR(v_final_true, v_final, vtol);

    // Verify that integrator statistics are valid, and reset the statistics.
    CheckGeneralStatsValidity(&integrator);

    // Switch to central differencing.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kCentralDifference);

    // Reset the time, position, and velocity.
    context_->SetTime(0.0);
    spring_damper_->set_position(context_.get(), initial_position);
    spring_damper_->set_velocity(context_.get(), initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);
    x_final = xc_final.GetAtIndex(0);
    v_final = xc_final.GetAtIndex(1);

    // Verify that integrator statistics and outputs are valid, and reset the
    // statistics.
    EXPECT_NEAR(x_final_true, x_final, xtol);
    EXPECT_NEAR(v_final_true, v_final, vtol);
    CheckGeneralStatsValidity(&integrator);

    // Switch to automatic differencing.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kAutomatic);

    // Reset the time, position, and velocity.
    context_->SetTime(0.0);
    spring_damper_->set_position(context_.get(), initial_position);
    spring_damper_->set_velocity(context_.get(), initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);
    x_final = xc_final.GetAtIndex(0);
    v_final = xc_final.GetAtIndex(1);

    // Verify that error control was used by making sure that the minimum step
    // size was smaller than large_dt_.
    EXPECT_LT(integrator.get_smallest_adapted_step_size_taken(), large_dt_);

    // Verify that integrator statistics and outputs are valid.
    EXPECT_NEAR(x_final_true, x_final, xtol);
    EXPECT_NEAR(v_final_true, v_final, vtol);
    CheckGeneralStatsValidity(&integrator);
  }

  // Integrate the modified mass-spring-damping system, which exhibits a
  // discontinuity in the velocity derivative at spring position x = 0.
  void DiscontinuousSpringMassDamperTest(ReuseType type) {
    // Create the integrator.
    T integrator(*mod_spring_damper_, context_.get());
    integrator.set_maximum_step_size(1e-5);
    integrator.set_throw_on_minimum_step_size_violation(false);
    integrator.set_target_accuracy(1e-5);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Setting the minimum step size speeds the unit test without (in this case)
    // affecting solution accuracy.
    integrator.set_requested_minimum_step_size(1e-5);

    // Set the initial position and initial velocity.
    const double initial_position = 1e-8;
    const double initial_velocity = 0;

    // Set initial condition.
    mod_spring_damper_->set_position(context_.get(), initial_position);
    mod_spring_damper_->set_velocity(context_.get(), initial_velocity);

    // Take all the defaults.
    integrator.Initialize();

    // Establish tolerances for time and solution. These tolerances are
    // arbitrary but seem to work well.
    const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
    const double sol_tol = 1e-8;

    // Integrate for 1 second.
    const double t_final = 1.0;
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the time.
    EXPECT_NEAR(context_->get_time(), t_final, ttol);

    // Get the final position.
    double x_final =
        context_->get_continuous_state().get_vector().GetAtIndex(0);

    // Verify that solution and integrator statistics are valid and reset the
    // statistics.
    EXPECT_NEAR(0.0, x_final, sol_tol);
    CheckGeneralStatsValidity(&integrator);

    // Switch the Jacobian scheme to central differencing.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kCentralDifference);

    // Reset the time, position, and velocity.
    context_->SetTime(0.0);
    mod_spring_damper_->set_position(context_.get(), initial_position);
    mod_spring_damper_->set_velocity(context_.get(), initial_velocity);

    // Integrate again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the solution and the time again, and reset the statistics again.
    x_final = context_->get_continuous_state().get_vector().GetAtIndex(0);
    EXPECT_NEAR(context_->get_time(), t_final, ttol);
    EXPECT_NEAR(0.0, x_final, sol_tol);
    CheckGeneralStatsValidity(&integrator);

    // Switch the Jacobian scheme to automatic differentiation.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kAutomatic);

    // Reset the time, position, and velocity.
    context_->SetTime(0.0);
    mod_spring_damper_->set_position(context_.get(), initial_position);
    mod_spring_damper_->set_velocity(context_.get(), initial_velocity);

    // Integrate again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the solution and the time again.
    x_final = context_->get_continuous_state().get_vector().GetAtIndex(0);
    EXPECT_NEAR(context_->get_time(), t_final, ttol);
    EXPECT_NEAR(0.0, x_final, sol_tol);
    CheckGeneralStatsValidity(&integrator);
  }

  // Integrate an undamped system and check its solution accuracy.
  void SpringMassStepTest(ReuseType type) {
    const double spring_k = 300.0;  // N/m

    // Create a new spring-mass system.
    SpringMassSystem<double> spring_mass(spring_k, mass_, false);

    // Set integrator parameters; we want error control to initially "fail",
    // necessitating step size adjustment.
    T integrator(spring_mass, context_.get());
    integrator.set_maximum_step_size(large_dt_);
    integrator.request_initial_step_size_target(large_dt_);
    integrator.set_target_accuracy(5e-5);
    integrator.set_requested_minimum_step_size(1e-6);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Setup the initial position and initial velocity.
    const double initial_position = 0.1;
    const double initial_velocity = 0.01;

    // Set initial condition.
    spring_mass.set_position(context_.get(), initial_position);
    spring_mass.set_velocity(context_.get(), initial_velocity);

    // Take all the defaults.
    integrator.Initialize();

    // Integrate for 1 second.
    const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
    const double t_final = 1.0;
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the time.
    EXPECT_NEAR(context_->get_time(), t_final, ttol);

    // Get the final position.
    double x_final =
        context_->get_continuous_state().get_vector().GetAtIndex(0);

    // Compute the true solution at t_final.
    double x_final_true, v_final_true;
    spring_mass.GetClosedFormSolution(initial_position, initial_velocity,
                                      t_final, &x_final_true, &v_final_true);

    // Check the solution to the same tolerance as the explicit Euler
    // integrator (see explicit_euler_integrator_test.cc, SpringMassStep).
    EXPECT_NEAR(x_final_true, x_final, 5e-3);

    // Verify that integrator statistics are valid and reset the statistics.
    CheckGeneralStatsValidity(&integrator);

    // Switch to central differencing.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kCentralDifference);

    // Reset the time, position, and velocity.
    context_->SetTime(0.0);
    spring_mass.set_position(context_.get(), initial_position);
    spring_mass.set_velocity(context_.get(), initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check results again.
    x_final = context_->get_continuous_state().get_vector().GetAtIndex(0);
    EXPECT_NEAR(x_final_true, x_final, 5e-3);
    EXPECT_NEAR(context_->get_time(), t_final, ttol);

    // Verify that integrator statistics are valid and reset the statistics.
    CheckGeneralStatsValidity(&integrator);

    // Switch to automatic differentiation.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kAutomatic);

    // Reset the time, position, and velocity.
    context_->SetTime(0.0);
    spring_mass.set_position(context_.get(), initial_position);
    spring_mass.set_velocity(context_.get(), initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check results again.
    x_final = context_->get_continuous_state().get_vector().GetAtIndex(0);
    EXPECT_NEAR(x_final_true, x_final, 5e-3);
    EXPECT_NEAR(context_->get_time(), t_final, ttol);

    // Verify that integrator statistics are valid
    CheckGeneralStatsValidity(&integrator);
  }

  // Checks the error estimator for the implicit Euler integrator using the
  // spring-mass system:
  // d^2x/dt^2 = -kx/m
  // solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
  // where omega = sqrt(k/m)
  // ẋ(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
  // for t = 0, x(0) = c1, ẋ(0) = c2*omega
  void ErrorEstimationTest(ReuseType type) {
    const double spring_k = 300.0;  // N/m

    // Create a new spring-mass system.
    SpringMassSystem<double> spring_mass(spring_k, mass_, false);

    // Set the integrator to operate in fixed step mode.
    T integrator(spring_mass, context_.get());
    integrator.set_maximum_step_size(large_dt_);
    integrator.set_fixed_step_mode(true);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Use automatic differentiation because we can.
    integrator.set_jacobian_computation_scheme(
        T::JacobianComputationScheme::kAutomatic);

    // Create the initial positions and velocities.
    const int n_initial_conditions = 3;
    const double initial_position[n_initial_conditions] = {0.1, 1.0, 0.0};
    const double initial_velocity[n_initial_conditions] = {0.01, 1.0, -10.0};

    // Create the integration step size array. NOTE: dt values greater than 1e-2
    // (or so) results in very poor error estimates. dt values smaller than 1e-8
    // (or so) results in NaN relative errors (indicating that solution matches
    // ideal one to very high accuracy).
    const int n_dts = 4;
    const double dts[n_dts] = {1e-8, 1e-4, 1e-3, 1e-2};

    // Take all the defaults.
    integrator.Initialize();

    // Set the allowed error on the time.
    const double ttol = 10 * std::numeric_limits<double>::epsilon();

    // Set the error estimate tolerance on absolute error. We get this by
    // starting from 1e-2 for a step size of 1e-2 and then multiply be 1e-2 for
    // each order of magnitude decrease in step size. This yields a quadratic
    // reduction in error, as expected.
    const double atol[n_dts] = {1e-14, 1e-6, 1e-4, 0.01};

    // Iterate the specified number of initial conditions.
    // Iterate over the number of integration step sizes.
    for (int j = 0; j < n_dts; ++j) {
      for (int i = 0; i < n_initial_conditions; ++i) {
        // Reset the time.
        context_->SetTime(0.0);

        // Set initial condition.
        spring_mass.set_position(context_.get(), initial_position[i]);
        spring_mass.set_velocity(context_.get(), initial_velocity[i]);

        // Integrate for the desired step size.
        ASSERT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(
            context_->get_time() + dts[j]));

        // Check the time.
        EXPECT_NEAR(context_->get_time(), dts[j], ttol);

        // Get the error estimate.
        const double est_err =
            std::abs(integrator.get_error_estimate()->CopyToVector()[0]);

        // Get the final position of the spring.
        const double x_final =
            context_->get_continuous_state().get_vector().GetAtIndex(0);

        // Get the true position.
        double x_final_true, v_final_true;
        spring_mass.GetClosedFormSolution(initial_position[i],
                                          initial_velocity[i], dts[j],
                                          &x_final_true, &v_final_true);

        // Check the relative error on position.
        const double err = std::abs(x_final - x_final_true);
        const double err_est_err = std::abs(err - est_err);
        EXPECT_LE(err, atol[j]);
        EXPECT_LE(err_est_err, atol[j]);
      }
    }
  }

  // Integrate over a significant period of time to verify that global error
  // estimation acts as we expect.
  void SpringMassStepAccuracyEffectsTest(ReuseType type) {
    const double spring_k = 300.0;  // N/m

    // Create a new spring-mass system.
    SpringMassSystem<double> spring_mass(spring_k, mass_, false);

    // Spring-mass system is necessary only to setup the problem.
    T integrator(spring_mass, context_.get());
    integrator.set_maximum_step_size(large_dt_);
    integrator.set_requested_minimum_step_size(small_dt_);
    integrator.set_throw_on_minimum_step_size_violation(false);
    integrator.set_target_accuracy(1e-4);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Setup the initial position and initial velocity.
    const double initial_position = 0.1;
    const double initial_velocity = 0.01;

    // Set initial condition.
    spring_mass.set_position(context_.get(), initial_position);
    spring_mass.set_velocity(context_.get(), initial_velocity);

    // Take all the defaults.
    integrator.Initialize();
    EXPECT_NEAR(integrator.get_accuracy_in_use(), 1e-4,
                std::numeric_limits<double>::epsilon());

    // Get the actual solution.
    double x_final_true, v_final_true;
    spring_mass.GetClosedFormSolution(initial_position, initial_velocity,
                                      large_dt_, &x_final_true, &v_final_true);

    // Integrate exactly one step.
    integrator.IntegrateWithMultipleStepsToTime(context_->get_time() +
                                                large_dt_);

    // Get the positional error.
    const double pos_err = std::abs(
        x_final_true - context_->get_continuous_state_vector().GetAtIndex(0));

    // Make the accuracy setting looser, integrate again, and verify that
    // positional error increases.
    integrator.set_target_accuracy(100.0);
    EXPECT_NEAR(integrator.get_accuracy_in_use(), 100.0,
                std::numeric_limits<double>::epsilon());
    integrator.Initialize();
    context_->SetTime(0);
    spring_mass.set_position(context_.get(), initial_position);
    spring_mass.set_velocity(context_.get(), initial_velocity);
    integrator.IntegrateWithMultipleStepsToTime(context_->get_time() +
                                                large_dt_);
    EXPECT_GT(std::abs(x_final_true -
                       context_->get_continuous_state_vector().GetAtIndex(0)),
              pos_err);
  }

  double dt() const { return dt_; }
  const SpringMassSystem<double>& spring() const { return *spring_; }
  Context<double>& context() { return *context_; }

 private:
  bool reuse_type_to_bool(ReuseType type ) {
    if (type == kNoReuse) {
      return false;
    } else {
      return true;
    }
  }

  // Checks the validity of general integrator statistics and resets statistics.
  void CheckGeneralStatsValidity(T* integrator) {
    EXPECT_GT(integrator->get_num_newton_raphson_iterations(), 0);
    EXPECT_GT(integrator->get_num_error_estimator_newton_raphson_iterations(),
              0);
    EXPECT_GT(integrator->get_previous_integration_step_size(), 0.0);
    EXPECT_GT(integrator->get_largest_step_size_taken(), 0.0);
    EXPECT_GE(integrator->get_num_steps_taken(), 0);
    EXPECT_GT(integrator->get_num_derivative_evaluations(), 0);
    EXPECT_GE(integrator->get_num_error_estimator_derivative_evaluations(), 0);
    EXPECT_GT(integrator->get_num_derivative_evaluations_for_jacobian(), 0);
    EXPECT_GE(
        integrator
            ->get_num_error_estimator_derivative_evaluations_for_jacobian(),
        0);
    EXPECT_GE(integrator->get_num_jacobian_evaluations(), 0);
    EXPECT_GE(integrator->get_num_error_estimator_jacobian_evaluations(), 0);
    EXPECT_GE(integrator->get_num_iteration_matrix_factorizations(), 0);
    EXPECT_GE(
        integrator->get_num_error_estimator_iteration_matrix_factorizations(),
        0);
    EXPECT_GE(integrator->get_num_substep_failures(), 0);
    EXPECT_GE(integrator->get_num_step_shrinkages_from_substep_failures(), 0);
    EXPECT_GE(integrator->get_num_step_shrinkages_from_error_control(), 0);
    integrator->ResetStatistics();
  }

  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<Context<double>> dspring_context_;
  std::unique_ptr<SpringMassSystem<double>> spring_;
  std::unique_ptr<implicit_integrator_test::SpringMassDamperSystem<double>>
      spring_damper_;
  std::unique_ptr<
      implicit_integrator_test::DiscontinuousSpringMassDamperSystem<double>>
      mod_spring_damper_;
  std::unique_ptr<analysis::test::StiffDoubleMassSpringSystem<double>>
      stiff_double_system_;

  const double dt_ = 1e-3;                // Default integration step size.
  const double large_dt_ = 1e-1;          // Large integration step size.
  const double small_dt_ = 1e-6;          // Smallest integration step size.
  const double mass_ = 2.0;               // Default particle mass.
  const double constant_force_mag_ = 10;  // Magnitude of the constant force.

  /// Default spring constant. Corresponds to a frequency of 0.1125 cycles per
  /// second without damping, assuming that mass = 2 (using formula
  /// f = sqrt(k/mass)/(2*pi), where k is the spring constant, and f is the
  /// frequency in cycles per second).
  const double spring_k_ = 1.0;

  /// Default spring constant for a stiff spring. Corresponds to a frequency
  /// of 11,254 cycles per second without damping, assuming that mass = 2
  /// (using formula f = sqrt(k/mass)/(2*pi), where k is the spring constant,
  /// and f is the requency in cycles per second).
  const double stiff_spring_k_ = 1e10;

  /// Default semi-stiff (in the computational sense) damping coefficient.
  /// For the "modified" spring and damper, and assuming that mass = 2 and
  /// stiff_spring_k = 1e10, this will result in a damping ratio of
  /// damping_b / (2*sqrt(mass*stiff_spring_k)) = 0.035, meaning that
  /// the system is underdamped.
  const double damping_b_ = 1e4;

  /// Default stiff (in the computational sense) damping coefficient. For
  /// the "vanilla" spring and damper, and assuming that mass = 2 and
  /// stiff_spring_k = 1e10, this will result in a damping ratio of
  /// stiff_damping_b / (2*sqrt(mass*stiff_spring_k)) = 353, meaning
  /// that the system is overdamped.
  const double stiff_damping_b_ = 1e8;
};
TYPED_TEST_CASE_P(ImplicitIntegratorTest);

TYPED_TEST_P(ImplicitIntegratorTest, MiscAPINoReuse) {
  this->MiscAPITest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, MiscAPIReuse) {
  this->MiscAPITest(kReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, FixedStepThrowsOnMultiStep) {
  auto robertson = std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Relatively large step size that we know fails to converge from the initial
  // state.
  const double dt = 1e-2;

  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(*robertson, context.get());

  // Make sure integrator can take the size we want.
  integrator.set_maximum_step_size(dt);

  // Enable fixed stepping.
  integrator.set_fixed_step_mode(true);

  // Values we have used successfully in other Robertson system tests.
  integrator.set_target_accuracy(5e-5);

  // Integrate to the desired step time. We expect this to return false because
  // the integrator is generally unlikely to converge for such a relatively
  // large step.
  integrator.Initialize();
  EXPECT_FALSE(
      integrator.IntegrateWithSingleFixedStepToTime(context->get_time() + dt));
}

TYPED_TEST_P(ImplicitIntegratorTest, ContextAccess) {
  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(this->spring(), &this->context());

  integrator.get_mutable_context()->SetTime(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(this->context().get_time(), 3.);
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  const double t_final = this->context().get_time() + this->dt();
  EXPECT_THROW(integrator.IntegrateNoFurtherThanTime(t_final, t_final, t_final),
               std::logic_error);
}

/// Verifies error estimation is supported.
TYPED_TEST_P(ImplicitIntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  using Integrator = TypeParam;
  Integrator integrator(this->spring(), &this->context());

  EXPECT_EQ(integrator.supports_error_estimation(), true);
  EXPECT_NO_THROW(integrator.set_target_accuracy(1e-1));
  EXPECT_NO_THROW(integrator.request_initial_step_size_target(this->dt()));
}


TYPED_TEST_P(ImplicitIntegratorTest, DoubleSpringMassDamperNoReuse) {
  this->DoubleSpringMassDamperTest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, DoubleSpringMassDamperReuse) {
  this->DoubleSpringMassDamperTest(kReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, SpringMassDamperStiffNoReuse) {
  this->SpringMassDamperStiffTest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, SpringMassDamperStiffReuse) {
  this->SpringMassDamperStiffTest(kReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, DiscontinuousSpringMassDamperNoReuse) {
  this->DiscontinuousSpringMassDamperTest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, DiscontinuousSpringMassDamperReuse) {
  this->DiscontinuousSpringMassDamperTest(kReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, SpringMassStepNoReuse) {
  this->SpringMassStepTest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, SpringMassStepReuse) {
  this->SpringMassStepTest(kReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, ErrorEstimationNoReuse) {
  this->ErrorEstimationTest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, ErrorEstimationReuse) {
  this->ErrorEstimationTest(kReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, SpringMassStepAccuracyEffectsNoReuse) {
  this->SpringMassStepAccuracyEffectsTest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, SpringMassStepAccuracyEffectsReuse) {
  this->SpringMassStepAccuracyEffectsTest(kReuse);
}

REGISTER_TYPED_TEST_CASE_P(
    ImplicitIntegratorTest, MiscAPINoReuse, MiscAPIReuse,
    FixedStepThrowsOnMultiStep, ContextAccess, AccuracyEstAndErrorControl,
    DoubleSpringMassDamperNoReuse, DoubleSpringMassDamperReuse,
    SpringMassDamperStiffNoReuse, SpringMassDamperStiffReuse,
    DiscontinuousSpringMassDamperNoReuse, DiscontinuousSpringMassDamperReuse,
    SpringMassStepNoReuse, SpringMassStepReuse, ErrorEstimationNoReuse,
    ErrorEstimationReuse, SpringMassStepAccuracyEffectsNoReuse,
    SpringMassStepAccuracyEffectsReuse);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

