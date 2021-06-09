#pragma once

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/test_utilities/discontinuous_spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"
#include "drake/systems/analysis/test_utilities/spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"
#include "drake/systems/analysis/test_utilities/stiff_double_mass_spring_system.h"

namespace drake {
namespace systems {

// Forward declare VelocityImplicitEulerIntegrator for the Reuse test
// so that we can check the integrator type, because the test needs to be
// different for the VIE integrator, since it uses slightly different logic.
template <class T>
class VelocityImplicitEulerIntegrator;

namespace analysis_test {

enum ReuseType { kNoReuse, kReuse };

template <typename IntegratorType>
class ImplicitIntegratorTest : public ::testing::Test {
 public:
  ImplicitIntegratorTest() {
    // Create the spring-mass systems.
    spring_mass_ = std::make_unique<SpringMassSystem<double>>(
        spring_k_, mass_, false /* no forcing */);

    spring_mass_damper_ = std::make_unique<
        implicit_integrator_test::SpringMassDamperSystem<double>>(
        stiff_spring_k_, stiff_damping_b_, mass_);

    // The discontinuous spring-mass-damper is critically damped.
    mod_spring_mass_damper_ = std::make_unique<
        implicit_integrator_test::DiscontinuousSpringMassDamperSystem<double>>(
        semistiff_spring_k_, std::sqrt(semistiff_spring_k_ / mass_), mass_,
        constant_force_mag_);

    stiff_double_system_ =
        std::make_unique<analysis::test::StiffDoubleMassSpringSystem<double>>();

    // Contexts for single mass systems.
    spring_mass_context_ = spring_mass_->CreateDefaultContext();
    spring_mass_damper_context_ = spring_mass_damper_->CreateDefaultContext();
    mod_spring_mass_damper_context_ =
        mod_spring_mass_damper_->CreateDefaultContext();

    // Separate context necessary for the double spring mass system.
    dspring_context_ = stiff_double_system_->CreateDefaultContext();
  }

  void MiscAPITest(ReuseType type) {
    // Create the integrator for a System<double>.
    IntegratorType integrator(*spring_mass_, spring_mass_context_.get());

    // Verifies set_reuse(flag) == get_reuse() == flag
    integrator.set_reuse(reuse_type_to_bool(type));
    EXPECT_EQ(integrator.get_reuse(), reuse_type_to_bool(type));

    // Verifies that calling Initialize without setting step size target or
    // maximum step size throws exception.
    EXPECT_THROW(integrator.Initialize(), std::logic_error);

    // Verify defaults match documentation.
    EXPECT_EQ(integrator.get_jacobian_computation_scheme(),
              IntegratorType::JacobianComputationScheme::kForwardDifference);

    // Test that setting the target accuracy and initial step size target is
    // successful.
    integrator.set_maximum_step_size(h_);
    if (integrator.supports_error_estimation()) {
      integrator.set_target_accuracy(1.0);
      integrator.request_initial_step_size_target(h_);
    }
    integrator.Initialize();

    // Verifies that setting accuracy too loose (from above) makes the working
    // accuracy different than the target accuracy after initialization.
    if (integrator.supports_error_estimation()) {
      EXPECT_NE(integrator.get_accuracy_in_use(),
                integrator.get_target_accuracy());
    } else {
      EXPECT_TRUE(std::isnan(integrator.get_target_accuracy()));
    }
  }

  // Solve a stiff double spring-mass damper. This system has a very stiff
  // spring and damper connecting two point masses together, and one of the
  // point masses is connected to "the world" using a spring with no damper. The
  // solution of this system should approximate the solution of an undamped
  // spring connected to a mass equal to the sum of both point masses.
  void DoubleSpringMassDamperTest(ReuseType type) {
    // Clone the spring mass system's state.
    std::unique_ptr<State<double>> state_copy = dspring_context_->CloneState();
    // Set integrator parameters.
    IntegratorType integrator(*stiff_double_system_, dspring_context_.get());

    // For fixed step integrators, we need to use a smaller step size to get
    // the desired accuracy. By experimentation, we found that 0.1 h_ works.
    double h = integrator.supports_error_estimation() ? large_h_ : 0.1 * h_;

    // Designate the solution tolerance. For reference, the true positions are
    // about 0.4351 and 1.4351.
    const double sol_tol_pos = 2e-2;
    // The velocity solution needs a looser tolerance in Radau1 and Implicit
    // Euler. For reference, the true velocity is about -4.772.
    const double sol_tol_vel = 1.2e-1;

    integrator.set_maximum_step_size(h);
    if (integrator.supports_error_estimation()) {
      integrator.request_initial_step_size_target(h);
      integrator.set_target_accuracy(1e-5);
    }
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

    // Check the position solution.
    const VectorX<double> nsol = dspring_context_->get_continuous_state()
                                     .get_generalized_position()
                                     .CopyToVector();
    const VectorX<double> sol = state_copy->get_continuous_state()
                                    .get_generalized_position()
                                    .CopyToVector();

    for (int i = 0; i < nsol.size(); ++i)
      EXPECT_NEAR(sol(i), nsol(i), sol_tol_pos);

    // Check the velocity solution.
    const VectorX<double> nsolv = dspring_context_->get_continuous_state()
                                      .get_generalized_velocity()
                                      .CopyToVector();
    const VectorX<double> solv = state_copy->get_continuous_state()
                                     .get_generalized_velocity()
                                     .CopyToVector();
    for (int i = 0; i < nsolv.size(); ++i)
      EXPECT_NEAR(solv(i), nsolv(i), sol_tol_vel);

    // Verify that integrator statistics are valid.
    CheckGeneralStatsValidity(&integrator);
  }

  // Integrate the mass-spring-damping system using huge stiffness and damping.
  // This equation should be stiff.
  void SpringMassDamperStiffTest(ReuseType type) {
    // Create the integrator.
    IntegratorType integrator(*spring_mass_damper_,
                              spring_mass_damper_context_.get());
    integrator.set_maximum_step_size(large_h_);
    integrator.set_requested_minimum_step_size(10 * small_h_);
    integrator.set_throw_on_minimum_step_size_violation(false);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Set error controlled integration parameters.
    const double xtol = 1e-6;
    const double vtol = xtol * 100;
    if (integrator.supports_error_estimation()) {
      integrator.set_target_accuracy(xtol);
    }

    // Set the initial position and initial velocity.
    const double initial_position = 1;
    const double initial_velocity = 0.1;

    // Set initial condition.
    spring_mass_damper_->set_position(spring_mass_damper_context_.get(),
                                      initial_position);
    spring_mass_damper_->set_velocity(spring_mass_damper_context_.get(),
                                      initial_velocity);

    // Take all the defaults.
    integrator.Initialize();

    // Integrate for sufficient time for the spring to go to rest.
    const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
    const double t_final = 2.0;
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the time.
    EXPECT_NEAR(spring_mass_damper_context_->get_time(), t_final, ttol);

    // Get the final position and velocity.
    const VectorBase<double>& xc_final =
        spring_mass_damper_context_->get_continuous_state().get_vector();
    double x_final = xc_final.GetAtIndex(0);
    double v_final = xc_final.GetAtIndex(1);

    // Get the closed form solution.
    double x_final_true, v_final_true;
    spring_mass_damper_->GetClosedFormSolution(initial_position,
                                               initial_velocity, t_final,
                                               &x_final_true, &v_final_true);

    // Check the solution.
    EXPECT_NEAR(x_final_true, x_final, xtol);
    EXPECT_NEAR(v_final_true, v_final, vtol);

    // Verify that integrator statistics are valid, and reset the statistics.
    CheckGeneralStatsValidity(&integrator);

    // Switch to central differencing.
    integrator.set_jacobian_computation_scheme(
        IntegratorType::JacobianComputationScheme::kCentralDifference);

    // Reset the time, position, and velocity.
    spring_mass_damper_context_->SetTime(0.0);
    spring_mass_damper_->set_position(spring_mass_damper_context_.get(),
                                      initial_position);
    spring_mass_damper_->set_velocity(spring_mass_damper_context_.get(),
                                      initial_velocity);

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
        IntegratorType::JacobianComputationScheme::kAutomatic);

    // Reset the time, position, and velocity.
    spring_mass_damper_context_->SetTime(0.0);
    spring_mass_damper_->set_position(spring_mass_damper_context_.get(),
                                      initial_position);
    spring_mass_damper_->set_velocity(spring_mass_damper_context_.get(),
                                      initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);
    x_final = xc_final.GetAtIndex(0);
    v_final = xc_final.GetAtIndex(1);

    // Verify that error control was used by making sure that the minimum step
    // size was smaller than large_h_.
    EXPECT_LT(integrator.get_smallest_adapted_step_size_taken(), large_h_);

    // Verify that integrator statistics and outputs are valid.
    EXPECT_NEAR(x_final_true, x_final, xtol);
    EXPECT_NEAR(v_final_true, v_final, vtol);
    CheckGeneralStatsValidity(&integrator);
  }

  // Integrate the modified mass-spring-damping system, which exhibits a
  // discontinuity in the velocity derivative at spring position x = 0.
  void DiscontinuousSpringMassDamperTest(ReuseType type) {
    // Create the integrator.
    IntegratorType integrator(*mod_spring_mass_damper_,
                              mod_spring_mass_damper_context_.get());
    integrator.set_maximum_step_size(h_);
    integrator.set_throw_on_minimum_step_size_violation(false);
    if (integrator.supports_error_estimation()) {
      integrator.set_target_accuracy(1e-5);
    }
    integrator.set_reuse(reuse_type_to_bool(type));

    // Setting the minimum step size speeds the unit test without (in this case)
    // affecting solution accuracy.
    integrator.set_requested_minimum_step_size(1e-8);

    // Set the initial position and initial velocity.
    const double initial_position = 1e-8;
    const double initial_velocity = 0;

    // Set initial condition.
    mod_spring_mass_damper_->set_position(mod_spring_mass_damper_context_.get(),
                                          initial_position);
    mod_spring_mass_damper_->set_velocity(mod_spring_mass_damper_context_.get(),
                                          initial_velocity);

    // Take all the defaults.
    integrator.Initialize();

    // Establish tolerances for time and solution. These tolerances are
    // arbitrary but seem to work well.
    const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
    const double sol_tol = 1e-12;

    // Integrate for 1 second.
    const double t_final = 1.0;
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the time.
    EXPECT_NEAR(mod_spring_mass_damper_context_->get_time(), t_final, ttol);

    // Get the final position and velocity.
    double x_final = mod_spring_mass_damper_context_->get_continuous_state()
                         .get_vector()
                         .GetAtIndex(0);
    double xdot_final = mod_spring_mass_damper_context_->get_continuous_state()
                            .get_vector()
                            .GetAtIndex(1);

    // TODO(edrumwri) accurate x_final should be the equilibrium solution, where
    // the velocity is zero and the spring and external forces are equal (and
    // opposite).
    const double equilibrium_position =
        -constant_force_magnitude() / semistiff_spring_stiffness();
    const double equilibrium_velocity = 0.0;

    // Verify that solution and integrator statistics are valid and reset
    // the statistics.
    EXPECT_NEAR(equilibrium_position, x_final, sol_tol);
    EXPECT_NEAR(equilibrium_velocity, xdot_final, sol_tol);
    CheckGeneralStatsValidity(&integrator);

    // Switch the Jacobian scheme to central differencing.
    integrator.set_jacobian_computation_scheme(
        IntegratorType::JacobianComputationScheme::kCentralDifference);

    // Reset the time, position, and velocity.
    mod_spring_mass_damper_context_->SetTime(0.0);
    mod_spring_mass_damper_->set_position(
        mod_spring_mass_damper_context_.get(), initial_position);
    mod_spring_mass_damper_->set_velocity(
        mod_spring_mass_damper_context_.get(), initial_velocity);

    // Integrate again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the solution and the time again, and reset the statistics again.
    x_final = mod_spring_mass_damper_context_->get_continuous_state()
                  .get_vector()
                  .GetAtIndex(0);
    xdot_final = mod_spring_mass_damper_context_->get_continuous_state()
                      .get_vector()
                      .GetAtIndex(1);
    EXPECT_NEAR(mod_spring_mass_damper_context_->get_time(), t_final, ttol);
    EXPECT_NEAR(equilibrium_position, x_final, sol_tol);
    EXPECT_NEAR(equilibrium_velocity, xdot_final, sol_tol);
    CheckGeneralStatsValidity(&integrator);

    // Switch the Jacobian scheme to automatic differentiation.
    integrator.set_jacobian_computation_scheme(
        IntegratorType::JacobianComputationScheme::kAutomatic);

    // Reset the time, position, and velocity.
    mod_spring_mass_damper_context_->SetTime(0.0);
    mod_spring_mass_damper_->set_position(
        mod_spring_mass_damper_context_.get(), initial_position);
    mod_spring_mass_damper_->set_velocity(
        mod_spring_mass_damper_context_.get(), initial_velocity);

    // Integrate again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the solution and the time again.
    x_final = mod_spring_mass_damper_context_->get_continuous_state()
                  .get_vector()
                  .GetAtIndex(0);
    xdot_final = mod_spring_mass_damper_context_->get_continuous_state()
                      .get_vector()
                      .GetAtIndex(1);
    EXPECT_NEAR(mod_spring_mass_damper_context_->get_time(), t_final, ttol);
    EXPECT_NEAR(equilibrium_position, x_final, sol_tol);
    EXPECT_NEAR(equilibrium_velocity, xdot_final, sol_tol);
    CheckGeneralStatsValidity(&integrator);
  }

  // Integrate an undamped system and check its solution accuracy.
  void SpringMassStepTest(ReuseType type) {
    const double spring_k = 300.0;  // N/m

    // Create a new spring-mass system.
    SpringMassSystem<double> spring_mass(spring_k, mass_, false);
    std::unique_ptr<Context<double>> context =
        spring_mass.CreateDefaultContext();

    // Set integrator parameters; we want error control to initially "fail",
    // necessitating step size adjustment.
    IntegratorType integrator(spring_mass, context.get());
    // For fixed step integrators, we need to use a smaller step size to get
    // the desired accuracy. By experimentation, we found that 0.5 h_ works.
    double h = integrator.supports_error_estimation() ? large_h_ : 0.5 * h_;
    integrator.set_maximum_step_size(h);
    if (integrator.supports_error_estimation()) {
      integrator.request_initial_step_size_target(h);
      integrator.set_target_accuracy(5e-5);
    }
    integrator.set_requested_minimum_step_size(1e-6);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Setup the initial position and initial velocity.
    const double initial_position = 0.1;
    const double initial_velocity = 0.01;

    // Set initial condition.
    spring_mass.set_position(context.get(), initial_position);
    spring_mass.set_velocity(context.get(), initial_velocity);

    // Take all the defaults.
    integrator.Initialize();

    // Integrate for 1 second.
    const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
    const double t_final = 1.0;
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check the time.
    EXPECT_NEAR(context->get_time(), t_final, ttol);

    // Get the final position.
    double x_final = context->get_continuous_state().get_vector().GetAtIndex(0);

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
        IntegratorType::JacobianComputationScheme::kCentralDifference);

    // Reset the time, position, and velocity.
    context->SetTime(0.0);
    spring_mass.set_position(context.get(), initial_position);
    spring_mass.set_velocity(context.get(), initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check results again.
    x_final = context->get_continuous_state().get_vector().GetAtIndex(0);
    EXPECT_NEAR(x_final_true, x_final, 5e-3);
    EXPECT_NEAR(context->get_time(), t_final, ttol);

    // Verify that integrator statistics are valid and reset the statistics.
    CheckGeneralStatsValidity(&integrator);

    // Switch to automatic differentiation.
    integrator.set_jacobian_computation_scheme(
        IntegratorType::JacobianComputationScheme::kAutomatic);

    // Reset the time, position, and velocity.
    context->SetTime(0.0);
    spring_mass.set_position(context.get(), initial_position);
    spring_mass.set_velocity(context.get(), initial_velocity);

    // Integrate for t_final seconds again.
    integrator.IntegrateWithMultipleStepsToTime(t_final);

    // Check results again.
    x_final = context->get_continuous_state().get_vector().GetAtIndex(0);
    EXPECT_NEAR(x_final_true, x_final, 5e-3);
    EXPECT_NEAR(context->get_time(), t_final, ttol);

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
    std::unique_ptr<Context<double>> context =
        spring_mass.CreateDefaultContext();

    // Set the integrator to operate in fixed step mode.
    IntegratorType integrator(spring_mass, context.get());

    // Skip this test if the integrator doesn't have error estimate.
    if (!integrator.supports_error_estimation()) GTEST_SKIP();

    integrator.set_maximum_step_size(large_h_);
    integrator.set_fixed_step_mode(true);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Use automatic differentiation because we can.
    integrator.set_jacobian_computation_scheme(
        IntegratorType::JacobianComputationScheme::kAutomatic);

    // Create the initial positions and velocities.
    const int n_initial_conditions = 3;
    const double initial_position[n_initial_conditions] = {0.1, 1.0, 0.0};
    const double initial_velocity[n_initial_conditions] = {0.01, 1.0, -10.0};

    // Create the integration step size array. NOTE: h values greater than 1e-2
    // (or so) results in very poor error estimates. h values smaller than 1e-8
    // (or so) results in NaN relative errors (indicating that solution matches
    // ideal one to very high accuracy).
    const int n_h = 4;
    const double h[n_h] = {1e-8, 1e-4, 1e-3, 1e-2};

    // Take all the defaults.
    integrator.Initialize();

    // Set the allowed error on the time.
    const double ttol = 10 * std::numeric_limits<double>::epsilon();

    // Set the error estimate tolerance on absolute error. We get this by
    // starting from 1e-2 for a step size of 1e-2 and then multiply be 1e-2 for
    // each order of magnitude decrease in step size. This yields a quadratic
    // reduction in error, as expected.
    const double atol[n_h] = {1e-14, 1e-6, 1e-4, 0.01};

    // Iterate the specified number of initial conditions.
    // Iterate over the number of integration step sizes.
    for (int j = 0; j < n_h; ++j) {
      for (int i = 0; i < n_initial_conditions; ++i) {
        // Reset the time.
        context->SetTime(0.0);

        // Set initial condition.
        spring_mass.set_position(context.get(), initial_position[i]);
        spring_mass.set_velocity(context.get(), initial_velocity[i]);

        // Integrate for the desired step size.
        ASSERT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(
            context->get_time() + h[j]));

        // Check the time.
        EXPECT_NEAR(context->get_time(), h[j], ttol);

        // Get the error estimate.
        const double est_err =
            std::abs(integrator.get_error_estimate()->CopyToVector()[0]);

        // Get the final position of the spring.
        const double x_final =
            context->get_continuous_state().get_vector().GetAtIndex(0);

        // Get the true position.
        double x_final_true, v_final_true;
        spring_mass.GetClosedFormSolution(initial_position[i],
                                          initial_velocity[i], h[j],
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
    std::unique_ptr<Context<double>> context =
        spring_mass.CreateDefaultContext();

    // Spring-mass system is necessary only to setup the problem.
    IntegratorType integrator(spring_mass, context.get());
    if (!integrator.supports_error_estimation()) GTEST_SKIP();
    integrator.set_maximum_step_size(large_h_);
    integrator.set_requested_minimum_step_size(small_h_);
    integrator.set_throw_on_minimum_step_size_violation(false);
    integrator.set_target_accuracy(1e-4);
    integrator.set_reuse(reuse_type_to_bool(type));

    // Setup the initial position and initial velocity.
    const double initial_position = 0.1;
    const double initial_velocity = 0.01;

    // Set initial condition.
    spring_mass.set_position(context.get(), initial_position);
    spring_mass.set_velocity(context.get(), initial_velocity);

    // Take all the defaults.
    integrator.Initialize();
    EXPECT_NEAR(integrator.get_accuracy_in_use(), 1e-4,
                std::numeric_limits<double>::epsilon());

    // Get the actual solution.
    double x_final_true, v_final_true;
    spring_mass.GetClosedFormSolution(initial_position, initial_velocity,
                                      large_h_, &x_final_true, &v_final_true);

    // Integrate exactly one step.
    integrator.IntegrateWithMultipleStepsToTime(context->get_time() + large_h_);

    // Get the positional error.
    const double pos_err = std::abs(
        x_final_true - context->get_continuous_state_vector().GetAtIndex(0));

    // Make the accuracy setting looser, integrate again, and verify that
    // positional error increases.
    integrator.set_target_accuracy(100.0);
    EXPECT_NEAR(integrator.get_accuracy_in_use(), 100.0,
                std::numeric_limits<double>::epsilon());
    integrator.Initialize();
    context->SetTime(0);
    spring_mass.set_position(context.get(), initial_position);
    spring_mass.set_velocity(context.get(), initial_velocity);
    integrator.IntegrateWithMultipleStepsToTime(context->get_time() + large_h_);
    EXPECT_GT(std::abs(x_final_true -
                       context->get_continuous_state_vector().GetAtIndex(0)),
              pos_err);
  }

  double h() const { return h_; }
  double constant_force_magnitude() const { return constant_force_mag_; }
  double semistiff_spring_stiffness() const { return semistiff_spring_k_; }
  const SpringMassSystem<double>& spring_mass() const { return *spring_mass_; }

  // Checks the validity of general integrator statistics and resets statistics.
  static void CheckGeneralStatsValidity(IntegratorType* integrator) {
    EXPECT_GT(integrator->get_num_newton_raphson_iterations(), 0);
    if (integrator->supports_error_estimation()) {
      EXPECT_GT(integrator->get_num_error_estimator_newton_raphson_iterations(),
                0);
    }
    EXPECT_GT(integrator->get_previous_integration_step_size(), 0.0);
    EXPECT_GT(integrator->get_largest_step_size_taken(), 0.0);
    EXPECT_GE(integrator->get_num_steps_taken(), 0);
    EXPECT_GT(integrator->get_num_derivative_evaluations(), 0);
    if (integrator->supports_error_estimation()) {
      EXPECT_GE(integrator->get_num_error_estimator_derivative_evaluations(),
                0);
    }
    EXPECT_GT(integrator->get_num_derivative_evaluations_for_jacobian(), 0);
    if (integrator->supports_error_estimation()) {
      EXPECT_GE(
          integrator
              ->get_num_error_estimator_derivative_evaluations_for_jacobian(),
          0);
    }
    EXPECT_GE(integrator->get_num_jacobian_evaluations(), 0);
    if (integrator->supports_error_estimation()) {
      EXPECT_GE(integrator->get_num_error_estimator_jacobian_evaluations(), 0);
    }
    EXPECT_GE(integrator->get_num_iteration_matrix_factorizations(), 0);
    if (integrator->supports_error_estimation()) {
      EXPECT_GE(
          integrator->get_num_error_estimator_iteration_matrix_factorizations(),
          0);
    }
    EXPECT_GE(integrator->get_num_substep_failures(), 0);
    EXPECT_GE(integrator->get_num_step_shrinkages_from_substep_failures(), 0);
    EXPECT_GE(integrator->get_num_step_shrinkages_from_error_control(), 0);
    integrator->ResetStatistics();
  }

 protected:
  std::unique_ptr<Context<double>> spring_mass_context_;
  std::unique_ptr<Context<double>> spring_mass_damper_context_;
  std::unique_ptr<Context<double>> mod_spring_mass_damper_context_;
  std::unique_ptr<Context<double>> dspring_context_;
  std::unique_ptr<SpringMassSystem<double>> spring_mass_;
  std::unique_ptr<implicit_integrator_test::SpringMassDamperSystem<double>>
      spring_mass_damper_;
  std::unique_ptr<
      implicit_integrator_test::DiscontinuousSpringMassDamperSystem<double>>
      mod_spring_mass_damper_;
  std::unique_ptr<analysis::test::StiffDoubleMassSpringSystem<double>>
      stiff_double_system_;

 private:
  bool reuse_type_to_bool(ReuseType type) {
    if (type == kNoReuse) {
      return false;
    } else {
      return true;
    }
  }

  const double h_ = 1e-3;                 // Default integration step size.
  const double large_h_ = 1e-1;           // Large integration step size.
  const double small_h_ = 1e-6;           // Smallest integration step size.
  const double mass_ = 2.0;               // Default particle mass.
  const double constant_force_mag_ = 10;  // Magnitude of the constant force.

  // Default spring constant. Corresponds to a frequency of 0.1125 cycles per
  // second without damping, assuming that mass = 2 (using formula
  // f = sqrt(k/mass)/(2*pi), where k is the spring constant, and f is the
  // frequency in cycles per second).
  const double spring_k_ = 1.0;

  // Default spring constant for a semi-stiff spring. Corresponds to a
  // frequency of 35.588 cycles per second without damping, assuming that mass
  // = 2 (using formula f = sqrt(k/mass)/(2*pi), where k is the spring
  // constant, and f is the requency in cycles per second).
  const double semistiff_spring_k_ = 1e5;

  // Default spring constant for a stiff spring. Corresponds to a frequency
  // of 11,254 cycles per second without damping, assuming that mass = 2
  // (using formula f = sqrt(k/mass)/(2*pi), where k is the spring constant,
  // and f is the requency in cycles per second).
  const double stiff_spring_k_ = 1e10;

  // Default semi-stiff (in the computational sense) damping coefficient.
  // For the "modified" spring and damper, and assuming that mass = 2 and
  // stiff_spring_k = 1e10, this will result in a damping ratio of
  // damping_b / (2*sqrt(mass*stiff_spring_k)) = 0.035, meaning that
  // the system is underdamped.
  const double damping_b_ = 1e4;

  // Default stiff (in the computational sense) damping coefficient. For
  // the "vanilla" spring and damper, and assuming that mass = 2 and
  // stiff_spring_k = 1e10, this will result in a damping ratio of
  // stiff_damping_b / (2*sqrt(mass*stiff_spring_k)) = 353, meaning
  // that the system is overdamped.
  const double stiff_damping_b_ = 1e8;
};

TYPED_TEST_SUITE_P(ImplicitIntegratorTest);

TYPED_TEST_P(ImplicitIntegratorTest, MiscAPINoReuse) {
  this->MiscAPITest(kNoReuse);
}

TYPED_TEST_P(ImplicitIntegratorTest, MiscAPIReuse) {
  this->MiscAPITest(kReuse);
}

// Tests the Jacobian and iteration matrix reuse strategies using a test
// problem and integrator for which we have knowledge of the convergence
// behavior from the initial state.
TYPED_TEST_P(ImplicitIntegratorTest, Reuse) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
      std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(*robertson, context.get());

  integrator.set_maximum_step_size(1e-2);  // Maximum step to be attempted.
  integrator.set_throw_on_minimum_step_size_violation(false);
  integrator.set_fixed_step_mode(true);
  integrator.set_reuse(true);  // The whole point of this.

  // Attempt to integrate the system. Our past experience indicates that this
  // system fails to converge from the initial state for this large step size.
  // This tests the case where the Jacobian matrix has yet to be formed. There
  // should be one Jacobian matrix evaluation - once at trial 1. There should
  // also be two iteration matrix factorizations: once at trial 1, and another
  // at trial 2. Trial 3 should be skipped because the first Jacobian matrix
  // computation makes the Jacobian "fresh". The exception is the
  // VelocityImplicitEulerIntegrator, which will recompute both on trial 3
  // because it does not reuse Jacobians for different step sizes; hence
  // it will have 3 factorizations and 2 Jacobian evaluations.
  // TODO(antequ): see TODO in ImplicitIntegrator::MaybeFreshenMatrices()
  // for potential improvements that will require changes here.
  integrator.Initialize();
  ASSERT_FALSE(integrator.IntegrateWithSingleFixedStepToTime(1e-2));
  if (!std::is_same_v<Integrator, VelocityImplicitEulerIntegrator<double>>) {
    EXPECT_EQ(integrator.get_num_iteration_matrix_factorizations(), 2);
    EXPECT_EQ(integrator.get_num_jacobian_evaluations(), 1);
  } else {
    EXPECT_EQ(integrator.get_num_iteration_matrix_factorizations(), 3);
    EXPECT_EQ(integrator.get_num_jacobian_evaluations(), 2);
  }

  // Now integrate again but with a smaller size. Again, past experience
  // that this step size should be sufficiently small for the integrator to
  // converge. The Jacobian matrix will be "fresh"; we assume no knowledge
  // of the number of iteration matrix factorizations.
  integrator.ResetStatistics();
  ASSERT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(1e-6));
  EXPECT_EQ(integrator.get_num_jacobian_evaluations(), 0);

  // Again try taking a large step, which we expect will be too large to
  // converge. There should be one Jacobian matrix evaluation- once at trial 3.
  // There should be two iteration matrix factorizations: one at trial 2 and
  // another at trial 3.
  integrator.ResetStatistics();
  ASSERT_FALSE(integrator.IntegrateWithSingleFixedStepToTime(1e-2));
  EXPECT_EQ(integrator.get_num_iteration_matrix_factorizations(), 2);
  EXPECT_EQ(integrator.get_num_jacobian_evaluations(), 1);
}

// Tests that the full-Newton approach computes a Jacobian matrix and factorizes
// the iteration matrix on every Newton-Raphson iteration.
TYPED_TEST_P(ImplicitIntegratorTest, FullNewton) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
      std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(*robertson, context.get());

  if (integrator.supports_error_estimation()) {
    integrator.request_initial_step_size_target(1e0);
  } else {
    integrator.set_maximum_step_size(1e0);
  }
  integrator.set_throw_on_minimum_step_size_violation(false);
  integrator.set_fixed_step_mode(true);
  integrator.set_use_full_newton(true);  // The whole point of this test.

  // Attempt to integrate the system. Our past experience indicates that this
  // system fails to converge from the initial state for this large step size.
  // This tests the case where the Jacobian matrix has yet to be formed.
  integrator.Initialize();
  ASSERT_FALSE(integrator.IntegrateWithSingleFixedStepToTime(1e0));
  EXPECT_EQ(integrator.get_num_iteration_matrix_factorizations(),
            integrator.get_num_newton_raphson_iterations());
  EXPECT_EQ(integrator.get_num_jacobian_evaluations(),
            integrator.get_num_newton_raphson_iterations());

  // Now integrate again but with a smaller size. Again, past experience tells
  // us that this step size should be sufficiently small for the integrator to
  // converge.
  integrator.ResetStatistics();
  ASSERT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(1e-6));
  EXPECT_EQ(integrator.get_num_iteration_matrix_factorizations(),
            integrator.get_num_newton_raphson_iterations());
  EXPECT_EQ(integrator.get_num_jacobian_evaluations(),
            integrator.get_num_newton_raphson_iterations());

  // Again try taking a large step, which we expect will be too large to
  // converge.
  integrator.ResetStatistics();
  ASSERT_FALSE(integrator.IntegrateWithSingleFixedStepToTime(1e0));
  EXPECT_EQ(integrator.get_num_iteration_matrix_factorizations(),
            integrator.get_num_newton_raphson_iterations());
  EXPECT_EQ(integrator.get_num_jacobian_evaluations(),
            integrator.get_num_newton_raphson_iterations());
}

// Tests the implicit integrator on a stationary system problem, which
// stresses numerical differentiation (since the state does not change).
// This test also verifies that integration with AutoDiff'd Jacobians
// succeeds when the derivative does not depend on the state.
TYPED_TEST_P(ImplicitIntegratorTest, Stationary) {
  auto stationary = std::make_unique<StationarySystem<double>>();
  std::unique_ptr<Context<double>> context = stationary->CreateDefaultContext();

  // Set the initial condition for the stationary system.
  VectorBase<double>& state =
      context->get_mutable_continuous_state().get_mutable_vector();
  state.SetAtIndex(0, 0.0);
  state.SetAtIndex(1, 0.0);

  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(*stationary, context.get());

  integrator.set_maximum_step_size(1.0);

  if (integrator.supports_error_estimation()) {
    integrator.set_target_accuracy(1e-3);
    integrator.request_initial_step_size_target(1e-3);
  }

  // Integrate the system
  integrator.Initialize();
  integrator.IntegrateWithMultipleStepsToTime(1.0);

  // Verify the solution.
  EXPECT_NEAR(state.GetAtIndex(0), 0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(state.GetAtIndex(1), 0, std::numeric_limits<double>::epsilon());
  ImplicitIntegratorTest<Integrator>::CheckGeneralStatsValidity(&integrator);

  // Set up the same problem with an AutoDiff'd Jacobian computation
  // scheme.
  integrator.set_jacobian_computation_scheme(
      Integrator::JacobianComputationScheme::kAutomatic);

  // Reset the time, position, and velocity.
  context->SetTime(0.0);
  VectorBase<double>& new_state =
      context->get_mutable_continuous_state().get_mutable_vector();
  new_state.SetAtIndex(0, 0.0);
  new_state.SetAtIndex(1, 0.0);

  integrator.set_maximum_step_size(1.0);

  if (integrator.supports_error_estimation()) {
    integrator.set_target_accuracy(1e-3);
    integrator.request_initial_step_size_target(1e-3);
  }

  // Integrate the system
  integrator.IntegrateWithMultipleStepsToTime(1.0);

  // Verify the solution.
  EXPECT_NEAR(new_state.GetAtIndex(0), 0,
              std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(new_state.GetAtIndex(1), 0,
              std::numeric_limits<double>::epsilon());
  ImplicitIntegratorTest<Integrator>::CheckGeneralStatsValidity(&integrator);
}

// Tests the implicit integrator on Robertson's stiff chemical reaction
// problem, which has been used to benchmark various implicit integrators.
// This problem is particularly good at testing large step sizes (since the
// solution quickly converges) and long simulation times.
TYPED_TEST_P(ImplicitIntegratorTest, Robertson) {
  std::unique_ptr<analysis::test::RobertsonSystem<double>> robertson =
      std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  const double t_final = robertson->get_end_time();
  const double tol = 5e-5;

  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(*robertson, context.get());

  // Very large step is necessary for this problem since given solution is
  // at t = 1e11. However, the current initial step size selection algorithm
  // will use a large factor of the maximum step size, which can result in
  // too large an initial step for this problem. Accordingly, we explicitly
  // select a small initial step size.
  // @TODO(edrumwri): Explore a better algorithm for selecting the initial
  //                  step size (see issue #6329).
  integrator.set_maximum_step_size(10000000.0);
  integrator.set_throw_on_minimum_step_size_violation(false);

  if (integrator.supports_error_estimation()) {
    integrator.set_target_accuracy(tol);
    integrator.request_initial_step_size_target(1e-4);
  }

  // Integrate the system
  integrator.Initialize();
  integrator.IntegrateWithMultipleStepsToTime(t_final);

  // Verify the solution.
  const VectorBase<double>& state =
      context->get_continuous_state().get_vector();
  const Eigen::Vector3d sol = robertson->GetSolution(t_final);
  EXPECT_NEAR(state.GetAtIndex(0), sol(0), tol);
  EXPECT_NEAR(state.GetAtIndex(1), sol(1), tol);
  EXPECT_NEAR(state.GetAtIndex(2), sol(2), tol);
}

TYPED_TEST_P(ImplicitIntegratorTest, FixedStepThrowsOnMultiStep) {
  auto robertson = std::make_unique<analysis::test::RobertsonSystem<double>>();
  std::unique_ptr<Context<double>> context = robertson->CreateDefaultContext();

  // Relatively large step size that we know fails to converge from the initial
  // state.
  const double h = 1e-2;

  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(*robertson, context.get());

  // Make sure integrator can take the size we want.
  integrator.set_maximum_step_size(h);

  // Enable fixed stepping.
  integrator.set_fixed_step_mode(true);

  // Values we have used successfully in other Robertson system tests.
  if (integrator.supports_error_estimation()) {
    integrator.set_target_accuracy(5e-5);
  }

  // Integrate to the desired step time. We expect this to return false because
  // the integrator is generally unlikely to converge for such a relatively
  // large step.
  integrator.Initialize();
  EXPECT_FALSE(
      integrator.IntegrateWithSingleFixedStepToTime(context->get_time() + h));
}

TYPED_TEST_P(ImplicitIntegratorTest, ContextAccess) {
  // Create the integrator.
  using Integrator = TypeParam;
  Integrator integrator(this->spring_mass(), this->spring_mass_context_.get());

  integrator.get_mutable_context()->SetTime(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  const double t_final = integrator.get_context().get_time() + this->h();
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  EXPECT_THROW(integrator.IntegrateNoFurtherThanTime(t_final, t_final, t_final),
               std::logic_error);
}

/// Verifies error estimation is supported.
TYPED_TEST_P(ImplicitIntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  using Integrator = TypeParam;
  Integrator integrator(this->spring_mass(), this->spring_mass_context_.get());

  if (!integrator.supports_error_estimation()) GTEST_SKIP();

  EXPECT_EQ(integrator.supports_error_estimation(), true);
  DRAKE_EXPECT_NO_THROW(integrator.set_target_accuracy(1e-1));
  DRAKE_EXPECT_NO_THROW(integrator.request_initial_step_size_target(this->h()));
}

// Tests accuracy for integrating linear systems (with the state at time t
// corresponding to f(t) ≡ St + C, where S is a scalar and C is the initial
// state) over t ∈ [0, 1]. The asymptotic term in every implicit integrator's
// error estimate is at least second order, meaning that it uses the Taylor
// Series expansion: f(t+h) ≈ f(t) + hf'(t) + O(h²). This formula indicates that
// the approximation error will be zero if f''(t) = 0, which is true for linear
// systems. We check that the error estimator gives a perfect error estimate for
// this function.
TYPED_TEST_P(ImplicitIntegratorTest, LinearTest) {
  LinearScalarSystem linear;
  auto linear_context = linear.CreateDefaultContext();
  const double C = linear.Evaluate(0);
  linear_context->SetTime(0.0);
  linear_context->get_mutable_continuous_state_vector()[0] = C;

  using Integrator = TypeParam;
  Integrator integrator1(linear, linear_context.get());
  const double t_final = 1.0;
  integrator1.set_maximum_step_size(t_final);
  integrator1.set_fixed_step_mode(true);
  integrator1.Initialize();
  ASSERT_TRUE(integrator1.IntegrateWithSingleFixedStepToTime(t_final));
  if (integrator1.supports_error_estimation()) {
    const double err_est = integrator1.get_error_estimate()->get_vector()[0];

    // Note the very tight tolerance used, which will likely not hold for
    // arbitrary values of C, t_final, or polynomial coefficients.
    EXPECT_NEAR(err_est, 0.0, 2 * std::numeric_limits<double>::epsilon());

    // Verify the solution.
    VectorX<double> state =
        linear_context->get_continuous_state().get_vector().CopyToVector();
    EXPECT_NEAR(state[0], linear.Evaluate(t_final),
                std::numeric_limits<double>::epsilon());

    // Repeat this test, but using a final time that is below the working
    // minimum step size (thereby triggering the implicit integrator's
    // alternate, explicit mode). To retain our existing tolerances, we change
    // the scale factor (S) for the linear system.
    integrator1.get_mutable_context()->SetTime(0);
    const double working_min = integrator1.get_working_minimum_step_size();
    LinearScalarSystem scaled_linear(4.0 / working_min);
    auto scaled_linear_context = scaled_linear.CreateDefaultContext();
    Integrator integrator2(scaled_linear, scaled_linear_context.get());
    const double updated_t_final = working_min / 2;
    integrator2.set_maximum_step_size(updated_t_final);
    integrator2.set_fixed_step_mode(true);
    integrator2.Initialize();
    ASSERT_TRUE(
        integrator2.IntegrateWithSingleFixedStepToTime(updated_t_final));

    const double updated_err_est =
        integrator2.get_error_estimate()->get_vector()[0];

    // Note the very tight tolerance used, which will likely not hold for
    // arbitrary values of C, t_final, or polynomial coefficients.
    EXPECT_NEAR(updated_err_est, 0.0,
                2 * std::numeric_limits<double>::epsilon());

    // Verify the solution too.
    EXPECT_NEAR(scaled_linear_context->get_continuous_state()
                    .get_vector()
                    .CopyToVector()[0],
                scaled_linear.Evaluate(updated_t_final),
                10 * std::numeric_limits<double>::epsilon());
  }
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

REGISTER_TYPED_TEST_SUITE_P(
    ImplicitIntegratorTest, Reuse, FullNewton, MiscAPINoReuse, MiscAPIReuse,
    Stationary, Robertson, FixedStepThrowsOnMultiStep, ContextAccess,
    AccuracyEstAndErrorControl, LinearTest, DoubleSpringMassDamperNoReuse,
    DoubleSpringMassDamperReuse, SpringMassDamperStiffNoReuse,
    SpringMassDamperStiffReuse, DiscontinuousSpringMassDamperNoReuse,
    DiscontinuousSpringMassDamperReuse, SpringMassStepNoReuse,
    SpringMassStepReuse, ErrorEstimationNoReuse, ErrorEstimationReuse,
    SpringMassStepAccuracyEffectsNoReuse, SpringMassStepAccuracyEffectsReuse);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
