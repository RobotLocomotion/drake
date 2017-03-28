#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator-inl.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace {

// This is a spring-mass-damper system.
template <class T>
class SpringMassDamperSystem : public SpringMassSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringMassDamperSystem);
  SpringMassDamperSystem(double spring_constant_N_per_m,
                         double damping_constant_N_per_m,
                         double mass_kg) :
    SpringMassSystem<T>(spring_constant_N_per_m, mass_kg, false /* unforced */),
    damping_constant_N_per_m_(damping_constant_N_per_m) { }

  /// Returns the damping constant that was provided at construction in N/m
  double get_damping_constant() const { return damping_constant_N_per_m_; }

 protected:
  System<AutoDiffXd>* DoToAutoDiffXd() const override {
    SpringMassDamperSystem<AutoDiffXd>* adiff_spring =
        new SpringMassDamperSystem<AutoDiffXd>(this->get_spring_constant(),
                                               get_damping_constant(),
                                               this->get_mass());
    return adiff_spring;
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T xd = state[1];
    (*derivatives)[0] = xd;

    // Compute the force acting on the mass. There is always a constant
    // force pushing the mass toward -inf. The spring and damping forces are
    // only active when x <= 0; the spring setpoint is x = 0.
    const double k = this->get_spring_constant();
    const double b = get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    T force = -k * (x - x0) - b * xd;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass();
  }

 private:
  double damping_constant_N_per_m_;
};

// This is a modified spring-mass-damper system that is only active in a small
// part of state space.
template <class T>
class ModifiedSpringMassDamperSystem : public SpringMassDamperSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModifiedSpringMassDamperSystem);
  ModifiedSpringMassDamperSystem(double spring_constant_N_per_m,
                                 double damping_constant_N_per_m,
                                 double mass_kg,
                                 double constant_force) :
    SpringMassDamperSystem<T>(spring_constant_N_per_m,
                              damping_constant_N_per_m,
                              mass_kg),
    constant_force_(constant_force) { }

  /// Gets the magnitude of the constant force acting on the system.
  double get_constant_force() const { return constant_force_; }

 protected:
  System<AutoDiffXd>* DoToAutoDiffXd() const override {
    ModifiedSpringMassDamperSystem<AutoDiffXd>* adiff_spring =
        new ModifiedSpringMassDamperSystem<AutoDiffXd>(
            this->get_spring_constant(),
            this->get_damping_constant(),
            this->get_mass(),
            get_constant_force());
    return adiff_spring;
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T xd = state[1];
    (*derivatives)[0] = xd;

    // Compute the force acting on the mass. There is always a constant
    // force pushing the mass toward -inf. The spring and damping forces are
    // only active when x <= 0; the spring setpoint is x = 0.
    T force = -constant_force_;
    const double k = this->get_spring_constant();
    const double b = this->get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    if (x <= x0)
      force -= k * (x - x0) + b * xd;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass();
  }

 private:
  double constant_force_;
};

class ImplicitIntegratorTest : public ::testing::Test {
 public:
  ImplicitIntegratorTest() {
    // Create three systems.
    spring = std::make_unique<SpringMassSystem<double>>(spring_k,
                                                        mass,
                                                        false /* no forcing */);

    spring_damper = std::make_unique<SpringMassDamperSystem<double>>(
        stiff_spring_k, stiff_damping_b, mass);
    mod_spring_damper = std::make_unique<
        ModifiedSpringMassDamperSystem<double>>(stiff_spring_k, damping_b,
                                                mass, constant_force_mag);

    // One context will be usable for all three systems.
    context = spring->CreateDefaultContext();
  }

  std::unique_ptr<Context<double>> context;
  std::unique_ptr<SpringMassSystem<double>> spring;
  std::unique_ptr<SpringMassDamperSystem<double>> spring_damper;
  std::unique_ptr<ModifiedSpringMassDamperSystem<double>> mod_spring_damper;
  const double dt = 1e-3;                // Default integration step size.
  const double large_dt = 1e-1;          // Large integration step size.
  const double spring_k = 1.0;           // Default spring constant.
  const double stiff_spring_k = 1e10;    // Default constant for a stiff spring.
  const double damping_b = 1e4;          // Default semi-stiff damper constant.
  const double stiff_damping_b = 1e8;    // Default stiff damper constant.
  const double mass = 2.0;               // Default particle mass.
  const double constant_force_mag = 10;  // Magnitude of the constant force.
  const double inf = std::numeric_limits<double>::infinity();
};

TEST_F(ImplicitIntegratorTest, MiscAPI) {
  // Create the integrator for a System<double>.
  ImplicitEulerIntegrator<double> integrator(*spring, dt, context.get());

  // Verify defaults match documentation.
  const double eps = std::numeric_limits<double>::epsilon();
  const double sqrt_eps = std::sqrt(eps);
  EXPECT_NEAR(integrator.get_delta_state_tolerance(), sqrt_eps, eps);
  EXPECT_NEAR(integrator.get_jacobian_reformulation_exponent(), 1.5, eps);
  EXPECT_EQ(integrator.get_jacobian_computation_scheme(),
            ImplicitEulerIntegrator<double>::JacobianComputationScheme::
                kForwardDifference);

  // Test that setting the target accuracy and initial step size target is
  // successful.
  EXPECT_NO_THROW(integrator.set_target_accuracy(1.0));
  EXPECT_NO_THROW(integrator.request_initial_step_size_target(1.0));
}

TEST_F(ImplicitIntegratorTest, ContextAccess) {
  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*spring, dt, context.get());

  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  EXPECT_THROW(integrator.StepOnceAtMost(dt, dt, dt), std::logic_error);
}

/// Verifies error estimation is unsupported.
TEST_F(ImplicitIntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  ImplicitEulerIntegrator<double> integrator(*spring, dt, context.get());

  EXPECT_EQ(integrator.get_error_estimate_order(), 2);
  EXPECT_EQ(integrator.supports_error_estimation(), true);
  EXPECT_NO_THROW(integrator.set_target_accuracy(1e-1));
  EXPECT_NO_THROW(integrator.request_initial_step_size_target(dt));
}

// Checks the validity of general integrator statistics and resets statistics.
void CheckGeneralStatsValidity(ImplicitEulerIntegrator<double>* integrator) {
  EXPECT_GT(integrator->get_num_newton_raphson_loops(), 0);
  EXPECT_GE(integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator->get_num_steps_taken(), 0);
  EXPECT_GT(integrator->get_num_function_evaluations(), 0);
  EXPECT_GE(integrator->get_num_jacobian_function_evaluations(), 0);
  EXPECT_GT(integrator->get_mean_scaling_factor(), 0.0);
  EXPECT_LE(integrator->get_mean_scaling_factor(), 1.0);
  integrator->ResetStatistics();
}

// Checks that decreasing the likelihood that the Jacobian will be reformulated
// increases the number of function evaluations.
TEST_F(ImplicitIntegratorTest, JacobianReformTol) {
  // Create a context.
  auto context = spring_damper->CreateDefaultContext();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*spring_damper, large_dt,
                                             context.get());

  // Enable fixed stepping to enable StepExactlyFixed().
  integrator.set_fixed_step_mode(true);

  // Use central differencing to yield a presumably more accurate
  // Jacobian matrix.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kCentralDifference);

  // Initialize the integrator.
  integrator.Initialize();

  // Set the initial conditions for the spring and amper.
  const double initial_position = 1;
  const double initial_velocity = 0.1;
  spring_damper->set_position(context.get(), initial_position);
  spring_damper->set_velocity(context.get(), initial_velocity);

  // Integrate once.
  integrator.StepExactlyFixed(large_dt);

  // Count the number of function evaluations.
  const int n_feval_reform = integrator.get_num_function_evaluations();

  // Don't allow any Jacobian reformulation.
  integrator.set_jacobian_reformulation_exponent(100.0);

  // Reset the initial conditions and integrate once again.
  spring_damper->set_position(context.get(), initial_position);
  spring_damper->set_velocity(context.get(), initial_velocity);
  integrator.StepExactlyFixed(large_dt);

  // Make sure that the number of function evaluations has gone down.
  const int n_feval_noreform = integrator.get_num_function_evaluations();
  EXPECT_GT(n_feval_noreform, n_feval_reform);
}

// Integrate the mass-spring-damping system using huge stiffness and damping.
// This equation should be stiff.
TEST_F(ImplicitIntegratorTest, SpringMassDamperStiff) {
  // Create a context.
  auto context = spring_damper->CreateDefaultContext();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*spring_damper, large_dt,
                                             context.get());

  // Set error controlled integration parameters.
  const double xtol = 1e10 * std::numeric_limits<double>::epsilon();
  integrator.set_target_accuracy(xtol);
  integrator.set_minimum_step_size(0.0);

  // Set the initial position and initial velocity.
  const double initial_position = 1;
  const double initial_velocity = 0.1;

  // Set initial condition.
  spring_damper->set_position(context.get(), initial_position);
  spring_damper->set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Integrate for sufficient time for the spring to go to rest.
  const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
  const double t_final = 2.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, large_dt));

  // Check the time.
  EXPECT_NEAR(context->get_time(), t, ttol);

  // Get the final position and velocity.
  const VectorBase<double>& xc_final = context->get_continuous_state()->
      get_vector();
  double x_final = xc_final.GetAtIndex(0);
  double v_final = xc_final.GetAtIndex(1);

  // Check the solution - the large spring should have driven the spring
  // position to zero, while the large damper should make the velocity
  // essentially zero.
  const double vtol = xtol * 100;
  EXPECT_NEAR(0.0, x_final, xtol);
  EXPECT_NEAR(0.0, v_final, vtol);

  // Verify that integrator statistics are valid
  CheckGeneralStatsValidity(&integrator);

  // Switch to central differencing.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kCentralDifference);

  // Reset the time, position, and velocity.
  context->set_time(0.0);
  spring_damper->set_position(context.get(), initial_position);
  spring_damper->set_velocity(context.get(), initial_velocity);

  // Integrate for t_final seconds again.
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, large_dt));
  x_final = xc_final.GetAtIndex(0);
  v_final = xc_final.GetAtIndex(1);

  // Verify that integrator statistics and outputs are valid.
  EXPECT_NEAR(0.0, x_final, xtol);
  EXPECT_NEAR(0.0, v_final, vtol);
  CheckGeneralStatsValidity(&integrator);

  // Switch to automatic differencing.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kAutomatic);

  // Reset the time, position, and velocity.
  context->set_time(0.0);
  spring_damper->set_position(context.get(), initial_position);
  spring_damper->set_velocity(context.get(), initial_velocity);

  // Integrate for t_final seconds again.
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, large_dt));
  x_final = xc_final.GetAtIndex(0);
  v_final = xc_final.GetAtIndex(1);

  // Verify that error control was used by making sure that the minimum step
  // size was smaller than large_dt.
  EXPECT_LT(integrator.get_smallest_adapted_step_size_taken(), large_dt);

  // Verify that integrator statistics and outputs are valid.
  EXPECT_NEAR(0.0, x_final, xtol);
  EXPECT_NEAR(0.0, v_final, vtol);
  CheckGeneralStatsValidity(&integrator);
}

// Try a purely continuous system with no sampling.
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(ImplicitIntegratorTest, SpringMassStep) {
  const double spring_k = 300.0;  // N/m

  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass, false /* no forcing */);

  // Create a new dt. This is actually a larger dt than that used for the same
  // time with the explicit Euler integrator. As these are both first order
  // integrators, we should be able to attain the same accuracy.
  const double dt = 1e-4;

  // Set integrator parameters.
  ImplicitEulerIntegrator<double> integrator(spring_mass, dt, context.get());
  integrator.set_target_accuracy(1e-4);
  integrator.set_minimum_step_size(1e-6);

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(spring_k / mass);

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Integrate for 1 second.
  const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check the time.
  EXPECT_NEAR(context->get_time(), t_final, ttol);

  // Get the final position.
  double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution to the same tolerance as the explicit Euler
  // integrator.
  EXPECT_NEAR(c1 * std::cos(omega * t) + c2 * std::sin(omega * t), x_final,
              5e-3);

  // Verify that integrator statistics are valid.
  CheckGeneralStatsValidity(&integrator);

  // Switch to central differencing.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kCentralDifference);

  // Reset the time, position, and velocity.
  context->set_time(0.0);
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Integrate for t_final seconds again.
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check results again.
  x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);
  EXPECT_NEAR(c1 * std::cos(omega * t) + c2 * std::sin(omega * t), x_final,
              5e-3);
  EXPECT_NEAR(context->get_time(), t_final, ttol);

  // Verify that integrator statistics are valid
  CheckGeneralStatsValidity(&integrator);

  // Switch to automatic differentiation.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kAutomatic);

  // Reset the time, position, and velocity.
  context->set_time(0.0);
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Integrate for t_final seconds again.
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check results again.
  x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);
  EXPECT_NEAR(c1 * std::cos(omega * t) + c2 * std::sin(omega * t), x_final,
              5e-3);
  EXPECT_NEAR(context->get_time(), t_final, ttol);

  // Verify that integrator statistics are valid
  CheckGeneralStatsValidity(&integrator);
}

// Checks the error estimator for the implicit Euler integrator using the
// spring-mass system:
// d^2x/dt^2 = -kx/m
// solution to this ODE: x(t) = c1*cos(omega*t) + c2*sin(omega*t)
// where omega = sqrt(k/m)
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(ImplicitIntegratorTest, ErrorEstimation) {
  const double spring_k = 300.0;  // N/m

  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass, false /* no forcing */);

  // Set the integrator to operate in fixed step mode and with very tight
  // tolerances.
  ImplicitEulerIntegrator<double> integrator(spring_mass, large_dt,
                                             context.get());
  integrator.set_fixed_step_mode(true);
  integrator.set_delta_state_tolerance(1e-15);

  // Use automatic differentiation because we can.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kAutomatic);

  // Create the initial positions and velocities.
  const int n_initial_conditions = 3;
  const double initial_position[n_initial_conditions] = { 0.1, 1.0, 0.0 };
  const double initial_velocity[n_initial_conditions] = { 0.01, 1.0, -10.0 };
  const double omega = std::sqrt(spring_k / mass);

  // Create the integration step size array. NOTE: dt values smaller than 1e-5
  // and greater than 1e-2 (or so) result in very poor error estimates.
  const int n_dts = 4;
  const double dts[n_dts] = { 1e-5, 1e-4, 1e-3, 1e-2 };

  // Take all the defaults.
  integrator.Initialize();

  // Set the allowed error on the time.
  const double ttol = 10 * std::numeric_limits<double>::epsilon();

  // Set the error estimate allowed error percentage. 11% error is achievable
  // for these step sizes and this problem.
  const double rtol = 0.11;

  // Iterate the specified number of initial conditions.
  // Iterate over the number of integration step sizes.
  for (int j = 0; j < n_dts; ++j) {
    for (int i = 0; i < n_initial_conditions; ++i) {
      // Reset the time.
      context->set_time(0.0);

      // Set initial condition.
      spring_mass.set_position(context.get(), initial_position[i]);
      spring_mass.set_velocity(context.get(), initial_velocity[i]);

      // Setup c1 and c2 for ODE constants.
      const double c1 = initial_position[i];
      const double c2 = initial_velocity[i] / omega;

      // Integrate for the desired step size.
      integrator.StepExactlyFixed(dts[j]);

      // Check the time.
      EXPECT_NEAR(context->get_time(), dts[j], ttol);

      // Get the error estimate.
      const double est_err = std::abs(
          integrator.get_error_estimate()->CopyToVector()[0]);

      // Get the final position of the spring.
      const double x_final =
          context->get_continuous_state()->get_vector().GetAtIndex(0);

      // Get the true position.
      const double x_final_true = c1 * std::cos(omega * dts[j]) +
          c2 * std::sin(omega * dts[j]);

      // Check the relative error on position.
      const double err = std::abs(x_final - x_final_true);
      const double rel_est_err = std::abs(err - est_err) / err;
      EXPECT_LT(rel_est_err, rtol);
    }
  }
}

// Try a purely continuous system with no sampling to verify that the
// convergence tolerances have an effect. See SpringMassStep test for
// description of the closed form solution.
TEST_F(ImplicitIntegratorTest, SpringMassStepAccuracyEffects) {
  const double spring_k = 300.0;  // N/m

  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass, false /* no forcing */);

  // Spring-mass system is necessary only to setup the problem.
  ImplicitEulerIntegrator<double> integrator(spring_mass, large_dt,
                                             context.get());

  // Turn fixed stepping on.
  integrator.set_fixed_step_mode(true);

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(spring_k / mass);

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants and compute the desired solution.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;
  const double x_des = c1 * std::cos(omega * large_dt) +
      c2 * std::sin(omega * large_dt);

  // Integrate exactly one step.
  integrator.StepExactlyFixed(large_dt);

  // Get the positional error.
  const double pos_err = std::abs(x_des -
      context->get_continuous_state_vector().GetAtIndex(0));

  // Increase the delta update tolerance, integrate again, and verify that
  // positional error increases.
  integrator.set_delta_state_tolerance(100.0);
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);
  integrator.StepExactlyFixed(large_dt);
  EXPECT_GT(std::abs(x_des -
      context->get_continuous_state_vector().GetAtIndex(0)), pos_err);
}

// Integrate the mass-spring-damping system with a constant force.
TEST_F(ImplicitIntegratorTest, ModifiedSpringMassDamper) {
  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*mod_spring_damper,
                                             dt,
                                             context.get());

  // Set the initial position and initial velocity.
  const double initial_position = 1e-8;
  const double initial_velocity = 0;

  // Set initial condition.
  mod_spring_damper->set_position(context.get(), initial_position);
  mod_spring_damper->set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Establish tolerances for time and solution.
  const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
  const double sol_tol = std::sqrt(std::numeric_limits<double>::epsilon());

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check the time.
  EXPECT_NEAR(context->get_time(), t_final, ttol);

  // Get the final position.
  double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Verify that solution and integrator statistics are valid
  EXPECT_NEAR(0.0, x_final, sol_tol);
  CheckGeneralStatsValidity(&integrator);

  // Switch the Jacobian scheme to central differencing.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kCentralDifference);

  // Reset the time, position, and velocity.
  context->set_time(0.0);
  mod_spring_damper->set_position(context.get(), initial_position);
  mod_spring_damper->set_velocity(context.get(), initial_velocity);

  // Integrate again.
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check the solution and the time again.
  x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);
  EXPECT_NEAR(context->get_time(), t_final, ttol);
  EXPECT_NEAR(0.0, x_final, sol_tol);
  CheckGeneralStatsValidity(&integrator);

  // Switch the Jacobian scheme to automatic differentiation.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kAutomatic);

  // Reset the time, position, and velocity.
  context->set_time(0.0);
  mod_spring_damper->set_position(context.get(), initial_position);
  mod_spring_damper->set_velocity(context.get(), initial_velocity);

  // Integrate again.
  for (t = 0.0; std::abs(t - t_final) >= ttol; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check the solution and the time again.
  x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);
  EXPECT_NEAR(context->get_time(), t_final, ttol);
  EXPECT_NEAR(0.0, x_final, sol_tol);
  CheckGeneralStatsValidity(&integrator);
}

}  // namespace
}  // namespace systems
}  // namespace drake
