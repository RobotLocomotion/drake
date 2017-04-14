#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator-inl.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/systems/analysis/test/my_spring_mass_system.h"

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
    return new SpringMassDamperSystem<AutoDiffXd>(this->get_spring_constant(),
                                               get_damping_constant(),
                                               this->get_mass());
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T xd = state[1];
    (*derivatives)[0] = xd;

    // Compute the force acting on the mass.
    const double k = this->get_spring_constant();
    const double b = get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    T force = -k * (x - x0) - b * xd;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass();
  }

  /// Returns the closed-form position and velocity solution for the unforced
  /// spring-mass-damper from the given initial conditions.
  /// @param x0 the position of the spring at time t = 0.
  /// @param v0 the velocity of the spring at time t = 0.
  /// @param tf the time at which to return the position and velocity.
  /// @param[out] xf the position of the spring at time tf, on return.
  /// @param[out] vf the velocity of the spring at time tf, on return.
  /// @throws std::logic_error if xf or vf is nullptr.
  void get_closed_form_solution(const T& x0, const T& v0, const T& tf,
                                        T* xf, T* vf) const override {
    using std::sqrt;
    using std::sin;
    using std::cos;
    using std::exp;

    if (!xf || !vf)
      throw std::logic_error("Passed final position/velocity is null.");

    // m⋅d²x/dt² + c⋅dx/dt + kx = 0
    // Solution to this ODE: x(t) = c₁⋅eʳᵗ + c₂⋅eˢᵗ
    //   where r and s are the roots to the equation mz² + cz + k = 0.
    // Thus, dx/dt = r⋅c₁⋅eʳᵗ + s⋅c₂⋅eˢᵗ.

    // Step 1: Solve the equation for z, yielding r and s.
    T r, s;
    std::tie(r, s) = SolveQuadratic(this->get_mass(), get_damping_constant(),
                                    this->get_spring_constant());

    // Step 2: Substituting t = 0 into the equatinons above, solve the resulting
    // linear system:
    // c1 + c2 = x0
    // r⋅c1 + s⋅c2 = v0
    // yielding:
    // c1 = -(-v0 + s⋅x0)/(r - s) and c2 = -(v0 - r⋅x0)/(r - s)
    const T c1 = -(v0 + s*x0)/(r - s);
    const T c2 = -(v0 - r*x0)/(r - s);

    // Step 3: Set the solutions.
    *xf = c1*exp(r*tf) + c2*exp(s*tf);
    *vf = r*c1*exp(r*tf) + s*c2*exp(s*tf);
  }

 private:
  // Signum function.
  static T sgn(const T& x) {
    if (x > 0) {
      return 1;
    } else {
      if (x < 0) {
        return -1;
      } else {
        return 0;
      }
    }
  }

  // Solves the quadratic equation ax² + bx + c = 0 for x, returned as a pair
  // of two values. Cancellation error is avoided. Aborts if b is zero.
  static std::pair<T, T> SolveQuadratic(const T& a, const T& b, const T& c) {
    DRAKE_DEMAND(b != 0);
    const T x1 = (-b - sgn(b)*sqrt(b*b - 4*a*c))/(2*a);
    const T x2 = c/(a*x1);
    return std::make_pair(x1, x2);
  }

  double damping_constant_N_per_m_;
};

// This is a modified spring-mass-damper system for which the acceleration
// component of the derivative function is discontinuous with respect to the
// point mass state. Tests the ability of the integrator to deal with
// such discontinuities.
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
    constant_force_(constant_force) {
    DRAKE_ASSERT(constant_force >= 0.0);
  }

  /// Gets the magnitude of the constant force acting on the system.
  double get_constant_force() const { return constant_force_; }

 protected:
  System<AutoDiffXd>* DoToAutoDiffXd() const override {
    return new ModifiedSpringMassDamperSystem<AutoDiffXd>(
            this->get_spring_constant(),
            this->get_damping_constant(),
            this->get_mass(),
            get_constant_force());
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
    // Create the spring-mass systems.
    spring = std::make_unique<SpringMassSystem<double>>(spring_k_,
                                                        mass_,
                                                        false /* no forcing */);

    spring_damper = std::make_unique<SpringMassDamperSystem<double>>(
        stiff_spring_k_, stiff_damping_b_, mass_);
    mod_spring_damper = std::make_unique<
        ModifiedSpringMassDamperSystem<double>>(stiff_spring_k_, damping_b_,
                                                mass_, constant_force_mag_);

    // One context will be usable for three of the systems.
    context = spring->CreateDefaultContext();
  }

  std::unique_ptr<Context<double>> context;
  std::unique_ptr<SpringMassSystem<double>> spring;
  std::unique_ptr<SpringMassDamperSystem<double>> spring_damper;
  std::unique_ptr<ModifiedSpringMassDamperSystem<double>> mod_spring_damper;

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

// Verifies compilation and that trying to use automatic differentiated
// Jacobian with an AutoDiff'd integrator chokes.
TEST_F(ImplicitIntegratorTest, AutoDiff) {
  // Create the integrator for a System<AutoDiffXd>.
  auto system = spring->ToAutoDiffXd();
  auto context = system->CreateDefaultContext();
  ImplicitEulerIntegrator<AutoDiffXd> integrator(*system, context.get());

  // Set reasonable integrator parameters.
  integrator.set_maximum_step_size(large_dt_);
  integrator.request_initial_step_size_target(large_dt_);
  integrator.set_target_accuracy(1e-5);
  integrator.set_minimum_step_size(small_dt_);
  integrator.Initialize();

  // Integrate for one step.
  EXPECT_THROW(integrator.StepExactlyFixed(large_dt_), std::logic_error);

  // TODO(edrumwri): Add test that an automatic differentiation of an implicit
  // integrator produces the expected result.
}

TEST_F(ImplicitIntegratorTest, MiscAPI) {
  // Create the integrator for a System<double>.
  ImplicitEulerIntegrator<double> integrator(*spring, context.get());

  // Verifies that calling Initialize without setting step size target or
  // maximum step size throws exception.
  EXPECT_THROW(integrator.Initialize(), std::logic_error);

  // Verify defaults match documentation.
  EXPECT_EQ(integrator.get_jacobian_computation_scheme(),
            ImplicitEulerIntegrator<double>::JacobianComputationScheme::
                kForwardDifference);
  EXPECT_TRUE(integrator.get_multistep_in_step_exactly_fixed_throws());

  // Test that setting the target accuracy and initial step size target is
  // successful.
  integrator.set_maximum_step_size(dt_);
  integrator.set_target_accuracy(1.0);
  integrator.request_initial_step_size_target(dt_);
  integrator.Initialize();

  // Verifies that setting accuracy too loose (from above) makes the working
  // accuracy different than the target accuracy after initialization.
  EXPECT_NE(integrator.get_accuracy_in_use(), integrator.get_target_accuracy());
}

TEST_F(ImplicitIntegratorTest, FixedStepThrowsOnMultiStep) {
  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k_, mass_,
                                       false /* no forcing */);

  // Set the integrator to operate in fixed step mode and with very tight
  // tolerances.
  ImplicitEulerIntegrator<double> integrator(spring_mass, context.get());
  const double huge_dt = 100.0;
  integrator.set_maximum_step_size(huge_dt);
  integrator.set_fixed_step_mode(true);

  // Use automatic differentiation because we can.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kAutomatic);

  // Set initial condition to something significant.
  spring_mass.set_position(context.get(), 1.0);
  spring_mass.set_velocity(context.get(), -1.0);

  // Take all the defaults.
  integrator.Initialize();

  // Integrate for the desired step size.
  EXPECT_THROW(integrator.StepExactlyFixed(huge_dt), std::runtime_error);
}

TEST_F(ImplicitIntegratorTest, ContextAccess) {
  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*spring, context.get());

  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  EXPECT_THROW(integrator.StepOnceAtMost(dt_, dt_, dt_), std::logic_error);
}

/// Verifies error estimation is unsupported.
TEST_F(ImplicitIntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  ImplicitEulerIntegrator<double> integrator(*spring, context.get());

  EXPECT_EQ(integrator.get_error_estimate_order(), 2);
  EXPECT_EQ(integrator.supports_error_estimation(), true);
  EXPECT_NO_THROW(integrator.set_target_accuracy(1e-1));
  EXPECT_NO_THROW(integrator.request_initial_step_size_target(dt_));
}

// Checks the validity of general integrator statistics and resets statistics.
void CheckGeneralStatsValidity(ImplicitEulerIntegrator<double>* integrator) {
  EXPECT_GT(integrator->get_num_newton_raphson_loops(), 0);
  EXPECT_GT(integrator->get_num_error_estimator_newton_raphson_loops(), 0);
  EXPECT_GT(integrator->get_previous_integration_step_size(), 0.0);
  EXPECT_GT(integrator->get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator->get_num_steps_taken(), 0);
  EXPECT_GT(integrator->get_num_derivative_evaluations(), 0);
  EXPECT_GT(integrator->get_num_error_estimator_derivative_evaluations(), 0);
  EXPECT_GT(integrator->get_num_derivative_evaluations_for_jacobian(), 0);
  EXPECT_GT(integrator->
      get_num_error_estimator_derivative_evaluations_for_jacobian(), 0);
  EXPECT_GE(integrator->get_num_jacobian_evaluations(), 0);
  EXPECT_GE(integrator->get_num_error_estimator_jacobian_evaluations(), 0);
  EXPECT_GE(integrator->get_num_iteration_matrix_factorizations(), 0);
  EXPECT_GE(integrator->
      get_num_error_estimator_iteration_matrix_factorizations(), 0);
  EXPECT_GE(integrator->get_num_substep_failures(), 0);
  EXPECT_GE(integrator->get_num_step_shrinkages_from_substep_failures(), 0);
  EXPECT_GE(integrator->get_num_step_shrinkages_from_error_control(), 0);
  integrator->ResetStatistics();
}

// Integrate the mass-spring-damping system using huge stiffness and damping.
// This equation should be stiff.
TEST_F(ImplicitIntegratorTest, SpringMassDamperStiff) {
  // Create a context.
  auto context = spring_damper->CreateDefaultContext();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*spring_damper, context.get());
  integrator.set_maximum_step_size(large_dt_);
  integrator.set_minimum_step_size(small_dt_);
  integrator.set_minimum_step_size_exceeded_throws(false);

  // Set error controlled integration parameters.
  const double xtol = 1e-6;
  integrator.set_target_accuracy(xtol);

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
  integrator.StepExactlyVariable(t_final);

  // Check the time.
  EXPECT_NEAR(context->get_time(), t_final, ttol);

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
  integrator.StepExactlyVariable(t_final);
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
  integrator.StepExactlyVariable(t_final);
  x_final = xc_final.GetAtIndex(0);
  v_final = xc_final.GetAtIndex(1);

  // Verify that error control was used by making sure that the minimum step
  // size was smaller than large_dt_.
  EXPECT_LT(integrator.get_smallest_adapted_step_size_taken(), large_dt_);

  // Verify that integrator statistics and outputs are valid.
  EXPECT_NEAR(0.0, x_final, xtol);
  EXPECT_NEAR(0.0, v_final, vtol);
  CheckGeneralStatsValidity(&integrator);
}

// Try a purely continuous system with no sampling.
TEST_F(ImplicitIntegratorTest, SpringMassStep) {
  const double spring_k = 300.0;  // N/m

  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass_, false /* no forcing */);

  // Set integrator parameters; we want error control to initially "fail",
  // necessitating step size adjustment.
  ImplicitEulerIntegrator<double> integrator(spring_mass, context.get());
  integrator.set_maximum_step_size(large_dt_);
  integrator.request_initial_step_size_target(large_dt_);
  integrator.set_target_accuracy(5e-5);
  integrator.set_minimum_step_size(1e-6);

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
  integrator.StepExactlyVariable(t_final);

  // Check the time.
  EXPECT_NEAR(context->get_time(), t_final, ttol);

  // Get the final position.
  double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Compute the true solution at t_final.
  double x_final_true, v_final_true;
  spring_mass.get_closed_form_solution(initial_position, initial_velocity,
                                       t_final, &x_final_true, &v_final_true);

  // Check the solution to the same tolerance as the explicit Euler
  // integrator.
  EXPECT_NEAR(x_final_true, x_final, 5e-3);

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
  integrator.StepExactlyVariable(t_final);

  // Check results again.
  x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);
  EXPECT_NEAR(x_final_true, x_final, 5e-3);
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
  integrator.StepExactlyVariable(t_final);

  // Check results again.
  x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);
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
// x'(t) = -c1*sin(omega*t)*omega + c2*cos(omega*t)*omega
// for t = 0, x(0) = c1, x'(0) = c2*omega
TEST_F(ImplicitIntegratorTest, ErrorEstimation) {
  const double spring_k = 300.0;  // N/m

  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass_, false /* no forcing */);

  // Set the integrator to operate in fixed step mode and with very tight
  // tolerances.
  ImplicitEulerIntegrator<double> integrator(spring_mass, context.get());
  integrator.set_maximum_step_size(large_dt_);
  integrator.set_fixed_step_mode(true);
  integrator.set_multistep_in_step_exactly_fixed_throws(false);

  // Use automatic differentiation because we can.
  integrator.set_jacobian_computation_scheme(
      ImplicitEulerIntegrator<double>::JacobianComputationScheme::
      kAutomatic);

  // Create the initial positions and velocities.
  const int n_initial_conditions = 3;
  const double initial_position[n_initial_conditions] = { 0.1, 1.0, 0.0 };
  const double initial_velocity[n_initial_conditions] = { 0.01, 1.0, -10.0 };

  // Create the integration step size array. NOTE: dt values greater than 1e-2
  // (or so) results in very poor error estimates. dt values smaller than 1e-8
  // (or so) results in NaN relative errors (indicating that solution matches
  // ideal one to very high accuracy).
  const int n_dts = 4;
  const double dts[n_dts] = { 1e-8, 1e-4, 1e-3, 1e-2 };

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
      double x_final_true, v_final_true;
      spring_mass.get_closed_form_solution(initial_position[i],
                                           initial_velocity[i],
                                           dts[j], &x_final_true,
                                           &v_final_true);

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
  SpringMassSystem<double> spring_mass(spring_k, mass_, false /* no forcing */);

  // Spring-mass system is necessary only to setup the problem.
  ImplicitEulerIntegrator<double> integrator(spring_mass, context.get());
  integrator.set_maximum_step_size(large_dt_);
  integrator.set_minimum_step_size(small_dt_);
  integrator.set_minimum_step_size_exceeded_throws(false);

  // Turn fixed stepping on.
  integrator.set_fixed_step_mode(true);

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();
  EXPECT_NEAR(integrator.get_accuracy_in_use(), 1e-1,
              std::numeric_limits<double>::epsilon());

  // Get the actual solution.
  double x_final_true, v_final_true;
  spring_mass.get_closed_form_solution(initial_position, initial_velocity,
                                       large_dt_, &x_final_true, &v_final_true);

  // Integrate exactly one step.
  integrator.StepExactlyFixed(large_dt_);

  // Get the positional error.
  const double pos_err = std::abs(x_final_true -
      context->get_continuous_state_vector().GetAtIndex(0));

  // Make the accuracy tolerance looser, integrate again, and verify that
  // positional error increases.
  integrator.set_target_accuracy(100.0);
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);
  integrator.StepExactlyFixed(large_dt_);
  EXPECT_GT(std::abs(x_final_true -
      context->get_continuous_state_vector().GetAtIndex(0)), pos_err);
}

// Integrate the modified mass-spring-damping system, which exhibits a
// discontinuity in the velocity derivative at spring position x = 0.
TEST_F(ImplicitIntegratorTest, ModifiedSpringMassDamper) {
  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*mod_spring_damper,
                                             context.get());
  integrator.set_maximum_step_size(dt_);
  integrator.set_minimum_step_size_exceeded_throws(false);
  integrator.set_multistep_in_step_exactly_fixed_throws(false);

  // Setting the minimum step size speeds the unit test without (in this case)
  // affecting solution accuracy.
  integrator.set_minimum_step_size(1e-3);

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
  const double sol_tol = 1e-5;

  // Integrate for 1 second.
  const double t_final = 1.0;
  integrator.StepExactlyVariable(t_final);

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
  integrator.StepExactlyVariable(t_final);

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
  integrator.StepExactlyVariable(t_final);

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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

