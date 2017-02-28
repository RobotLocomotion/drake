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
  SpringMassDamperSystem(const T& spring_constant_N_per_m,
                         const T& damping_constant_N_per_m,
                         const T& mass_kg) : 
    SpringMassSystem<T>(spring_constant_N_per_m, mass_kg, false /* unforced */),
    damping_constant_N_per_m_(damping_constant_N_per_m) { }

  /// Returns the damping constant that was provided at construction in N/m
  const T& get_damping_constant() const { return damping_constant_N_per_m_; }

 protected:
  void DoCalcTimeDerivatives(const Context<T>& context, ContinuousState<T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T xd = state[1];
    (*derivatives)[0] = xd;

    // Compute the force acting on the mass. There is always a constant
    // force pushing the mass toward -inf. The spring and damping forces are
    // only active when x <= 0; the spring setpoint is x = 0.
    const T k = this->get_spring_constant();
    const T b = get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    T force = -k * (x - x0) - b * xd;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass(); 
  }

 private:
   T damping_constant_N_per_m_;
};

// This is a modified spring-mass-damper system that is only active in a small
// part of state space.
template <class T>
class ModifiedSpringMassDamperSystem : public SpringMassDamperSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModifiedSpringMassDamperSystem);
  ModifiedSpringMassDamperSystem(const T& spring_constant_N_per_m,
                                 const T& damping_constant_N_per_m,
                                 const T& mass_kg,
                                 const T& constant_force) : 
    SpringMassDamperSystem<T>(spring_constant_N_per_m, 
                              damping_constant_N_per_m,
                              mass_kg),
    constant_force_(constant_force) { }

  /// Gets the magnitude of the constant force acting on the system.
  const T& get_constant_force() const { return constant_force_; }

 protected:
  void DoCalcTimeDerivatives(const Context<T>& context, ContinuousState<T>* derivatives) const override {
    // Get the current state of the spring.
    const ContinuousState<T>& state = *context.get_continuous_state();

    // First element of the derivative is spring velocity.
    const T xd = state[1];
    (*derivatives)[0] = xd;

    // Compute the force acting on the mass. There is always a constant
    // force pushing the mass toward -inf. The spring and damping forces are
    // only active when x <= 0; the spring setpoint is x = 0.
    T force = -constant_force_;
    const T k = this->get_spring_constant();
    const T b = this->get_damping_constant();
    const T x0 = 0;
    const T x = state[0];
    if (x <= x0) 
      force -= k * (x - x0) + b * xd;

    // Second element of the derivative is spring acceleration.
    (*derivatives)[1] = force / this->get_mass(); 
  }

 private:
   T constant_force_;
};

GTEST_TEST(IntegratorTest, MiscAPI) {
  SpringMassSystem<double> spring_mass_dbl(1., 1., 0.);

  // Setup the integration step size.
  const double dt = 1e-3;

  // Create a context.
  auto context_dbl = spring_mass_dbl.CreateDefaultContext();

  // Create the integrator as a double and as an autodiff type
  ImplicitEulerIntegrator<double> int_dbl(spring_mass_dbl, dt,
                                          context_dbl.get());

  // Test that setting the target accuracy or initial step size target fails.
  EXPECT_THROW(int_dbl.set_target_accuracy(1.0), std::logic_error);
  EXPECT_THROW(int_dbl.request_initial_step_size_target(1.0), std::logic_error);
}

// TODO(edrumwri): Somehow verify that the integrator statistics were properly
// reset?

GTEST_TEST(IntegratorTest, ContextAccess) {
  // Create the mass spring system.
  SpringMassSystem<double> spring_mass(1., 1., 0.);

  // Setup the integration step size.
  const double dt = 1e-3;

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  integrator.get_mutable_context()->set_time(3.);
  EXPECT_EQ(integrator.get_context().get_time(), 3.);
  EXPECT_EQ(context->get_time(), 3.);\
  integrator.reset_context(nullptr);
  EXPECT_THROW(integrator.Initialize(), std::logic_error);
  EXPECT_THROW(integrator.StepOnceAtMost(dt, dt, dt), std::logic_error);
}

/// Verifies error estimation is unsupported.
GTEST_TEST(IntegratorTest, AccuracyEstAndErrorControl) {
  // Spring-mass system is necessary only to setup the problem.
  SpringMassSystem<double> spring_mass(1., 1., 0.);
  const double dt = 1e-3;
  auto context = spring_mass.CreateDefaultContext();
  ImplicitEulerIntegrator<double> integrator(
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
  const double spring_k = 300.0;  // N/m
  const double mass = 2.0;      // kg

  // Create the spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass, 0.);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Setup the integration size and infinity.
  const double dt = 1e-2;
  const double inf = std::numeric_limits<double>::infinity();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  // TODO(edrumwri): consider making the convergence tolerance a parameter
  // to the constructor in ImplicitEulerIntegrator.

  // Setup the initial position and initial velocity.
  const double initial_position = 0.1;
  const double initial_velocity = 0.01;
  const double omega = std::sqrt(spring_k / mass);

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);

  // Take all the defaults.
  integrator.Initialize();

  // Setup c1 and c2 for ODE constants.
  const double c1 = initial_position;
  const double c2 = initial_velocity / omega;

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) > dt; t = context->get_time()) 
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // TODO(edrumwri): Tighten this test.
  EXPECT_NEAR(context->get_time(), t, dt);

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  EXPECT_NEAR(c1 * std::cos(omega * t) + c2 * std::sin(omega * t), x_final,
              5e-3);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

// Integrate the mass-spring-damping system using huge stiffness and damping.
// This equation should be stiff. 
GTEST_TEST(IntegratorTest, SpringMassDamperStiff) {
  const double spring_k = 1e10;  // N/m
  const double damping_b = 1e8;   // N/m
  const double mass = 2.0;      // kg

  // Create the spring-mass-damping system.
  SpringMassDamperSystem<double> spring_mass(spring_k, damping_b, mass);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Set the integration size and infinity.
  const double dt = 1e-1;
  const double inf = std::numeric_limits<double>::infinity();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  // TODO(edrumwri): consider making the convergence tolerance a parameter
  // to the constructor in ImplicitEulerIntegrator.

  // Set the initial position and initial velocity.
  const double initial_position = 1;
  const double initial_velocity = 0.1;

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) > dt; t = context->get_time()) 
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // TODO(edrumwri): Tighten this test.
  EXPECT_NEAR(context->get_time(), t, dt);

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  const double tol = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, x_final, tol);

std::cout << "Number of spurious gradient encounters: " << integrator.get_num_misdirected_descents() << std::endl;
std::cout << "Number of objective function increases: " << integrator.get_num_objective_function_increases() << std::endl;
  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

// Integrate the mass-spring-damping system with a constant force. 
GTEST_TEST(IntegratorTest, ModifiedSpringMassDamper) {
  const double spring_k = 1e4;  // N/m
  const double damping_b = 1e2;   // N/m
  const double mass = 2.0;      // kg

  // Create the modified spring-mass-damping system.
  const double steady_force = 10.0;
  ModifiedSpringMassDamperSystem<double> spring_mass(spring_k,
                                                     damping_b,
                                                     mass,
                                                     steady_force);

  // Create a context.
  auto context = spring_mass.CreateDefaultContext();

  // Set the integration size and infinity.
  const double dt = 1e-3;
  const double inf = std::numeric_limits<double>::infinity();

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(
      spring_mass, dt, context.get());  // Use default Context.

  // TODO(edrumwri): consider making the convergence tolerance a parameter
  // to the constructor in ImplicitEulerIntegrator.
  integrator.set_convergence_tolerance(1e-5);  

  // Set the initial position and initial velocity.
  const double initial_position = 1;
  const double initial_velocity = 0;

  // Set initial condition.
  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) > dt; t = context->get_time()) 
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // TODO(edrumwri): Tighten this test.
  EXPECT_NEAR(context->get_time(), t, dt);

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution.
  const double tol = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, x_final, tol);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
std::cout << "Number of spurious gradient encounters: " << integrator.get_num_misdirected_descents() << std::endl;
std::cout << "Number of objective function increases: " << integrator.get_num_objective_function_increases() << std::endl;
}


}  // namespace
}  // namespace systems
}  // namespace drake
