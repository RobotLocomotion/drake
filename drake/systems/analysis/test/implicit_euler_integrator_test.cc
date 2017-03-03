#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator-inl.h"

#include <cmath>

#include "gtest/gtest.h"

#include "drake/examples/rod2d/rod2d.h"
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

class ImplicitIntegratorTest : public ::testing::Test {
 public:
  ImplicitIntegratorTest() {

    // Create three systems.
    spring = std::make_unique<SpringMassSystem<double>>(spring_k,
                                                         mass,
                                                         false /* no forcing */
    );
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
  const double dt = 1e-3;               // Default integration step size.
  const double spring_k = 1.0;          // Default spring constant.
  const double stiff_spring_k = 1e10;   // Default constant for a stiff spring.
  const double damping_b = 1e4;         // Default semi-stiff damper constant.
  const double stiff_damping_b = 1e8;   // Default stiff damper constant.
  const double mass = 2.0;              // Default particle mass.
  const double constant_force_mag = 10; // Magnitude of the constant force.
  const double inf = std::numeric_limits<double>::infinity();
};

TEST_F(ImplicitIntegratorTest, MiscAPI) {
  // Create the integrator as a double and as an autodiff type
  ImplicitEulerIntegrator<double> integrator(*spring, dt, context.get());

  // Test that setting the target accuracy or initial step size target fails.
  EXPECT_THROW(integrator.set_target_accuracy(1.0), std::logic_error);
  EXPECT_THROW(integrator.request_initial_step_size_target(1.0),
               std::logic_error);
}

// TODO(edrumwri): Somehow verify that the integrator statistics were properly
// reset?

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
TEST_F(ImplicitIntegratorTest, SpringMassStep) {
  const double spring_k = 300.0;  // N/m

  // Create a new spring-mass system.
  SpringMassSystem<double> spring_mass(spring_k, mass, false /* no forcing */);

  // Create a new dt. This is actually a larger dt than that used for the same
  // time with the explicit Euler integrator. As these are both first order
  // integrators, we should be able to attain the same accuracy.
  const double dt = 1e-5;

  // Spring-mass system is necessary only to setup the problem.
  ImplicitEulerIntegrator<double> integrator(spring_mass, dt, context.get());

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
  for (t = 0.0; std::abs(t - t_final) >= dt; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check the time. 
  const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(context->get_time(), t, ttol);

  // Get the final position.
  const double x_final =
      context->get_continuous_state()->get_vector().GetAtIndex(0);

  // Check the solution to the same tolerance as the explicit Euler
  // integrator.
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
TEST_F(ImplicitIntegratorTest, SpringMassDamperStiff) {
  // Create a context.
  auto context = spring_damper->CreateDefaultContext();

  // Use a new integration size.
  const double dt = 1e-1;

  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*spring_damper, dt, context.get());

  // Set the initial position and initial velocity.
  const double initial_position = 1;
  const double initial_velocity = 0.1;

  // Set initial condition.
  spring_damper->set_position(context.get(), initial_position);
  spring_damper->set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Integrate for sufficient time for the spring to go to rest. 
  const double t_final = 2.0;
  double t;
  for (t = 0.0; std::abs(t - t_final) >= dt; t = context->get_time())
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));

  // Check the time. 
  const double ttol = 1e2 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(context->get_time(), t, ttol);

  // Get the final position and velocity.
  const VectorBase<double>& xc_final = context->get_continuous_state()->
      get_vector();
  const double x_final = xc_final.GetAtIndex(0);
  const double v_final = xc_final.GetAtIndex(1);

  // Check the solution - the large spring should have driven the spring
  // position to zero, while the large damper should make the velocity
  // essentially zero.
  const double xtol = 1e6 * std::numeric_limits<double>::epsilon();
  const double vtol = xtol * 100;
  EXPECT_NEAR(0.0, x_final, xtol);
  EXPECT_NEAR(0.0, v_final, vtol);

  // Verify that integrator statistics are valid
  EXPECT_GE(integrator.get_previous_integration_step_size(), 0.0);
  EXPECT_GE(integrator.get_largest_step_size_taken(), 0.0);
  EXPECT_GE(integrator.get_num_steps_taken(), 0);
  EXPECT_EQ(integrator.get_error_estimate(), nullptr);
}

// Integrate the mass-spring-damping system with a constant force.
TEST_F(ImplicitIntegratorTest, ModifiedSpringMassDamper) {
  // Create the integrator.
  ImplicitEulerIntegrator<double> integrator(*mod_spring_damper,
                                             dt,
                                             context.get());
  integrator.set_convergence_tolerance(1e-8);

  // Set the initial position and initial velocity.
  const double initial_position = 1e-8;
  const double initial_velocity = 0;

  // Set initial condition.
  mod_spring_damper->set_position(context.get(), initial_position);
  mod_spring_damper->set_velocity(context.get(), initial_velocity);

  // Take all the defaults.
  integrator.Initialize();

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
//  int last_obj_inc = 0;
  for (t = 0.0; std::abs(t - t_final) >= dt; t = context->get_time()) {
//    const VectorBase<double>& xc = context->get_continuous_state_vector();
//   std::cout << (integrator.get_num_objective_function_increases() - last_obj_inc) << " objective function increases for xc=" << xc.GetAtIndex(0) << ", " << xc.GetAtIndex(1) << " at " << context->get_time() << std::endl;
//    last_obj_inc = integrator.get_num_objective_function_increases();
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));
  }

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
}

// Integrates the two-dimensional rod system.
GTEST_TEST(ImplicitIntegratorGTest, Rod2d) {
  const double inf = std::numeric_limits<double>::infinity();
  examples::rod2d::Rod2D<double> rod(
      examples::rod2d::Rod2D<double>::SimulationType::kCompliant,
      0.0 /* no time stepping */);

  // Make the system stiff.
  rod.set_mu_coulomb(1);
  rod.set_stiffness(1e8);
  rod.set_dissipation(1e3);
  rod.set_stiction_speed_tolerance(1e-4);
  rod.set_mu_static(1.5);

  // Create the context.
  auto context = rod.CreateDefaultContext();

  // Set a zero input force (this is the default).
  std::unique_ptr<systems::BasicVector<double>> ext_input =
      std::make_unique<systems::BasicVector<double>>(3);
  ext_input->SetAtIndex(0, 0.0);
  ext_input->SetAtIndex(1, 0.0);
  ext_input->SetAtIndex(2, 0.0);
  context->FixInputPort(0, std::move(ext_input));

  // Use a relatively large step size.
  const double dt = 1e-2;

  // Create and initialize the integrator.
  ImplicitEulerIntegrator<double> integrator(rod, dt, context.get());
  integrator.Initialize();
  integrator.set_convergence_tolerance(1e-3);

  // Integrate for 1 second.
  const double t_final = 1.0;
  double t;
//  int last_obj_inc = 0;
  for (t = 0.0; std::abs(t - t_final) >= dt; t = context->get_time()) {
    const VectorBase<double>& xc = context->get_continuous_state_vector();
    std::cerr << t << " " << xc.CopyToVector().transpose() << std::endl;
//   std::cout << (integrator.get_num_objective_function_increases() - last_obj_inc) << " objective function increases for xc=" << xc.GetAtIndex(0) << ", " << xc.GetAtIndex(1) << " at " << context->get_time() << std::endl;
//    last_obj_inc = integrator.get_num_objective_function_increases();
    integrator.StepOnceAtMost(inf, inf, std::min(t_final - t, dt));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
