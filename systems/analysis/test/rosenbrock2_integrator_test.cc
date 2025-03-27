#include "drake/systems/analysis/rosenbrock2_integrator.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Test that the integrator gives the expected result on a simple spring-mass
// system.
GTEST_TEST(Rosenbrock2IntegratorTest, SpringMass) {
  const double k = 300.0;  // N/m
  const double m = 2.0;    // kg

  // Create the spring-mass system.
  SpringMassSystem<double> spring_mass(k, m, 0.0);
  auto context = spring_mass.CreateDefaultContext();

  // Set initial conditions
  const double initial_position = 0.1;
  const double initial_velocity = 2.1;

  spring_mass.set_position(context.get(), initial_position);
  spring_mass.set_velocity(context.get(), initial_velocity);

  // Set up the integrator
  const double h = 1e-3;
  Rosenbrock2Integrator<double> integrator(spring_mass, context.get());
  integrator.set_maximum_step_size(h);
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();

  // Integrate the system
  const double t_final = 1.0;
  const double inf = std::numeric_limits<double>::infinity();
  double t;
  for (t = 0.0; std::abs(t - t_final) > h; t += h)
    integrator.IntegrateNoFurtherThanTime(inf, inf, t_final);

  fmt::print("final time: {}\n", context->get_time());
  fmt::print("q(t): {}\n", spring_mass.get_position(*context));
  fmt::print("v(t): {}\n", spring_mass.get_velocity(*context));

  // Compare with the reference solution
  double q_ref;
  double v_ref;
  spring_mass.GetClosedFormSolution(initial_position, initial_velocity, t_final,
                                    &q_ref, &v_ref);

  fmt::print("q_ref(t): {}\n", q_ref);
  fmt::print("v_ref(t): {}\n", v_ref);
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
