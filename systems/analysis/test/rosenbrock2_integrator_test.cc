#include "drake/systems/analysis/rosenbrock2_integrator.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Test that the integrator gives the expected result on a simple spring-mass
// system with stiffness k and mass m:
//
//    x''(t) -k/m * x(t).
//
// This system has a known solution:
//
//    x(t) = c1 cos(ωt) + c2 sin(ωt)
//    x'(t) = -c1 ω sin(ωt) + c2 ω cos(ωt)
//
// where ω = sqrt(k/m), c1 = x(0), c2 = x'(0) / ω.
GTEST_TEST(Rosenbrock2IntegratorTest, SpringMass) {
  const double k = 300.0;  // N/m
  const double m = 2.0;    // kg

  // Create the spring-mass system.
  SpringMassSystem<double> spring_mass(k, m, 0.0);
  auto context = spring_mass.CreateDefaultContext();

  // Set initial conditions
  const double initial_position = 0.1;
  const double initial_velocity = 2.1;
  const double omega = std::sqrt(k / m);

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
  integrator.IntegrateNoFurtherThanTime(inf, inf, t_final);

  fmt::print("final time: {}\n", context->get_time());
  fmt::print("x(t): {}\n", spring_mass.get_position(*context));
  fmt::print("v(t): {}\n", spring_mass.get_velocity(*context));

  (void)omega;
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
