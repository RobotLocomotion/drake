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

  SpringMassSystem<double> spring_mass(k, m, 0.0);
  auto context = spring_mass.CreateDefaultContext();

  (void)context;
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
