#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Test Velocity-Implicit Euler integrator on common implicit tests.
typedef ::testing::Types<VelocityImplicitEulerIntegrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test

}  // namespace systems
}  // namespace drake
