#include "drake/systems/analysis/implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Test Euler integrator.
typedef ::testing::Types<ImplicitEulerIntegrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

