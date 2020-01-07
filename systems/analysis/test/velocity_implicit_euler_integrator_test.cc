#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/test_utilities/cubic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/discontinuous_spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/implicit_integrator_test.h"
#include "drake/systems/analysis/test_utilities/linear_scalar_system.h"
#include "drake/systems/analysis/test_utilities/quadratic_scalar_system.h"
#include "drake/systems/analysis/test_utilities/robertson_system.h"
#include "drake/systems/analysis/test_utilities/spring_mass_damper_system.h"
#include "drake/systems/analysis/test_utilities/stationary_system.h"
#include "drake/systems/analysis/test_utilities/stiff_double_mass_spring_system.h"
#include "drake/systems/plants/spring_mass_system/spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

// Test Velocity-Implicit Euler integrator on common implicit tests.
typedef ::testing::Types<VelocityImplicitEulerIntegrator<double>> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, ImplicitIntegratorTest, MyTypes);

}  // namespace analysis_test

}  // namespace systems
}  // namespace drake
