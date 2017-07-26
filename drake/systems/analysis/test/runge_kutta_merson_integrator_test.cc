#include "drake/systems/analysis/runge_kutta_merson_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/test/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

typedef ::testing::Types<RungeKuttaMersonIntegrator<double>> Types;
INSTANTIATE_TYPED_TEST_CASE_P(My, ExplicitErrorControlledIntegratorTest, Types);

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
