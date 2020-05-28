#include "drake/systems/analysis/simulator_flags.h"

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(SimulatorFlagsTest, ResetIntegrator) {
  ConstantVectorSource<double> source(2);
  Simulator<double> simulator(source);
  const void* prior_integrator = &simulator.get_integrator();
  IntegratorBase<double>& result = ResetIntegratorFromFlags(
      &simulator, "runge_kutta2", 0.001);
  EXPECT_NE(&simulator.get_integrator(), prior_integrator);
  EXPECT_EQ(&simulator.get_integrator(), &result);
  EXPECT_EQ(NiceTypeName::Get(result),
            "drake::systems::RungeKutta2Integrator<double>");
}

GTEST_TEST(SimulatorFlagsTest, GetSchemes) {
  const std::vector<std::string>& schemes = GetIntegrationSchemes();
  EXPECT_GE(schemes.size(), 5);

  // Check that all of the schemes are actually valid.
  ConstantVectorSource<double> source(2);
  Simulator<double> simulator(source);
  for (const auto& one_scheme : schemes) {
    ResetIntegratorFromFlags(&simulator, one_scheme, 0.001);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
