#include "drake/systems/analysis/simulator_config_functions.h"

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

using drake::yaml::LoadYamlString;
using drake::yaml::SaveYamlString;

namespace drake {
namespace systems {
namespace {

template <typename T>
class SimulatorConfigFunctionsTest : public ::testing::Test {};
using MyTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(SimulatorConfigFunctionsTest, MyTypes);

TYPED_TEST(SimulatorConfigFunctionsTest, ResetIntegratorTest) {
  using T = TypeParam;
  ConstantVectorSource<T> source(2);
  Simulator<T> simulator(source);
  const void* prior_integrator = &simulator.get_integrator();
  IntegratorBase<T>& result =
      ResetIntegratorFromFlags(&simulator, "runge_kutta2", T(0.001));
  EXPECT_NE(&simulator.get_integrator(), prior_integrator);
  EXPECT_EQ(&simulator.get_integrator(), &result);
  const std::string result_name_expected = fmt::format(
      "drake::systems::RungeKutta2Integrator<{}>", NiceTypeName::Get<T>());
  EXPECT_EQ(NiceTypeName::Get(result), result_name_expected);
}

TYPED_TEST(SimulatorConfigFunctionsTest, GetSchemes) {
  using T = TypeParam;

  const std::vector<std::string>& schemes = GetIntegrationSchemes();
  EXPECT_GE(schemes.size(), 5);

  // Check that all of the schemes are actually valid.
  ConstantVectorSource<T> source(2);
  Simulator<T> simulator(source);
  for (const auto& one_scheme : schemes) {
    DRAKE_EXPECT_NO_THROW(
        ResetIntegratorFromFlags(&simulator, one_scheme, T(0.001)));
  }
}

template <typename T>
class DummySystem final : public drake::systems::LeafSystem<T> {
 public:
  DummySystem() {}
};

TYPED_TEST(SimulatorConfigFunctionsTest, CongruenceTest) {
  // Ensure that a default constructed SimulatorConfig has the same values as a
  // default constructed Simulator.
  // N.B. Due to the RoundTripTest, we know that ExtractSimulatorConfig is doing
  // actual work, not just returning a default constructed SimulatorConfig.
  using T = TypeParam;

  const DummySystem<T> dummy;
  Simulator<T> simulator(dummy);

  const SimulatorConfig config_defaults;
  const SimulatorConfig sim_defaults = ExtractSimulatorConfig(simulator);
  EXPECT_EQ(sim_defaults.integration_scheme,
            config_defaults.integration_scheme);
  EXPECT_EQ(sim_defaults.max_step_size, config_defaults.max_step_size);
  EXPECT_EQ(sim_defaults.accuracy, config_defaults.accuracy);
  EXPECT_EQ(sim_defaults.use_error_control, config_defaults.use_error_control);
  EXPECT_EQ(sim_defaults.target_realtime_rate,
            config_defaults.target_realtime_rate);
  EXPECT_EQ(sim_defaults.publish_every_time_step,
            config_defaults.publish_every_time_step);
}

TYPED_TEST(SimulatorConfigFunctionsTest, RoundTripTest) {
  using T = TypeParam;
  const std::string bespoke_str = "integration_scheme: runge_kutta5\n"
                                  "max_step_size: 0.003\n"
                                  "accuracy: 0.03\n"
                                  "use_error_control: true\n"
                                  "target_realtime_rate: 3.0\n"
                                  "publish_every_time_step: true\n";

  // Ensure that the string and the struct have the same fields.
  const auto bespoke = LoadYamlString<SimulatorConfig>(bespoke_str);

  // Ensure that a round trip through the archive process does not corrupt data.
  const std::string readback_str = SaveYamlString(bespoke);
  EXPECT_EQ(bespoke_str, readback_str);

  // Ensure that a roundtrip through the Accept and Extract functions does not
  // corrupt data.
  const DummySystem<T> dummy;
  Simulator<T> simulator(dummy);
  ApplySimulatorConfig(&simulator, bespoke);
  const SimulatorConfig readback = ExtractSimulatorConfig(simulator);
  EXPECT_EQ(readback.integration_scheme, bespoke.integration_scheme);
  EXPECT_EQ(readback.max_step_size, bespoke.max_step_size);
  EXPECT_EQ(readback.accuracy, bespoke.accuracy);
  EXPECT_EQ(readback.use_error_control, bespoke.use_error_control);
  EXPECT_EQ(readback.target_realtime_rate, bespoke.target_realtime_rate);
  EXPECT_EQ(readback.publish_every_time_step, bespoke.publish_every_time_step);
}

}  // namespace
}  // namespace systems
}  // namespace drake
