#include "drake/systems/analysis/simulator_config_functions.h"

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

using drake::yaml::YamlReadArchive;
using drake::yaml::YamlWriteArchive;

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

class DummySystem final : public drake::systems::LeafSystem<double> {
 public:
  DummySystem() {}
};

GTEST_TEST(SimulatorConfigFunctionsTest, ExtractDefaultsTest) {
  const DummySystem dummy;
  Simulator<double> simulator(dummy);

  // We can always extract the defaults.
  const SimulatorConfig defaults = ExtractSimulatorConfig(simulator);
  EXPECT_EQ(defaults.integration_scheme, "runge_kutta3");
  // TODO(jeremy.nimmer) The integrators return incorrect defaults for
  // max_step_size and accuracy.  Once Drake is fixed, we should EXPECT_EQ
  // instead here.
  EXPECT_GT(defaults.max_step_size, 0);
  EXPECT_GT(defaults.accuracy, 0);
  EXPECT_EQ(defaults.use_error_control, true);
  EXPECT_EQ(defaults.target_realtime_rate, 0.0);
  EXPECT_EQ(defaults.publish_every_time_step, false);
}

GTEST_TEST(SimulatorConfigFunctionsTest, RoundTripTest) {
  const std::string bespoke_str = "integration_scheme: runge_kutta5\n"
                                  "max_step_size: 0.003\n"
                                  "accuracy: 0.03\n"
                                  "use_error_control: true\n"
                                  "target_realtime_rate: 3.0\n"
                                  "publish_every_time_step: true\n";
  const YAML::Node bespoke_yaml = YAML::Load(bespoke_str);

  // Ensure that the string and the struct have the same fields.
  YamlReadArchive::Options options;
  options.allow_cpp_with_no_yaml = false;
  options.allow_yaml_with_no_cpp = false;
  SimulatorConfig bespoke;
  YamlReadArchive(bespoke_yaml, options).Accept(&bespoke);

  // Ensure that a round trip through the archive process does not corrupt data.
  YamlWriteArchive archive;
  archive.Accept(bespoke);
  const std::string readback_str = archive.EmitString("");
  EXPECT_EQ(bespoke_str, readback_str);

  // Ensure that a roundtrip through the Accept and Extract functions does not
  // corrupt data.
  const DummySystem dummy;
  Simulator<double> simulator(dummy);
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
