#include <memory>
#include <optional>
#include <string>
#include <variant>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/cenic/cenic_integrator_parameters.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/analysis/simulator_config_functions.h"

namespace drake {
namespace systems {

using multibody::CenicIntegrator;
using multibody::CenicIntegratorParameters;
using planning::RobotDiagramBuilder;

GTEST_TEST(SimulatorConfigFunctionsCenicTest, ExtractSimulatorConfig) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  ApplySimulatorConfig(
      SimulatorConfig{
          .integration_scheme = "cenic",
      },
      &simulator);
  auto readback = ExtractSimulatorConfig(simulator);
  EXPECT_EQ(readback.integration_scheme, "cenic");
}

GTEST_TEST(SimulatorConfigFunctionsCenicTest, IntegratorParams) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  auto diagram = builder.Build();
  CenicIntegratorParameters params;
  params.explicit_external_inputs = true;
  params.icf_solver.min_tolerance = 1e-6;
  const SimulatorConfig config{
      .integration_scheme = "cenic",
      .use_error_control = false,
      .integrator_params = params,
  };

  // Apply and extract.
  Simulator<double> simulator(*diagram);
  ApplySimulatorConfig(config, &simulator);
  const auto& cenic =
      dynamic_cast<const CenicIntegrator<double>&>(simulator.get_integrator());
  EXPECT_TRUE(cenic.get_explicit_external_inputs());
  EXPECT_EQ(cenic.get_solver_parameters().min_tolerance, 1e-6);
  const auto readback = ExtractSimulatorConfig(simulator);
  const auto* readback_params =
      std::get_if<CenicIntegratorParameters>(&readback.integrator_params);
  ASSERT_NE(readback_params, nullptr);
  EXPECT_TRUE(readback_params->explicit_external_inputs);
  EXPECT_EQ(readback_params->icf_solver.min_tolerance, 1e-6);

  // Create.
  auto created = CreateIntegratorFromConfig(diagram.get(), config);
  EXPECT_TRUE(dynamic_cast<const CenicIntegrator<double>&>(*created)
                  .get_explicit_external_inputs());

  // Leaving the parameters unset uses the integrator's defaults.
  ApplySimulatorConfig(SimulatorConfig{.integration_scheme = "cenic"},
                       &simulator);
  const auto defaults = ExtractSimulatorConfig(simulator);
  const auto* default_params =
      std::get_if<CenicIntegratorParameters>(&defaults.integrator_params);
  ASSERT_NE(default_params, nullptr);
  EXPECT_FALSE(default_params->explicit_external_inputs);
}

GTEST_TEST(SimulatorConfigFunctionsCenicTest, IntegratorParamsMismatch) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  auto diagram = builder.Build();
  const SimulatorConfig config{
      .integration_scheme = "runge_kutta3",
      .integrator_params = CenicIntegratorParameters{},
  };
  Simulator<double> simulator(*diagram);
  const char* const expected_error =
      ".*CenicIntegratorParameters.*scheme is 'runge_kutta3'.*";
  DRAKE_EXPECT_THROWS_MESSAGE(ApplySimulatorConfig(config, &simulator),
                              expected_error);
  DRAKE_EXPECT_THROWS_MESSAGE(CreateIntegratorFromConfig(diagram.get(), config),
                              expected_error);
}

GTEST_TEST(SimulatorConfigFunctionsCenicTest, IntegratorParamsYaml) {
  const auto load = [](const std::string& yaml) {
    return yaml::LoadYamlString<SimulatorConfig>(
        yaml, /* child_name = */ std::nullopt, /* defaults = */ std::nullopt,
        yaml::LoadYamlOptions{.allow_cpp_with_no_yaml = true});
  };

  // A CENIC-tagged value loads, including nested ICF parameters.
  const SimulatorConfig tagged = load(R"""(
integration_scheme: cenic
use_error_control: false
integrator_params: !CenicIntegratorParameters
  explicit_external_inputs: true
  icf_solver:
    min_tolerance: 1e-6
)""");
  const auto* params =
      std::get_if<CenicIntegratorParameters>(&tagged.integrator_params);
  ASSERT_NE(params, nullptr);
  EXPECT_TRUE(params->explicit_external_inputs);
  EXPECT_EQ(params->icf_solver.max_iterations, 100);  // Unmentioned => default.

  // Null, or absent, means unset.
  for (const char* yaml : {"integrator_params:", "integrator_params: null",
                           "max_step_size: 0.2"}) {
    EXPECT_TRUE(
        std::holds_alternative<std::monostate>(load(yaml).integrator_params))
        << yaml;
  }

  // The YAML round trips.
  const SimulatorConfig roundtrip = load(yaml::SaveYamlString(tagged));
  EXPECT_TRUE(std::get<CenicIntegratorParameters>(roundtrip.integrator_params)
                  .explicit_external_inputs);
  EXPECT_TRUE(std::holds_alternative<std::monostate>(
      load(yaml::SaveYamlString(SimulatorConfig{})).integrator_params));
}

GTEST_TEST(SimulatorConfigFunctionsCenicTest, CreateIntegratorFromConfig) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  auto diagram = builder.Build();
  std::unique_ptr<IntegratorBase<double>> dut = CreateIntegratorFromConfig(
      diagram.get(), SimulatorConfig{
                         .integration_scheme = "cenic",
                     });
  EXPECT_NO_THROW(unused(dynamic_cast<const CenicIntegrator<double>&>(*dut)));
}

}  // namespace systems
}  // namespace drake
