#include <memory>

#include <gtest/gtest.h>

#include "drake/common/unused.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/analysis/simulator_config_functions.h"

namespace drake {
namespace systems {

using multibody::CenicIntegrator;
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

GTEST_TEST(SimulatorConfigFunctionsCenicTest, ExplicitExternalInputs) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  auto diagram = builder.Build();
  const SimulatorConfig config{
      .integration_scheme = "cenic",
      .use_error_control = false,
      .cenic_explicit_external_inputs = true,
  };

  // Apply and extract.
  Simulator<double> simulator(*diagram);
  ApplySimulatorConfig(config, &simulator);
  const auto& cenic =
      dynamic_cast<const CenicIntegrator<double>&>(simulator.get_integrator());
  EXPECT_TRUE(cenic.get_solver_parameters().explicit_external_inputs);
  EXPECT_TRUE(ExtractSimulatorConfig(simulator).cenic_explicit_external_inputs);

  // Create.
  auto created = CreateIntegratorFromConfig(diagram.get(), config);
  EXPECT_TRUE(dynamic_cast<const CenicIntegrator<double>&>(*created)
                  .get_solver_parameters()
                  .explicit_external_inputs);

  // The default is off.
  ApplySimulatorConfig(SimulatorConfig{.integration_scheme = "cenic"},
                       &simulator);
  EXPECT_FALSE(
      ExtractSimulatorConfig(simulator).cenic_explicit_external_inputs);
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
