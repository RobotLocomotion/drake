#include "drake/multibody/cenic/make_cenic_integrator.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace multibody {
namespace cenic {
namespace {

using planning::RobotDiagram;
using planning::RobotDiagramBuilder;
using systems::ConstantVectorSource;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::IntegratorBase;

GTEST_TEST(MakeCenicIntegratorTest, SuccessTrivial) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  std::unique_ptr<RobotDiagram<double>> robot_diagram = builder.Build();

  std::unique_ptr<IntegratorBase<double>> dut =
      MakeCenicIntegrator(*robot_diagram);
  auto& cenic = dynamic_cast<CenicIntegrator<double>&>(*dut);

  // The CENIC plant is the same object as the robot diagram's plant.
  const auto& dut_plant = cenic.plant();
  const auto& diagram_plant = robot_diagram->plant();
  EXPECT_EQ(&dut_plant, &diagram_plant);
}

GTEST_TEST(MakeCenicIntegratorTest, SuccessNested) {
  RobotDiagramBuilder<double> builder{/* time_step = */ 0.0};
  std::unique_ptr<RobotDiagram<double>> robot_diagram = builder.Build();
  const auto& diagram_plant = robot_diagram->plant();

  DiagramBuilder<double> root_builder;
  root_builder.AddSystem(std::move(robot_diagram));
  const auto root_diagram = root_builder.Build();

  std::unique_ptr<IntegratorBase<double>> dut =
      MakeCenicIntegrator(*root_diagram);
  auto& cenic = dynamic_cast<CenicIntegrator<double>&>(*dut);

  // The CENIC plant is the same object as the robot diagram's plant.
  const auto& dut_plant = cenic.plant();
  EXPECT_EQ(&dut_plant, &diagram_plant);
}

GTEST_TEST(MakeCenicIntegratorTest, SuccessUnnested) {
  MultibodyPlant<double> plant(0.0);
  plant.Finalize();

  std::unique_ptr<IntegratorBase<double>> dut = MakeCenicIntegrator(plant);
  auto& cenic = dynamic_cast<CenicIntegrator<double>&>(*dut);

  // The CENIC plant is the same object as the offered plant.
  const auto& dut_plant = cenic.plant();
  EXPECT_EQ(&dut_plant, &plant);
}

GTEST_TEST(MakeCenicIntegratorTest, FailureDiscrete) {
  RobotDiagramBuilder<double> builder;
  std::unique_ptr<RobotDiagram<double>> robot_diagram = builder.Build();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeCenicIntegrator(*robot_diagram),
      ".*requires.*continuous.*found.*time_step=0.001.*");
}

GTEST_TEST(MakeCenicIntegratorTest, FailureNoDiagramNoPlant) {
  ConstantVectorSource<double> system(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeCenicIntegrator(system),
                              ".*zero.*continuous.*");
}

GTEST_TEST(MakeCenicIntegratorTest, FailureDiagramNoPlant) {
  DiagramBuilder<double> builder;
  builder.AddSystem<ConstantVectorSource>(0.0);
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  DRAKE_EXPECT_THROWS_MESSAGE(MakeCenicIntegrator(*diagram),
                              ".*zero.*continuous.*");
}

GTEST_TEST(MakeCenicIntegratorTest, SuccessNoSceneGraph) {
  DiagramBuilder<double> builder;
  builder.AddSystem<ConstantVectorSource>(0.0);
  auto plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
  plant->Finalize();
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  std::unique_ptr<IntegratorBase<double>> dut = MakeCenicIntegrator(*diagram);
  auto& cenic = dynamic_cast<CenicIntegrator<double>&>(*dut);

  // The CENIC plant is the same object as the diagram's plant.
  const auto& dut_plant = cenic.plant();
  EXPECT_EQ(&dut_plant, plant);
}

GTEST_TEST(MakeCenicIntegratorTest, FailureExtraPlant) {
  DiagramBuilder<double> builder;
  RobotDiagramBuilder<double> robot_builder{/* time_step = */ 0.0};
  builder.AddSystem(robot_builder.Build());
  auto result = AddMultibodyPlantSceneGraph(&builder, 0.0);
  result.plant.Finalize();
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  DRAKE_EXPECT_THROWS_MESSAGE(MakeCenicIntegrator(*diagram),
                              ".*more.*continuous.*");
}

GTEST_TEST(MakeCenicIntegratorTest, SuccessExtraDiscretePlant) {
  DiagramBuilder<double> builder;
  RobotDiagramBuilder<double> robot_builder{/* time_step = */ 0.0};
  auto robot_diagram = builder.AddSystem(robot_builder.Build());
  auto result = AddMultibodyPlantSceneGraph(&builder, 0.01);
  result.plant.Finalize();
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  std::unique_ptr<IntegratorBase<double>> dut =
      MakeCenicIntegrator(*robot_diagram);
  auto& cenic = dynamic_cast<CenicIntegrator<double>&>(*dut);

  // The CENIC plant is the same object as the robot diagram's plant.
  const auto& dut_plant = cenic.plant();
  const auto& diagram_plant = robot_diagram->plant();
  EXPECT_EQ(&dut_plant, &diagram_plant);
}

}  // namespace
}  // namespace cenic
}  // namespace multibody
}  // namespace drake
