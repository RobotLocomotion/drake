#include "drake/multibody/cenic/make_cenic_integrator.h"

#include <memory>

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

GTEST_TEST(MakeCenicIntegratorTest, Success) {
  RobotDiagramBuilder<double> builder;
  std::unique_ptr<RobotDiagram<double>> robot_diagram = builder.Build();

  std::unique_ptr<IntegratorBase<double>> dut =
      MakeCenicIntegrator(*robot_diagram);
  auto& cenic = dynamic_cast<CenicIntegrator<double>&>(*dut);

  // The CENIC plant is the same object as the robot diagram's plant.
  const auto& dut_plant = cenic.plant();
  const auto& diagram_plant = robot_diagram->plant();
  EXPECT_EQ(&dut_plant, &diagram_plant);
}

GTEST_TEST(MakeCenicIntegratorTest, FailureNonDiagram) {
  ConstantVectorSource<double> system(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(MakeCenicIntegrator(system),
                              ".*must be given a Diagram.*");
}

GTEST_TEST(MakeCenicIntegratorTest, FailureNoPlant) {
  DiagramBuilder<double> builder;
  builder.AddSystem<ConstantVectorSource>(0.0);
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  DRAKE_EXPECT_THROWS_MESSAGE(MakeCenicIntegrator(*diagram),
                              ".*does not have a subsystem named plant.*");
}

}  // namespace
}  // namespace cenic
}  // namespace multibody
}  // namespace drake
