#include "drake/planning/robot_diagram.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/primitives/shared_pointer_system.h"

namespace drake {
namespace planning {
namespace {

using geometry::SceneGraph;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using symbolic::Expression;
using systems::Context;
using systems::DiagramBuilder;
using systems::SharedPointerSystem;
using systems::System;

std::unique_ptr<RobotDiagramBuilder<double>> MakeSampleDut() {
  auto builder = std::make_unique<RobotDiagramBuilder<double>>();
  builder->parser().AddModelsFromUrl(
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf");
  return builder;
}

GTEST_TEST(RobotDiagramBuilderTest, TimeStepDefault) {
  const MultibodyPlantConfig default_plant_config;
  auto builder = std::make_unique<RobotDiagramBuilder<double>>();
  EXPECT_EQ(builder->plant().time_step(), default_plant_config.time_step);
}

GTEST_TEST(RobotDiagramBuilderTest, TimeStepExplicit) {
  const double time_step = 0.01;
  auto builder = std::make_unique<RobotDiagramBuilder<double>>(time_step);
  EXPECT_EQ(builder->plant().time_step(), time_step);
}

GTEST_TEST(RobotDiagramBuilderTest, Getters) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  const RobotDiagramBuilder<double>* const_dut = dut.get();

  DiagramBuilder<double>& mutable_builder = dut->builder();
  const DiagramBuilder<double>& builder = const_dut->builder();
  Parser& mutable_parser = dut->parser();
  const Parser& parser = const_dut->parser();
  MultibodyPlant<double>& mutable_plant = dut->plant();
  const MultibodyPlant<double>& plant = const_dut->plant();
  SceneGraph<double>& mutable_scene_graph = dut->scene_graph();
  const SceneGraph<double>& scene_graph = const_dut->scene_graph();

  // The parser has a reference to the builder.
  EXPECT_EQ(mutable_parser.builder(), &mutable_builder);

  // The getters for mutable vs readonly are consistent.
  EXPECT_EQ(&mutable_builder, &builder);
  EXPECT_EQ(&mutable_parser, &parser);
  EXPECT_EQ(&mutable_plant, &plant);
  EXPECT_EQ(&mutable_scene_graph, &scene_graph);

  // The inter-connects are consistent.
  EXPECT_EQ(&mutable_parser.plant(), &plant);
  EXPECT_THAT(builder.GetSystems(), ::testing::Contains(&plant));
  EXPECT_THAT(builder.GetSystems(), ::testing::Contains(&scene_graph));
}

// If the user has exported an input port, nothing else gets exported.
GTEST_TEST(RobotDiagramBuilderTest, NoDefaultExportCustomInputPort) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  dut->plant().Finalize();
  dut->builder().ExportInput(dut->plant().GetInputPort("actuation"));
  auto diagram = dut->Build();
  EXPECT_EQ(diagram->num_input_ports(), 1);
  EXPECT_EQ(diagram->num_output_ports(), 0);
  EXPECT_NO_THROW(diagram->GetInputPort("plant_actuation"));
}

// If the user has exported an output port, nothing else gets exported.
GTEST_TEST(RobotDiagramBuilderTest, NoDefaultExportCustomOutputPort) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  dut->builder().ExportOutput(dut->scene_graph().GetOutputPort("query"));
  auto diagram = dut->Build();
  EXPECT_EQ(diagram->num_input_ports(), 0);
  EXPECT_EQ(diagram->num_output_ports(), 1);
  EXPECT_NO_THROW(diagram->GetOutputPort("scene_graph_query"));
}

// If the user has added an extra system or renamed any system then nothing gets
// exported.
GTEST_TEST(RobotDiagramBuilderTest, NoDefaultExportCustomSystems) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut;

  std::array<std::function<void()>, 3> edits = {
      [&dut]() {
        dut->builder().AddSystem<SharedPointerSystem>(std::shared_ptr<void>());
      },
      [&dut]() {
        dut->plant().set_name("foo");
      },
      [&dut]() {
        dut->scene_graph().set_name("foo");
      },
  };

  for (size_t i = 0; i < edits.size(); ++i) {
    SCOPED_TRACE(fmt::format("i = {}", i));
    dut = MakeSampleDut();
    edits[i]();
    auto diagram = dut->Build();
    EXPECT_EQ(diagram->num_input_ports(), 0);
    EXPECT_EQ(diagram->num_output_ports(), 0);
  }
}

// All exported ports must be non-deprecated.
GTEST_TEST(RobotDiagramBuilderTest, NoExportDeprecatedPorts) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  auto diagram = dut->Build();
  for (int i = 0; i < diagram->num_input_ports(); ++i) {
    const auto& diagram_port = diagram->get_input_port(i);
    const std::string& diagram_port_name = diagram_port.get_name();
    if (diagram_port_name.starts_with("plant_")) {
      const auto& leaf_port =
          diagram->plant().GetInputPort(diagram_port_name.substr(6));
      EXPECT_EQ(leaf_port.get_deprecation(), std::nullopt)
          << leaf_port.get_name();
    } else if (diagram_port_name.starts_with("scene_graph_")) {
      const auto& leaf_port =
          diagram->scene_graph().GetInputPort(diagram_port_name.substr(12));
      EXPECT_EQ(leaf_port.get_deprecation(), std::nullopt)
          << leaf_port.get_name();
    } else {
      GTEST_FAIL() << diagram_port_name;
    }
  }
  for (int i = 0; i < diagram->num_output_ports(); ++i) {
    const auto& diagram_port = diagram->get_output_port(i);
    const std::string& diagram_port_name = diagram_port.get_name();
    if (diagram_port_name.starts_with("plant_")) {
      const auto& leaf_port =
          diagram->plant().GetOutputPort(diagram_port_name.substr(6));
      EXPECT_EQ(leaf_port.get_deprecation(), std::nullopt);
    } else if (diagram_port_name.starts_with("scene_graph_")) {
      const auto& leaf_port =
          diagram->scene_graph().GetOutputPort(diagram_port_name.substr(12));
      EXPECT_EQ(leaf_port.get_deprecation(), std::nullopt);
    } else {
      GTEST_FAIL() << diagram_port_name;
    }
  }
}

GTEST_TEST(RobotDiagramBuilderTest, Lifecycle) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();

  EXPECT_FALSE(dut->plant().is_finalized());
  EXPECT_FALSE(dut->IsDiagramBuilt());

  dut->plant().Finalize();
  EXPECT_TRUE(dut->plant().is_finalized());
  EXPECT_FALSE(dut->IsDiagramBuilt());

  auto robot_diagram = dut->Build();
  EXPECT_TRUE(dut->IsDiagramBuilt());

  // The lifecycle of the built Diagram has no effect on the builder's ability
  // to report its own lifecycle status.
  robot_diagram.reset();
  EXPECT_TRUE(dut->IsDiagramBuilt());
}

GTEST_TEST(RobotDiagramBuilderTest, LifecycleFailFast) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  const RobotDiagramBuilder<double>* const_dut = dut.get();
  auto robot_diagram = dut->Build();
  robot_diagram.reset();

  EXPECT_TRUE(dut->IsDiagramBuilt());
  constexpr const char* error = ".*may no longer be used.*";
  DRAKE_EXPECT_THROWS_MESSAGE(dut->builder(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(const_dut->builder(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(dut->parser(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(const_dut->parser(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(dut->plant(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(const_dut->plant(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(dut->scene_graph(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(const_dut->scene_graph(), error);
  DRAKE_EXPECT_THROWS_MESSAGE(dut->Build(), error);
}

GTEST_TEST(RobotDiagramBuilderTest, TooMuchSurgery) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut;

  dut = MakeSampleDut();
  dut->builder().RemoveSystem(dut->plant());
  DRAKE_EXPECT_THROWS_MESSAGE(dut->plant(), ".*not remove.*MultibodyPlant.*");

  dut = MakeSampleDut();
  dut->builder().RemoveSystem(dut->scene_graph());
  DRAKE_EXPECT_THROWS_MESSAGE(dut->scene_graph(), ".*not remove.*SceneGraph.*");
}

GTEST_TEST(RobotDiagramBuilderTest, AddSystem) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  dut->builder().AddNamedSystem<SharedPointerSystem>(
      "foo", std::make_shared<std::string>("bar"));
  auto diagram = dut->Build();
  EXPECT_TRUE(diagram->HasSubsystemNamed("foo"));
}

GTEST_TEST(RobotDiagramTest, SmokeTest) {
  std::unique_ptr<RobotDiagramBuilder<double>> dut = MakeSampleDut();
  std::unique_ptr<RobotDiagram<double>> robot_diagram = dut->Build();
  ASSERT_NE(robot_diagram, nullptr);
  EXPECT_NE(robot_diagram->CreateDefaultContext(), nullptr);
}

GTEST_TEST(RobotDiagramTest, ToAutoDiff) {
  std::unique_ptr<RobotDiagram<double>> robot_diagram_double =
      MakeSampleDut()->Build();
  std::unique_ptr<RobotDiagram<AutoDiffXd>> robot_diagram_autodiff =
      System<double>::ToAutoDiffXd(*robot_diagram_double);
  ASSERT_NE(robot_diagram_autodiff, nullptr);
  EXPECT_NE(robot_diagram_autodiff->CreateDefaultContext(), nullptr);
}

GTEST_TEST(RobotDiagramTest, ToSymbolic) {
  std::unique_ptr<RobotDiagram<double>> robot_diagram_double =
      MakeSampleDut()->Build();
  std::unique_ptr<RobotDiagram<Expression>> robot_diagram_symbolic =
      System<double>::ToSymbolic(*robot_diagram_double);
  ASSERT_NE(robot_diagram_symbolic, nullptr);
  EXPECT_NE(robot_diagram_symbolic->CreateDefaultContext(), nullptr);
}

GTEST_TEST(RobotDiagramBuilderTest, AutoDiffFromBirthToDeath) {
  auto builder_autodiff = std::make_unique<RobotDiagramBuilder<AutoDiffXd>>();
  builder_autodiff->plant().AddModelInstance("mike");
  std::unique_ptr<RobotDiagram<AutoDiffXd>> robot_diagram_autodiff =
      builder_autodiff->Build();
  ASSERT_NE(robot_diagram_autodiff, nullptr);
  EXPECT_NE(robot_diagram_autodiff->CreateDefaultContext(), nullptr);
  EXPECT_TRUE(robot_diagram_autodiff->plant().HasModelInstanceNamed("mike"));
}

GTEST_TEST(RobotDiagramTest, SystemGetters) {
  std::unique_ptr<RobotDiagram<double>> dut = MakeSampleDut()->Build();

  const MultibodyPlant<double>& plant = dut->plant();
  SceneGraph<double>& mutable_scene_graph = dut->mutable_scene_graph();
  const SceneGraph<double>& scene_graph = dut->scene_graph();

  // The getters for mutable vs readonly are consistent.
  EXPECT_EQ(&mutable_scene_graph, &scene_graph);

  // The inter-connects are consistent.
  EXPECT_THAT(dut->GetSystems(), ::testing::Contains(&plant));
  EXPECT_THAT(dut->GetSystems(), ::testing::Contains(&scene_graph));
}

GTEST_TEST(RobotDiagramTest, ContextGetters) {
  std::unique_ptr<RobotDiagram<double>> dut = MakeSampleDut()->Build();

  std::unique_ptr<Context<double>> root_context = dut->CreateDefaultContext();
  const Context<double>& plant_context = dut->plant_context(*root_context);
  const Context<double>& scene_graph_context =
      dut->scene_graph_context(*root_context);
  Context<double>& mutable_plant_context =
      dut->mutable_plant_context(root_context.get());
  Context<double>& mutable_scene_graph_context =
      dut->mutable_scene_graph_context(root_context.get());

  // The getters for mutable vs readonly are consistent.
  EXPECT_EQ(&mutable_plant_context, &plant_context);
  EXPECT_EQ(&mutable_scene_graph_context, &scene_graph_context);

  // The contexts are appropriate for their systems.
  EXPECT_NO_THROW(dut->plant().ValidateContext(plant_context));
  EXPECT_NO_THROW(dut->scene_graph().ValidateContext(scene_graph_context));
}

GTEST_TEST(RobotDiagramTest, Clone) {
  std::unique_ptr<RobotDiagram<double>> dut = MakeSampleDut()->Build();
  std::unique_ptr<RobotDiagram<double>> copy = System<double>::Clone(*dut);

  // Sanity check: the new plant is part of the new diagram.
  EXPECT_THAT(copy->GetSystems(), ::testing::Contains(&copy->plant()));

  // The new plant is distinct from the old plant.
  EXPECT_NE(&copy->plant(), &dut->plant());
}

GTEST_TEST(RobotDiagramTest, PortNamesExist) {
  std::unique_ptr<RobotDiagram<double>> dut = MakeSampleDut()->Build();

  // Write the Graphviz output for offline debugging.
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    std::ofstream out(std::filesystem::path(dir) / "iiwa14.dot");
    out << dut->GetGraphvizString();
    EXPECT_TRUE(out.good());
  }

  // Check the names mentioned in the class overview comment.
  EXPECT_NO_THROW(dut->GetInputPort("plant_actuation"));
  EXPECT_NO_THROW(dut->GetInputPort("plant_applied_generalized_force"));
  EXPECT_NO_THROW(dut->GetOutputPort("plant_state"));
  EXPECT_NO_THROW(dut->GetOutputPort("scene_graph_query"));

  // The scene graph should not export any inputs.
  for (int i = 0; i < dut->num_input_ports(); ++i) {
    EXPECT_THAT(dut->get_input_port(i).get_name(),
                ::testing::StartsWith("plant_"));
  }
}

}  // namespace
}  // namespace planning
}  // namespace drake
