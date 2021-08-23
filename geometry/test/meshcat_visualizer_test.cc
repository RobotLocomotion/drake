#include "drake/geometry/meshcat_visualizer.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace geometry {
namespace {

// The tests in this file require a dependency on MultibodyPlant.  One could
// implement the tests without that dependency, but not without duplicating (or
// sharing) a significant amount of setup code like we see in
// DrakeVisualizerTest.
//
// Relative to DrakeVisualizerTest, this file does not have to provide test
// coverage for the message passing, nor for the variety of geometry types.
// Those are covered by the tests for Meshcat.  Here we only aim to demonstrate
// that we call Meshcat correctly from MeshcatVisualizer.

class MeshcatVisualizerWithIiwaTest : public ::testing::Test {
 protected:
  void SetUpDiagram(MeshcatVisualizerParams params = {}) {
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    plant_ = &plant;
    scene_graph_ = &scene_graph;
    multibody::Parser(plant_).AddModelFromFile(
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                            "iiwa14_spheres_collision.urdf"));
    plant.WeldFrames(plant.world_frame(),
                     plant.GetBodyByName("base").body_frame());
    plant.Finalize();

    auto zero_torque =
        builder.template AddSystem<systems::ConstantVectorSource<double>>(
            Eigen::VectorXd::Zero(7));
    builder.Connect(zero_torque->get_output_port(),
                    plant.get_actuation_input_port());

    visualizer_ = &MeshcatVisualizer<double>::AddToBuilder(
        &builder, scene_graph, &meshcat_, std::move(params));

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
  }

  Meshcat meshcat_;
  multibody::MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  const MeshcatVisualizer<double>* visualizer_{};
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> context_{};
};

TEST_F(MeshcatVisualizerWithIiwaTest, BasicTest) {
  SetUpDiagram();

  EXPECT_EQ(visualizer_->meshcat(), &meshcat_);
  EXPECT_FALSE(meshcat_.HasPath("/drake/visualizer/iiwa14"));
  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_.HasPath("/drake/visualizer/iiwa14"));
  for (int link = 0; link < 8; link++) {
    EXPECT_NE(meshcat_.GetPackedTransform(
                  fmt::format("/drake/visualizer/iiwa14/iiwa_link_{}", link)),
              "");
  }

  // Confirm that the transforms change after running a simulation.
  const std::string packed_X_W7 =
      meshcat_.GetPackedTransform("visualizer/iiwa14/iiwa_link_7");
  systems::Simulator<double> simulator(*diagram_);
  simulator.AdvanceTo(0.1);
  EXPECT_NE(meshcat_.GetPackedTransform("visualizer/iiwa14/iiwa_link_7"),
            packed_X_W7);
}

TEST_F(MeshcatVisualizerWithIiwaTest, PublishPeriod) {
  MeshcatVisualizerParams params;
  params.publish_period = 0.123;
  SetUpDiagram(params);

  int num_periodic_events = 0;
  auto periodic_events = visualizer_->GetPeriodicEvents();
  for (const auto& data_and_vector : periodic_events) {
    for (const auto& event : data_and_vector.second) {
      if (event->get_trigger_type() == systems::TriggerType::kPeriodic) {
        EXPECT_EQ(data_and_vector.first.period_sec(), params.publish_period);
        EXPECT_EQ(data_and_vector.first.offset_sec(), 0.0);
        num_periodic_events++;
      }
    }
  }
  EXPECT_EQ(num_periodic_events, 1);
}

TEST_F(MeshcatVisualizerWithIiwaTest, Roles) {
  MeshcatVisualizerParams params;
  for (Role role : {Role::kProximity, Role::kIllustration, Role::kPerception}) {
    params.role = role;
    SetUpDiagram(params);
    EXPECT_FALSE(meshcat_.HasPath("visualizer/iiwa14/iiwa_link_7"));
    diagram_->Publish(*context_);
    EXPECT_TRUE(meshcat_.HasPath("visualizer/iiwa14/iiwa_link_7"));
    meshcat_.Delete();
  }

  params.role = Role::kUnassigned;
  DRAKE_EXPECT_THROWS_MESSAGE(SetUpDiagram(params),
                              ".*Role::kUnassigned.*");
}

TEST_F(MeshcatVisualizerWithIiwaTest, Prefix) {
  MeshcatVisualizerParams params;

  // Absolute path.
  params.prefix = "/foo";
  SetUpDiagram(params);
  EXPECT_FALSE(meshcat_.HasPath("/foo/iiwa14"));
  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_.HasPath("/foo/iiwa14"));
  EXPECT_FALSE(meshcat_.HasPath("/drake/visualizer"));

  // Relative path.
  params.prefix = "foo";
  EXPECT_FALSE(meshcat_.HasPath("/drake/foo/iiwa14"));
  SetUpDiagram(params);
  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_.HasPath("/drake/foo/iiwa14"));
  EXPECT_FALSE(meshcat_.HasPath("/drake/visualizer"));
}

TEST_F(MeshcatVisualizerWithIiwaTest, DeletePrefixOnInitialization) {
  MeshcatVisualizerParams params;
  params.delete_prefix_on_initialization_event = true;
  SetUpDiagram(params);
  // Scribble a transform onto the scene tree beneath the visualizer prefix.
  meshcat_.SetTransform("/drake/visualizer/my_random_path",
                        math::RigidTransformd());
  EXPECT_TRUE(meshcat_.HasPath("/drake/visualizer/my_random_path"));

  {  // Send an initialization event.
    auto events = diagram_->AllocateCompositeEventCollection();
    diagram_->GetInitializationEvents(*context_, events.get());
    diagram_->Publish(*context_, events->get_publish_events());
  }
  // Confirm that my scribble was deleted.
  EXPECT_FALSE(meshcat_.HasPath("/drake/visualizer/my_random_path"));

  // Repeat, but this time with delete prefix disable.
  params.delete_prefix_on_initialization_event = false;
  SetUpDiagram(params);
  meshcat_.SetTransform("/drake/visualizer/my_random_path",
                        math::RigidTransformd());
  {  // Send an initialization event.
    auto events = diagram_->AllocateCompositeEventCollection();
    diagram_->GetInitializationEvents(*context_, events.get());
    diagram_->Publish(*context_, events->get_publish_events());
  }
  // Confirm that my scribble remains.
  EXPECT_TRUE(meshcat_.HasPath("/drake/visualizer/my_random_path"));
}

TEST_F(MeshcatVisualizerWithIiwaTest, DeletePrefix) {
  SetUpDiagram();
  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_.HasPath("/drake/visualizer"));
  visualizer_->DeletePrefix();
  EXPECT_FALSE(meshcat_.HasPath("/drake/visualizer"));
}

GTEST_TEST(MeshcatVisualizerTest, MultipleModels) {
  Meshcat meshcat;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
  std::string urdf = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_no_collision.urdf");
  auto iiwa0 = multibody::Parser(&plant).AddModelFromFile(urdf);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base", iiwa0).body_frame());
  auto iiwa1 = multibody::Parser(&plant).AddModelFromFile(urdf, "second_iiwa");
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base", iiwa1).body_frame());
  plant.Finalize();

  auto zero_torque = builder.AddSystem<systems::ConstantVectorSource>(
      Eigen::VectorXd::Zero(7));
  builder.Connect(zero_torque->get_output_port(),
                  plant.get_actuation_input_port(iiwa0));
  builder.Connect(zero_torque->get_output_port(),
                  plant.get_actuation_input_port(iiwa1));

  // Use the query_output_port version of AddToBuilder.
  const auto& visualizer = MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph.get_query_output_port(), &meshcat);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  EXPECT_EQ(visualizer.meshcat(), &meshcat);
  EXPECT_FALSE(meshcat.HasPath("/drake/visualizer/iiwa14"));
  EXPECT_FALSE(meshcat.HasPath("/drake/visualizer/second_iiwa"));

  diagram->Publish(*context);

  EXPECT_TRUE(meshcat.HasPath("/drake/visualizer/iiwa14"));
  EXPECT_TRUE(meshcat.HasPath("/drake/visualizer/second_iiwa"));
  for (int link = 0; link < 8; link++) {
    EXPECT_NE(meshcat.GetPackedTransform(
                  fmt::format("/drake/visualizer/iiwa14/iiwa_link_{}", link)),
              "");
    EXPECT_NE(meshcat.GetPackedTransform(fmt::format(
                  "/drake/visualizer/second_iiwa/iiwa_link_{}", link)),
              "");
  }
}

GTEST_TEST(MeshcatVisualizerTest, ScalarConversion) {
  auto meshcat = std::make_shared<Meshcat>();

  MeshcatVisualizerd raw(meshcat.get());
  MeshcatVisualizerd owned(meshcat);

  DRAKE_EXPECT_THROWS_MESSAGE(
      raw.ToAutoDiffXd(),
      "MeshcatVisualizer can only be scalar converted if it owns.*");
  auto autodiff = owned.ToAutoDiffXd();
  autodiff->CreateDefaultContext();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
