#include "drake/geometry/meshcat_point_cloud_visualizer.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace geometry {
namespace {

class MeshcatPointCloudVisualizerTest : public ::testing::Test {
 protected:
  MeshcatPointCloudVisualizerTest() : meshcat_(std::make_shared<Meshcat>()) {}

  void SetUpDiagram(bool connect_pose = true,
                    double publish_period = 1 / 32.0) {
    systems::DiagramBuilder<double> builder;

    visualizer_ = builder.AddSystem<MeshcatPointCloudVisualizer>(
        meshcat_, "cloud", publish_period);

    perception::PointCloud cloud(5);
    // clang-format off
    cloud.mutable_xyzs().transpose() <<
      1, 2, 3,
      10, 20, 30,
      100, 200, 300,
      4, 5, 6,
      40, 50, 60;
    // clang-format on

    auto cloud_system = builder.template AddSystem<
      systems::ConstantValueSource>(Value<perception::PointCloud>(cloud));
    builder.Connect(cloud_system->get_output_port(),
                    visualizer_->cloud_input_port());

    if (connect_pose) {
      auto pose_system = builder.template AddSystem<
        systems::ConstantValueSource>(Value<math::RigidTransformd>());
      builder.Connect(pose_system->get_output_port(),
                      visualizer_->pose_input_port());
    }

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
  }

  std::shared_ptr<Meshcat> meshcat_;
  MeshcatPointCloudVisualizer<double>* visualizer_{};
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> context_{};
};


TEST_F(MeshcatPointCloudVisualizerTest, Publish) {
  SetUpDiagram();

  EXPECT_TRUE(meshcat_->GetPackedObject("cloud").empty());
  EXPECT_TRUE(meshcat_->GetPackedTransform("cloud").empty());
  diagram_->Publish(*context_);
  EXPECT_FALSE(meshcat_->GetPackedObject("cloud").empty());
  EXPECT_FALSE(meshcat_->GetPackedTransform("cloud").empty());
}

TEST_F(MeshcatPointCloudVisualizerTest, NoPose) {
  SetUpDiagram(false);

  EXPECT_TRUE(meshcat_->GetPackedObject("cloud").empty());
  EXPECT_TRUE(meshcat_->GetPackedTransform("cloud").empty());
  diagram_->Publish(*context_);
  EXPECT_FALSE(meshcat_->GetPackedObject("cloud").empty());
  // It still publishes a transform; but it publishes the identity.
  EXPECT_FALSE(meshcat_->GetPackedTransform("cloud").empty());
}

TEST_F(MeshcatPointCloudVisualizerTest, PublishPeriod) {
  const double kPeriod = 1/12.0;
  SetUpDiagram(true, kPeriod);

  auto periodic_events = visualizer_->GetPeriodicEvents();
  for (const auto& data_and_vector : periodic_events) {
    EXPECT_EQ(data_and_vector.second.size(), 1);  // only one periodic event
    EXPECT_EQ(data_and_vector.first.period_sec(), kPeriod);
    EXPECT_EQ(data_and_vector.first.offset_sec(), 0.0);
  }
}

TEST_F(MeshcatPointCloudVisualizerTest, Delete) {
  SetUpDiagram();

  diagram_->Publish(*context_);
  EXPECT_FALSE(meshcat_->GetPackedObject("cloud").empty());

  visualizer_->Delete();
  EXPECT_TRUE(meshcat_->GetPackedObject("cloud").empty());
}

TEST_F(MeshcatPointCloudVisualizerTest, VisualizationParameters) {
  SetUpDiagram();

  // We don't have a good way to test their effects without visualization; but
  // simply provide test coverage for the methods.
  visualizer_->set_point_size(0.1);
  visualizer_->set_default_rgba(Rgba(0, 0, 1, 1));
}

TEST_F(MeshcatPointCloudVisualizerTest, ScalarConversion) {
  SetUpDiagram(false);

  auto ad_diagram = diagram_->ToAutoDiffXd();
  auto ad_context = ad_diagram->CreateDefaultContext();

  // Call publish to provide code coverage for the AutoDiffXd version of
  // UpdateMeshcat.  We simply confirm that the code doesn't blow up.
  ad_diagram->Publish(*ad_context);
}



}  // namespace
}  // namespace geometry
}  // namespace drake
