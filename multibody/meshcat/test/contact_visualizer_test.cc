#include "drake/multibody/meshcat/contact_visualizer.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using geometry::Meshcat;

class ContactVisualizerTest : public ::testing::Test {
 protected:
  ContactVisualizerTest() : meshcat_(std::make_shared<Meshcat>()) {}

  // Sets up a simple plant, scene graph, and visualizer.
  // The plant has two spheres on prismatic joints.
  //
  // @param add_to_builder_overload must be 0, 1, or 2 to select which
  // "AddToBuilder" overload will be used.
  void SetUpDiagram(
      int add_to_builder_overload = 0,
      bool add_second_geometry = false,
      const ContactVisualizerParams& params = {}) {
    // Create the systems.
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

    // Add the spheres and joints.
    multibody::Parser parser(&plant);
    const std::string sdf = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/sphere.sdf");
    auto sphere1_model = parser.AddModelFromFile(sdf, "sphere1");
    const auto& sphere1 = plant.GetBodyByName("base_link", sphere1_model);
    plant.AddJoint<multibody::PrismaticJoint>(
        "sphere1_x", plant.world_body(), std::nullopt, sphere1, std::nullopt,
        Eigen::Vector3d::UnitX());
    auto sphere2_model = parser.AddModelFromFile(sdf, "sphere2");
    const auto& sphere2 = plant.GetBodyByName("base_link", sphere2_model);
    plant.AddJoint<multibody::PrismaticJoint>(
        "sphere2_x", plant.world_body(), std::nullopt, sphere2, std::nullopt,
        Eigen::Vector3d::UnitX());
    if (add_second_geometry) {
      auto shape = std::make_unique<geometry::Sphere>(0.04);
      const math::RigidTransformd X(Eigen::Vector3d(0.0, 0.0, 0.025));
      plant.RegisterCollisionGeometry(sphere2, X, *shape, "bonus",
          multibody::CoulombFriction<double>());
    }
    plant.Finalize();

    // Add the visualizer.
    switch (add_to_builder_overload) {
     case 0:
      visualizer_ = &ContactVisualizerd::AddToBuilder(
          &builder, plant, meshcat_, params);
      break;
     case 1:
      visualizer_ = &ContactVisualizerd::AddToBuilder(
          &builder, plant.get_contact_results_output_port(),
          scene_graph.get_query_output_port(),
          meshcat_, params);
      break;
     case 2:
      visualizer_ = &ContactVisualizerd::AddToBuilder(
          &builder, plant.get_contact_results_output_port(),
          meshcat_, params);
      break;
     default:
      DRAKE_UNREACHABLE();
    }

    // Start the two spheres in contact, but not completely overlapping.
    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
    plant.SetPositions(&plant.GetMyMutableContextFromRoot(context_.get()),
                       Eigen::Vector2d{-0.03, 0.03});
  }

  void PublishAndCheck(
      bool expect_geometry_names = false) {
    diagram_->Publish(*context_);
    if (expect_geometry_names) {
      EXPECT_TRUE(meshcat_->HasPath(
          "contact_forces/point/sphere1.base_link+sphere2.base_link.bonus"));
    } else {
      EXPECT_TRUE(meshcat_->HasPath(
          "contact_forces/point/sphere1.base_link+sphere2.base_link"));
    }
  }

  std::shared_ptr<Meshcat> meshcat_;
  const ContactVisualizer<double>* visualizer_{};
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> context_{};
};

// Tests the preferred spelling of AddToBuilder.
TEST_F(ContactVisualizerTest, AddToBuilder0) {
  SetUpDiagram(0);
  PublishAndCheck();
}

// Tests the first alternative spelling of AddToBuilder.
TEST_F(ContactVisualizerTest, AddToBuilder1) {
  SetUpDiagram(1);
  PublishAndCheck();
}

// Tests the second alternative spelling of AddToBuilder.
TEST_F(ContactVisualizerTest, AddToBuilder2) {
  SetUpDiagram(2);
  PublishAndCheck();
}

// Tests that the preferred spelling of AddToBuilder does not crash under the
// insane conditions of being added into a diagram that lacks a SceneGraph.
// (A diagram without a SceneGraph has no geometry so could never produce any
// contact results.)
TEST_F(ContactVisualizerTest, InsaneAddToBuilder0) {
  systems::DiagramBuilder<double> builder;
  auto* plant = builder.AddSystem<MultibodyPlant<double>>(0.001);
  plant->Finalize();
  visualizer_ = &ContactVisualizerd::AddToBuilder(
      &builder, *plant, meshcat_);
}

// Test that per-geometry names make it through.
TEST_F(ContactVisualizerTest, PerGeometryNaming0) {
  SetUpDiagram(0, true);
  PublishAndCheck(true);
}

// Test that per-geometry names make it through.
TEST_F(ContactVisualizerTest, PerGeometryNaming1) {
  SetUpDiagram(1, true);
  PublishAndCheck(true);
}

TEST_F(ContactVisualizerTest, Parameters) {
  ContactVisualizerParams params;
  params.prefix = "test_prefix";
  params.publish_period = 1 / 12.0;
  // We don't have a good way to test the other parameters without
  // visualization; they are partially covered by meshcat_manual_test.
  SetUpDiagram(0, false, params);

  diagram_->Publish(*context_);
  EXPECT_FALSE(meshcat_->HasPath("contact_forces"));
  EXPECT_TRUE(meshcat_->HasPath("test_prefix"));

  auto periodic_events = visualizer_->GetPeriodicEvents();
  for (const auto& data_and_vector : periodic_events) {
    EXPECT_EQ(data_and_vector.second.size(), 1);  // only one periodic event
    EXPECT_EQ(data_and_vector.first.period_sec(), params.publish_period);
    EXPECT_EQ(data_and_vector.first.offset_sec(), 0.0);
  }
}

TEST_F(ContactVisualizerTest, Delete) {
  SetUpDiagram();

  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("contact_forces"));

  visualizer_->Delete();
  EXPECT_FALSE(meshcat_->HasPath("contact_forces"));

  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("contact_forces"));
}

GTEST_TEST(ContactVisualizer, PortTest) {
  auto meshcat = std::make_shared<Meshcat>();
  // Also tests the default constructor.
  ContactVisualizer<double> visualizer(meshcat);
  auto context = visualizer.CreateDefaultContext();

  // Confirm that I can assign a ContactResults to the input port.
  multibody::ContactResults<double> results;
  visualizer.contact_results_input_port().FixValue(context.get(), results);

  // Confirm that I can assign a ContactResults to the input port.
  geometry::QueryObject<double> query;
  visualizer.query_object_input_port().FixValue(context.get(), query);
}

TEST_F(ContactVisualizerTest, ScalarConversion) {
  SetUpDiagram();

  auto ad_diagram = diagram_->ToAutoDiffXd();
  auto ad_context = ad_diagram->CreateDefaultContext();

  // Call publish to provide code coverage for the AutoDiffXd version of
  // UpdateMeshcat.  We simply confirm that the code doesn't blow up.
  ad_diagram->Publish(*ad_context);
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
