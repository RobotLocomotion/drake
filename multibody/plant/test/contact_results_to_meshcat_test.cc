#include "drake/multibody/plant/contact_results_to_meshcat.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using geometry::Meshcat;

// Set up a simple plant + visualizer.  The plant has two spheres on prismatic
// joints.
class ContactResultsToMeshcatTest : public ::testing::Test {
 protected:
  ContactResultsToMeshcatTest() : meshcat_(std::make_shared<Meshcat>()) {}

  void SetUpDiagram(const ContactResultsToMeshcatParams& cparams =
                        ContactResultsToMeshcatParams()) {
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    multibody::Parser parser(&plant);
    const std::string sdf = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/sphere.sdf");
    auto sphere1_model = parser.AddModelFromFile(sdf, "sphere1");
    plant.AddJoint<multibody::PrismaticJoint>(
        "sphere1_x", plant.world_body(), std::nullopt,
        plant.GetBodyByName("base_link", sphere1_model), std::nullopt,
        Eigen::Vector3d::UnitX());
    auto sphere2_model = parser.AddModelFromFile(sdf, "sphere2");
    plant.AddJoint<multibody::PrismaticJoint>(
        "sphere2_x", plant.world_body(), std::nullopt,
        plant.GetBodyByName("base_link", sphere2_model), std::nullopt,
        Eigen::Vector3d::UnitX());
    plant.Finalize();
    // The SceneGraph is needed for contact, but not referenced directly.
    unused(scene_graph);

    visualizer_ = &ContactResultsToMeshcatd::AddToBuilder(&builder, plant,
                                                          meshcat_, cparams);

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
    // Start them in contact, but not completely overlapping.
    plant.SetPositions(&plant.GetMyMutableContextFromRoot(context_.get()),
                       Eigen::Vector2d{-0.03, 0.03});
  }

  std::shared_ptr<Meshcat> meshcat_;
  const ContactResultsToMeshcat<double>* visualizer_{};
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> context_{};
};

TEST_F(ContactResultsToMeshcatTest, Publish) {
  SetUpDiagram();

  EXPECT_FALSE(meshcat_->HasPath("contact_forces"));
  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("contact_forces"));
}

TEST_F(ContactResultsToMeshcatTest, Parameters) {
  ContactResultsToMeshcatParams params;
  params.prefix = "test_prefix";
  params.publish_period = 1 / 12.0;
  // We don't have a good way to test the other parameters without
  // visualization; they are partially covered by meshcat_manual_test.
  SetUpDiagram(params);

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

TEST_F(ContactResultsToMeshcatTest, Delete) {
  SetUpDiagram();

  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("contact_forces"));

  visualizer_->Delete();
  EXPECT_FALSE(meshcat_->HasPath("contact_forces"));

  diagram_->Publish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("contact_forces"));
}

GTEST_TEST(ContactResultsToMeshcat, PortTest) {
  auto meshcat = std::make_shared<Meshcat>();
  // Also tests the default constructor.
  ContactResultsToMeshcat<double> visualizer(meshcat);
  auto context = visualizer.CreateDefaultContext();

  // Confirm that I can assign a ContactResults to this port.
  multibody::ContactResults<double> results;
  visualizer.contact_results_input_port().FixValue(context.get(), results);
}

// Test the other variant of AddToBuilder (taking the port instead of the
// plant).
GTEST_TEST(ContactResultsToMeshcat, AddToBuilderVariant) {
  auto meshcat = std::make_shared<Meshcat>();
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser parser(&plant);
  const std::string sdf = FindResourceOrThrow(
      "drake/examples/manipulation_station/models/sphere.sdf");
  auto sphere1_model = parser.AddModelFromFile(sdf, "sphere1");
  plant.AddJoint<multibody::PrismaticJoint>(
      "sphere1_x", plant.world_body(), std::nullopt,
      plant.GetBodyByName("base_link", sphere1_model), std::nullopt,
      Eigen::Vector3d::UnitX());
  auto sphere2_model = parser.AddModelFromFile(sdf, "sphere2");
  plant.AddJoint<multibody::PrismaticJoint>(
      "sphere2_x", plant.world_body(), std::nullopt,
      plant.GetBodyByName("base_link", sphere2_model), std::nullopt,
      Eigen::Vector3d::UnitX());
  plant.Finalize();
  // The SceneGraph is needed for contact, but not referenced directly.
  unused(scene_graph);

  ContactResultsToMeshcatd::AddToBuilder(
      &builder, plant.get_contact_results_output_port(), meshcat);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  // Start them in contact, but not completely overlapping.
  plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()),
                     Eigen::Vector2d{-0.03, 0.03});
  diagram->Publish(*context);
  EXPECT_TRUE(meshcat->HasPath("contact_forces"));
}

TEST_F(ContactResultsToMeshcatTest, ScalarConversion) {
  SetUpDiagram();

  auto ad_diagram = diagram_->ToAutoDiffXd();
  auto ad_context = ad_diagram->CreateDefaultContext();

  // Call publish to provide code coverage for the AutoDiffXd version of
  // UpdateMeshcat.  We simply confirm that the code doesn't blow up.
  ad_diagram->Publish(*ad_context);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
