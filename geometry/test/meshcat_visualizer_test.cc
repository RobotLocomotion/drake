#include "drake/geometry/meshcat_visualizer.h"

#include <drake_vendor/msgpack.hpp>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat_types.h"
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
//
// We're intentionally not building MeshcatVisualizer<AutoDiffXd> directly.
// Parsing with AutoDiffXd is not supported and populating the MBP is more work
// than it's worth. The scalar-converted instance is tested and that provides
// sufficient evidence for the validity of the type.

class MeshcatVisualizerWithIiwaTest : public ::testing::Test {
 protected:
  MeshcatVisualizerWithIiwaTest() : meshcat_(std::make_shared<Meshcat>()) {}

  void SetUpDiagram(MeshcatVisualizerParams params = {}) {
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    plant_ = &plant;
    scene_graph_ = &scene_graph;
    multibody::Parser(plant_).AddModels(
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
        &builder, scene_graph, meshcat_, std::move(params));

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();
  }

  std::shared_ptr<Meshcat> meshcat_;
  multibody::MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  MeshcatVisualizer<double>* visualizer_{};
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> context_{};
};

TEST_F(MeshcatVisualizerWithIiwaTest, BasicTest) {
  SetUpDiagram();

  EXPECT_FALSE(meshcat_->HasPath("/drake/visualizer/iiwa14"));
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("/drake/visualizer/iiwa14"));
  for (int link = 0; link < 8; link++) {
    EXPECT_NE(meshcat_->GetPackedTransform(
                  fmt::format("/drake/visualizer/iiwa14/iiwa_link_{}", link)),
              "");
  }

  // Confirm that the transforms change after running a simulation.
  const std::string packed_X_W7 =
      meshcat_->GetPackedTransform("visualizer/iiwa14/iiwa_link_7");
  systems::Simulator<double> simulator(*diagram_);
  simulator.AdvanceTo(0.1);
  EXPECT_NE(meshcat_->GetPackedTransform("visualizer/iiwa14/iiwa_link_7"),
            packed_X_W7);
}

TEST_F(MeshcatVisualizerWithIiwaTest, PublishPeriod) {
  MeshcatVisualizerParams params;
  params.publish_period = 0.123;
  SetUpDiagram(params);
  auto periodic_events_map = visualizer_->MapPeriodicEventsByTiming();
  for (const auto& data_and_vector : periodic_events_map) {
    EXPECT_EQ(data_and_vector.second.size(), 1);  // only one periodic event
    EXPECT_EQ(data_and_vector.first.period_sec(), params.publish_period);
    EXPECT_EQ(data_and_vector.first.offset_sec(), 0.0);
  }
}

// Confirms that all geometry registered to iiwa_link_7 in the urdf (in all
// three allowed roles) gets properly added.
TEST_F(MeshcatVisualizerWithIiwaTest, Roles) {
  // This also tests adding multiple MeshcatVisualizers to a single meshcat,
  // which is a common workflow in Python notebooks.
  MeshcatVisualizerParams params;
  for (Role role : {Role::kProximity, Role::kIllustration, Role::kPerception}) {
    params.role = role;
    SetUpDiagram(params);
    EXPECT_FALSE(meshcat_->HasPath("visualizer/iiwa14/iiwa_link_7"));
    diagram_->ForcedPublish(*context_);
    EXPECT_TRUE(meshcat_->HasPath("visualizer/iiwa14/iiwa_link_7"));
    auto& inspector = scene_graph_->model_inspector();
    FrameId iiwa_link_7 = plant_->GetBodyFrameIdOrThrow(
        plant_->GetBodyByName("iiwa_link_7").index());
    for (GeometryId geom_id : inspector.GetGeometries(iiwa_link_7, role)) {
      EXPECT_TRUE(meshcat_->HasPath(
          fmt::format("visualizer/iiwa14/iiwa_link_7/{}", geom_id)));
    }
    meshcat_->Delete();
  }

  params.role = Role::kUnassigned;
  DRAKE_EXPECT_THROWS_MESSAGE(SetUpDiagram(params),
                              ".*Role::kUnassigned.*");
}

// Tests that adding multiple MeshcatVisualizers using the same role to a
// single meshcat works, as this is a common workflow in Python notebooks.
TEST_F(MeshcatVisualizerWithIiwaTest, DuplicateRole) {
  MeshcatVisualizerParams params;
  params.role = Role::kIllustration;
  SetUpDiagram(params);
  SetUpDiagram(params);
}

TEST_F(MeshcatVisualizerWithIiwaTest, Prefix) {
  MeshcatVisualizerParams params;

  // Absolute path.
  params.prefix = "/foo";
  SetUpDiagram(params);
  EXPECT_EQ(visualizer_->get_name(), "meshcat_visualizer(/foo)");
  EXPECT_FALSE(meshcat_->HasPath("/foo/iiwa14"));
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("/foo/iiwa14"));
  EXPECT_FALSE(meshcat_->HasPath("/drake/visualizer"));

  // Relative path.
  params.prefix = "foo";
  EXPECT_FALSE(meshcat_->HasPath("/drake/foo/iiwa14"));
  SetUpDiagram(params);
  EXPECT_EQ(visualizer_->get_name(), "meshcat_visualizer(foo)");
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("/drake/foo/iiwa14"));
  EXPECT_FALSE(meshcat_->HasPath("/drake/visualizer"));
}

TEST_F(MeshcatVisualizerWithIiwaTest, NotVisibleByDefault) {
  // Set "visible" to start out false.
  MeshcatVisualizerParams params;
  params.visible_by_default = false;

  // Create the diagram and publish both the initialization and periodic event.
  SetUpDiagram(params);
  {
    auto events = diagram_->AllocateCompositeEventCollection();
    diagram_->GetInitializationEvents(*context_, events.get());
    diagram_->Publish(*context_, events->get_publish_events());
    diagram_->ForcedPublish(*context_);
  }

  // Confirm that the path was added but was set to be invisible.
  ASSERT_TRUE(meshcat_->HasPath("/drake/visualizer"));
  const std::string property =
      meshcat_->GetPackedProperty("/drake/visualizer", "visible");
  msgpack::object_handle oh = msgpack::unpack(property.data(), property.size());
  auto data = oh.get().as<internal::SetPropertyData<bool>>();
  EXPECT_EQ(data.property, "visible");
  EXPECT_EQ(data.value, false);
}

TEST_F(MeshcatVisualizerWithIiwaTest, DeletePrefixOnInitialization) {
  MeshcatVisualizerParams params;
  params.delete_on_initialization_event = true;
  SetUpDiagram(params);
  // Scribble a transform onto the scene tree beneath the visualizer prefix.
  meshcat_->SetTransform("/drake/visualizer/my_random_path",
                        math::RigidTransformd());
  EXPECT_TRUE(meshcat_->HasPath("/drake/visualizer/my_random_path"));

  {  // Send an initialization event.
    auto events = diagram_->AllocateCompositeEventCollection();
    diagram_->GetInitializationEvents(*context_, events.get());
    diagram_->Publish(*context_, events->get_publish_events());
  }
  // Confirm that my scribble was deleted.
  EXPECT_FALSE(meshcat_->HasPath("/drake/visualizer/my_random_path"));

  // Repeat, but this time with delete prefix disabled.
  params.delete_on_initialization_event = false;
  SetUpDiagram(params);
  meshcat_->SetTransform("/drake/visualizer/my_random_path",
                        math::RigidTransformd());
  {  // Send an initialization event.
    auto events = diagram_->AllocateCompositeEventCollection();
    diagram_->GetInitializationEvents(*context_, events.get());
    diagram_->Publish(*context_, events->get_publish_events());
  }
  // Confirm that my scribble remains.
  EXPECT_TRUE(meshcat_->HasPath("/drake/visualizer/my_random_path"));
}

TEST_F(MeshcatVisualizerWithIiwaTest, Delete) {
  SetUpDiagram();
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("/drake/visualizer"));
  visualizer_->Delete();
  EXPECT_FALSE(meshcat_->HasPath("/drake/visualizer"));
}

// If the results of Publish have been recorded to `animation`, then a
// SetTransform will have been set; we can check this by querying the
// "position".
bool has_iiwa_frame(const MeshcatAnimation& animation, int frame) {
  return animation
      .get_key_frame<std::vector<double>>(
          0, "visualizer/iiwa14/iiwa_link_1", "position")
      .has_value();
}

TEST_F(MeshcatVisualizerWithIiwaTest, Recording) {
  MeshcatVisualizerParams params;
  SetUpDiagram(params);
  DRAKE_EXPECT_THROWS_MESSAGE(visualizer_->get_mutable_recording(),
                              ".*You must create a recording.*");

  // Publish once without recording and confirm that we don't have the iiwa
  // frame.
  diagram_->ForcedPublish(*context_);
  visualizer_->StartRecording();
  auto animation = visualizer_->get_mutable_recording();
  EXPECT_FALSE(has_iiwa_frame(*animation, 0));

  // Publish again *with* recording and confirm that we do now have the frame.
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(has_iiwa_frame(*animation, 0));

  // Deleting the recording removes that frame.
  visualizer_->DeleteRecording();
  animation = visualizer_->get_mutable_recording();
  EXPECT_FALSE(has_iiwa_frame(*animation, 0));

  // We are still recording, so publish *will* add it.
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(has_iiwa_frame(*animation, 0));

  // But if we stop recording, then it's not added.
  visualizer_->StopRecording();
  visualizer_->DeleteRecording();
  animation = visualizer_->get_mutable_recording();
  EXPECT_FALSE(has_iiwa_frame(*animation, 0));
  diagram_->ForcedPublish(*context_);
  EXPECT_FALSE(has_iiwa_frame(*animation, 0));

  // Now publish a time 0.0 and time = 1.0 and confirm we have the frames.
  animation = visualizer_->StartRecording();
  diagram_->ForcedPublish(*context_);
  context_->SetTime(1.0);
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(has_iiwa_frame(*animation, 0));
  EXPECT_TRUE(
      has_iiwa_frame(*animation, std::floor(1.0 / params.publish_period)));

  // Confirm that PublishRecording runs.  Its correctness is established by
  // meshcat_manual_test.
  visualizer_->PublishRecording();
}

TEST_F(MeshcatVisualizerWithIiwaTest, RecordingWithoutSetTransform) {
  SetUpDiagram();
  diagram_->ForcedPublish(*context_);
  std::string X_7_message =
      meshcat_->GetPackedTransform("/drake/visualizer/iiwa14/iiwa_link_7");

  // Now update the position of the iiwa.
  plant_->SetPositions(&plant_->GetMyMutableContextFromRoot(context_.get()),
                       Eigen::VectorXd::Constant(7, .1));

  bool set_transforms_while_recording = false;
  visualizer_->StartRecording(set_transforms_while_recording);
  // This publish should *not* change the transform in the Meshcat scene tree.
  diagram_->ForcedPublish(*context_);
  EXPECT_EQ(
      meshcat_->GetPackedTransform("/drake/visualizer/iiwa14/iiwa_link_7"),
      X_7_message);

  set_transforms_while_recording = true;
  visualizer_->StartRecording(set_transforms_while_recording);
  // This publish *should* change the transform in the Meshcat scene tree.
  diagram_->ForcedPublish(*context_);
  EXPECT_NE(
      meshcat_->GetPackedTransform("/drake/visualizer/iiwa14/iiwa_link_7"),
      X_7_message);
}

TEST_F(MeshcatVisualizerWithIiwaTest, ScalarConversion) {
  SetUpDiagram();

  auto ad_diagram = diagram_->ToAutoDiffXd();
  auto ad_context = ad_diagram->CreateDefaultContext();

  // Call publish to provide code coverage for the AutoDiffXd version of
  // UpdateMeshcat / SetObjects SetTransforms.  We simply confirm that the code
  // doesn't blow up.
  ad_diagram->ForcedPublish(*ad_context);
}

TEST_F(MeshcatVisualizerWithIiwaTest, UpdateAlphaSliders) {
  MeshcatVisualizerParams params;
  params.enable_alpha_slider = true;
  SetUpDiagram(params);
  systems::Simulator<double> simulator(*diagram_);

  // Simulate for a moment and publish to populate the visualizer.
  simulator.AdvanceTo(0.1);
  diagram_->ForcedPublish(*context_);

  meshcat_->SetSliderValue("visualizer α", 0.5);

  // Simulate and publish again to cause an update.
  simulator.AdvanceTo(0.1);
  diagram_->ForcedPublish(*context_);
}

// When opted-in by the user, we should display the hydroelastic tellesation
// instead of the primitive shape.
GTEST_TEST(MeshcatVisualizerTest, HydroGeometry) {
  auto meshcat = std::make_shared<Meshcat>();
  for (bool show_hydroelastic : {false, true}) {
    // Load a scene with hydroelastic geometry.
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    multibody::Parser(&plant).AddModelsFromUrl(
        "package://drake/multibody/meshcat/test/hydroelastic.sdf");
    plant.Finalize();

    // Dig out a GeometryId that we just loaded.
    const auto& inspector = scene_graph.model_inspector();
    const auto& collision_pairs = inspector.GetCollisionCandidates();
    ASSERT_GT(collision_pairs.size(), 0);
    const GeometryId sphere1 = collision_pairs.begin()->first;
    ASSERT_EQ(inspector.GetName(sphere1), "two_bodies::body1_collision");

    // Add a proximity visualizer, with or without hydro.
    const std::string prefix = show_hydroelastic ? "show_hydro" : "non_hydro";
    MeshcatVisualizerParams params;
    params.role = Role::kProximity;
    params.show_hydroelastic = show_hydroelastic;
    params.prefix = prefix;
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                            params);

    // Send the geometry to Meshcat.
    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();
    diagram->ForcedPublish(*context);

    // Read back the mesh for a hydroelastic shape. Its size (in bytes) will
    // tell us whether or not hydro was used -- the normal representation is
    // just the ellipse axes (very small); the hydro reperesentation is all
    // of the tesselated faces (very large).
    const std::string data = meshcat->GetPackedObject(fmt::format(
        "/drake/{}/two_bodies/body1/{}", prefix, sphere1.get_value()));
    if (show_hydroelastic) {
      EXPECT_GT(data.size(), 5000);
    } else {
      EXPECT_LT(data.size(), 1000);
    }
  }
}

GTEST_TEST(MeshcatVisualizerTest, MultipleModels) {
  auto meshcat = std::make_shared<Meshcat>();

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
  std::string urdf = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_no_collision.urdf");
  auto iiwa0 = multibody::Parser(&plant).AddModels(urdf).at(0);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base", iiwa0).body_frame());
  auto iiwa1 = multibody::Parser(&plant, "second").AddModels(urdf).at(0);
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
  MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph.get_query_output_port(), meshcat);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  EXPECT_FALSE(meshcat->HasPath("/drake/visualizer/iiwa14"));
  EXPECT_FALSE(meshcat->HasPath("/drake/visualizer/second/iiwa14"));

  diagram->ForcedPublish(*context);

  EXPECT_TRUE(meshcat->HasPath("/drake/visualizer/iiwa14"));
  EXPECT_TRUE(meshcat->HasPath("/drake/visualizer/second/iiwa14"));
  for (int link = 0; link < 8; link++) {
    EXPECT_NE(meshcat->GetPackedTransform(
                  fmt::format("/drake/visualizer/iiwa14/iiwa_link_{}", link)),
              "");
    EXPECT_NE(meshcat->GetPackedTransform(fmt::format(
                  "/drake/visualizer/second/iiwa14/iiwa_link_{}", link)),
              "");
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
