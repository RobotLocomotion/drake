#include "drake/geometry/meshcat_visualizer.h"

#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat_internal.h"
#include "drake/geometry/meshcat_types_internal.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace geometry {
namespace {

using internal::TransformGeometryName;
using multibody::AddMultibodyPlantSceneGraph;

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
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
    plant_ = &plant;
    scene_graph_ = &scene_graph;
    multibody::Parser(plant_).AddModelsFromUrl(
        "package://drake_models/iiwa_description/urdf/"
        "iiwa14_spheres_collision.urdf");
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

  void CheckVisible(const std::string& path, bool visibility) {
    ASSERT_TRUE(meshcat_->HasPath(path));
    const std::string property = meshcat_->GetPackedProperty(path, "visible");
    ASSERT_GT(property.size(), 0);
    msgpack::object_handle oh =
        msgpack::unpack(property.data(), property.size());
    auto data = oh.get().as<internal::SetPropertyData<bool>>();
    EXPECT_EQ(data.property, "visible");
    EXPECT_EQ(data.value, visibility);
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

  // Visibility remains unset until geometry gets added.
  EXPECT_EQ(meshcat_->GetPackedProperty("/drake/visualizer", "visible").size(),
            0);

  EXPECT_FALSE(meshcat_->HasPath("/drake/visualizer/iiwa14"));
  diagram_->ForcedPublish(*context_);
  EXPECT_TRUE(meshcat_->HasPath("/drake/visualizer/iiwa14"));
  for (int link = 0; link < 8; link++) {
    EXPECT_NE(meshcat_->GetPackedTransform(
                  fmt::format("/drake/visualizer/iiwa14/iiwa_link_{}", link)),
              "");
  }
  CheckVisible("/drake/visualizer", true);

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
          fmt::format("visualizer/iiwa14/iiwa_link_7/{}",
                      TransformGeometryName(geom_id, inspector))));
    }
    meshcat_->Delete();
  }

  params.role = Role::kUnassigned;
  DRAKE_EXPECT_THROWS_MESSAGE(SetUpDiagram(params), ".*Role::kUnassigned.*");
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

  // Create and run the diagram.
  SetUpDiagram(params);
  systems::Simulator<double> simulator(*diagram_);
  simulator.AdvanceTo(0.1);

  // Confirm that the path was added but was set to be invisible.
  CheckVisible("/drake/visualizer", false);
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
    const systems::EventStatus status =
        diagram_->Publish(*context_, events->get_publish_events());
    EXPECT_TRUE(status.succeeded());
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
    const systems::EventStatus status =
        diagram_->Publish(*context_, events->get_publish_events());
    EXPECT_TRUE(status.did_nothing());
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
      .get_key_frame<std::vector<double>>(0, "visualizer/iiwa14/iiwa_link_1",
                                          "position")
      .has_value();
}

TEST_F(MeshcatVisualizerWithIiwaTest, Recording) {
  MeshcatVisualizerParams params;
  SetUpDiagram(params);

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

// Confirm that the default frame rates match the publish period of the
// visualizer. Otherwise the rounding to an animation frame done in
// MeshcatAnimation can lead to odd visualization artifacts, like the first
// visualized frame not being the initial state. (Technically, it's OK to have
// the visualizer's publish period be any integer multiple of the meshcat
// recording's keyframe period; for expediency, we just test for exact
// equality.)
TEST_F(MeshcatVisualizerWithIiwaTest, RecordingFrameRate) {
  MeshcatVisualizerParams params;
  SetUpDiagram(params);

  // StartRecording via the MeshcatVisualizer API.
  visualizer_->StartRecording();
  MeshcatAnimation* animation = &meshcat_->get_mutable_recording();
  EXPECT_EQ(1.0 / animation->frames_per_second(), params.publish_period);
  visualizer_->DeleteRecording();

  // Set the animation to a different frame rate before our final test, for good
  // measure.
  meshcat_->StartRecording(12.3);
  animation = &meshcat_->get_mutable_recording();
  EXPECT_EQ(animation->frames_per_second(), 12.3);
  visualizer_->DeleteRecording();

  // StartRecording via the Meshcat API.
  meshcat_->StartRecording();
  animation = &meshcat_->get_mutable_recording();
  EXPECT_EQ(1.0 / animation->frames_per_second(), params.publish_period);
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

// When opted-in by the user, we should display the hydroelastic tessellation
// instead of the primitive shape.
GTEST_TEST(MeshcatVisualizerTest, HydroGeometry) {
  auto meshcat = std::make_shared<Meshcat>();
  for (bool show_hydroelastic : {false, true}) {
    // Load a scene with hydroelastic geometry.
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
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
    // just the ellipse axes (very small); the hydro representation is all
    // of the tessellated faces (very large).
    const std::string data = meshcat->GetPackedObject(
        fmt::format("/drake/{}/two_bodies/body1/{}", prefix,
                    TransformGeometryName(sphere1, inspector)));
    if (show_hydroelastic) {
      EXPECT_GT(data.size(), 5000);
      // The BufferGeometry has explicitly declared its material to be flat
      // shaded. The encoding includes the property name and the value \xC3 for
      // true. (False is \xC2.)
      EXPECT_THAT(data, testing::HasSubstr("flatShading\xC3")) << data;
    } else {
      EXPECT_LT(data.size(), 1000);
    }
  }
}

// When visualizing proximity geometry, if a geometry has a convex hull it is
// used in place of the geometry.
GTEST_TEST(MeshcatVisualizerTest, ConvexHull) {
  auto meshcat = std::make_shared<Meshcat>();

  // Load a scene with mesh collision geometry.
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake/geometry/render/test/box.sdf");
  plant.Finalize();

  // Dig out a GeometryId that we just loaded.
  const auto& inspector = scene_graph.model_inspector();
  const GeometryId box_id =
      inspector.GetAllGeometryIds(Role::kProximity).front();
  ASSERT_EQ(inspector.GetName(box_id), "box::collision");
  ASSERT_NE(inspector.GetConvexHull(box_id), nullptr);
  ASSERT_EQ(inspector.GetShape(box_id).type_name(), "Mesh");
  // We didn't add anything with a hydroelastic representation.
  ASSERT_TRUE(std::holds_alternative<std::monostate>(
      inspector.maybe_get_hydroelastic_mesh(box_id)));

  // Add a proximity visualizer.
  // We set show_hydroelastic to true to make sure the convex hull still comes
  // through for meshes that don't have hydro representations (see above).
  // This does *not* test the case where a mesh has both a hydro representation
  // and a convex mesh. The test criterion below (BufferGeometry) is unable to
  // distinguish between visualized hydro geometry and convex hull.
  MeshcatVisualizerParams params{.role = Role::kProximity,
                                 .show_hydroelastic = true};
  MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                          params);

  // Send the geometry to Meshcat.
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  diagram->ForcedPublish(*context);

  // Read back the mesh shape. The message would have type _meshfile_object if
  // the obj had been sent. If, however, the generated convex hull is sent, the
  // type will be BufferGeometry.
  const std::string data = meshcat->GetPackedObject(
      fmt::format("/drake/{}/box/box/{}", params.prefix,
                  TransformGeometryName(box_id, inspector)));
  EXPECT_THAT(data, testing::HasSubstr("BufferGeometry"));
}

GTEST_TEST(MeshcatVisualizerTest, MultipleModels) {
  auto meshcat = std::make_shared<Meshcat>();

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  const std::string urdf_url =
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_no_collision.urdf";
  auto iiwa0 = multibody::Parser(&plant).AddModelsFromUrl(urdf_url).at(0);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base", iiwa0).body_frame());
  auto iiwa1 =
      multibody::Parser(&plant, "second").AddModelsFromUrl(urdf_url).at(0);
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

// Use geometry properties to control which geometry is shown.
GTEST_TEST(MeshcatVisualizerTest, AcceptingProperty) {
  for (bool include_unspecified_accepting : {true, false}) {
    for (const std::string accepting : {"", "prefix", "no_match"}) {
      SCOPED_TRACE(fmt::format("include_unspecified = {}, accepting_str = {}",
                               include_unspecified_accepting,
                               accepting.empty() ? "null" : accepting.c_str()));

      // Load a simple model with one geometry.
      auto meshcat = std::make_shared<Meshcat>();
      systems::DiagramBuilder<double> builder;
      auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
      multibody::Parser(&plant).AddModelsFromUrl(
          "package://drake/geometry/render/test/box.sdf");
      plant.Finalize();

      // Add the accepting tag (if given) to the geometry.
      auto& inspector = scene_graph.model_inspector();
      const FrameId body_frame =
          plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("box").index());
      const auto geom_ids =
          inspector.GetGeometries(body_frame, Role::kIllustration);
      DRAKE_DEMAND(geom_ids.size() == 1);
      const GeometryId geom_id = *geom_ids.begin();
      const IllustrationProperties* old_props =
          scene_graph.model_inspector().GetIllustrationProperties(geom_id);
      DRAKE_DEMAND(old_props != nullptr);
      if (!accepting.empty()) {
        IllustrationProperties new_props(*old_props);
        new_props.AddProperty("meshcat", "accepting", accepting);
        scene_graph.AssignRole(*plant.get_source_id(), geom_id, new_props,
                               RoleAssign::kReplace);
      }

      // Create the visualizer.
      MeshcatVisualizerParams params;
      params.include_unspecified_accepting = include_unspecified_accepting;
      params.prefix = "prefix";
      MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                              params);
      auto diagram = builder.Build();
      auto context = diagram->CreateDefaultContext();

      // Publish geometry. Check whether the shape was published.
      const std::string geom_path = fmt::format(
          "prefix/box/box/{}", TransformGeometryName(geom_id, inspector));
      const bool should_show =
          (accepting == "prefix") ||
          (include_unspecified_accepting && accepting.empty());
      diagram->ForcedPublish(*context);
      EXPECT_EQ(meshcat->HasPath(geom_path), should_show);
    }
  }
}

// Full system acceptance test of setting alpha slider values (including the
// initial value).
TEST_F(MeshcatVisualizerWithIiwaTest, AlphaSlidersSystemCheck) {
  // Note: due to the quantizing effect of the slider, we can't set an
  // arbitrary value for the initial slider value and expect a perfect match.
  // Only values that are integer multiples of 0.02 will work.
  const MeshcatVisualizerParams params{.enable_alpha_slider = true,
                                       .initial_alpha_slider_value = 0.5};
  SetUpDiagram(params);
  systems::Simulator<double> simulator(*diagram_);

  EXPECT_EQ(meshcat_->GetSliderValue("visualizer α"), 0.5);

  // Simulate for a moment and publish to populate the visualizer.
  simulator.AdvanceTo(0.1);
  diagram_->ForcedPublish(*context_);

  meshcat_->SetSliderValue("visualizer α", 0.5);

  // Simulate and publish again to cause an update.
  simulator.AdvanceTo(0.1);
  diagram_->ForcedPublish(*context_);
}

// Tests to see if the given meshcat instance has had the "modulated_opacity"
// set for the given path. Returns the value if so, nullopt otherwise.
std::optional<double> GetOpacityProperty(const Meshcat& meshcat,
                                         const std::string& path) {
  const std::string bytes =
      meshcat.GetPackedProperty(path, "modulated_opacity");
  if (bytes.empty()) {
    return {};
  }
  msgpack::object_handle oh = msgpack::unpack(bytes.data(), bytes.size());
  auto decoded = oh.get().as<internal::SetPropertyData<double>>();
  return decoded.value;
}

// Check the effect that changing alpha sliders has on geometry opacity.
// MeshcatVisualizer now has limited logic for controlling alpha based on slider
// value -- the majority of the heavy lifting is done by meshcat.js.
// MeshcatVisualizer is responsible for initializing all of the initial alphas
// and efficiently updating after the fact. We'll be checking that the expected
// messages have been sent.
GTEST_TEST(MeshcatVisualizerTest, AlphaSliderCheckResults) {
  // Load a simple model with one geometry.
  auto meshcat = std::make_shared<Meshcat>();
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake/geometry/render/test/box.sdf");
  plant.Finalize();

  // Get the geometry id so we can create the path for the geometry.
  auto& inspector = scene_graph.model_inspector();
  const FrameId body_frame =
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("box").index());
  const auto geom_ids =
      inspector.GetGeometries(body_frame, Role::kIllustration);
  DRAKE_DEMAND(geom_ids.size() == 1);
  const GeometryId geom_id = *geom_ids.begin();
  const std::string geom_path = fmt::format(
      "visualizer/box/box/{}", TransformGeometryName(geom_id, inspector));

  // Create the visualizer.
  MeshcatVisualizerParams params;
  params.prefix = "visualizer";
  params.enable_alpha_slider = true;
  MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                          params);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // After instantiation, the first publish should initialize the modulated
  // opacity for each geometry individually with the initial value of 1.
  diagram->ForcedPublish(*context);
  const std::optional<double> init_alpha =
      GetOpacityProperty(*meshcat, geom_path);
  ASSERT_TRUE(init_alpha.has_value());
  EXPECT_EQ(*init_alpha, 1.0);

  // The opacity value started as one, attempting to redundantly "change" it to
  // the same value will do nothing.
  meshcat->SetSliderValue("visualizer α", 1.0);
  diagram->ForcedPublish(*context);
  ASSERT_FALSE(GetOpacityProperty(*meshcat, params.prefix).has_value());

  // For a somewhat arbitrary sequence of opacity values, we're confirming that
  // the slider value is always set to the "modulating_opacity" property.
  // These values must be integer multiples of 0.02 between 0.02 and 1 -- this
  // is how the slider is configured -- and 1 must not come first -- because
  // the slider value started as one, and setting it redundantly is ignored.
  for (const double slider_value : {0.2, 0.76, 1.0, 0.02}) {
    meshcat->SetSliderValue("visualizer α", slider_value);
    diagram->ForcedPublish(*context);

    // We should have dispatched a set property on the *visualizer root* with
    // the given slider value.
    const std::optional<double> mod_opacity_value =
        GetOpacityProperty(*meshcat, params.prefix);
    ASSERT_TRUE(mod_opacity_value.has_value());
    EXPECT_EQ(*mod_opacity_value, slider_value);
  }
}

GTEST_TEST(MeshcatVisualizerTest, RealtimeRate) {
  // MeshcatVisualizer doesn't compute realtime rate, but it is responsible for
  // calling Meshcat::SetSimulationTime() (which has been tested elsewhere).
  // Here we just need proof that we're doing so when we publish.
  //
  // To that end, we'll change the times in the context and force publish. We'll
  // confirm that meshcat reports that simulation time.
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  MeshcatParams meshcat_params{.realtime_rate_period = 0.125 /*seconds*/};
  auto meshcat = std::make_shared<Meshcat>(meshcat_params);
  auto& visualizer =
      MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& vis_context = visualizer.GetMyContextFromRoot(*context);

  constexpr double kT = 1.5;
  context->SetTime(kT);
  visualizer.ForcedPublish(vis_context);
  ASSERT_EQ(meshcat->GetSimulationTime(), kT);
}

TEST_F(MeshcatVisualizerWithIiwaTest, Graphviz) {
  SetUpDiagram();
  EXPECT_THAT(visualizer_->GetGraphvizString(),
              testing::HasSubstr("-> meshcat_in"));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
