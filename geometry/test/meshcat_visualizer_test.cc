#include "drake/geometry/meshcat_visualizer.h"

#include <thread>

#include <gtest/gtest.h>
#include <msgpack.hpp>

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

  void CheckVisible(const std::string& path, bool visibility) {
    ASSERT_TRUE(meshcat_->HasPath(path));
    const std::string property =
        meshcat_->GetPackedProperty(path, "visible");
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
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
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
      const std::string geom_path =
          fmt::format("prefix/box/box/{}", geom_id.get_value());
      const bool should_show =
          (accepting == "prefix") ||
          (include_unspecified_accepting && accepting.empty());
      diagram->ForcedPublish(*context);
      EXPECT_EQ(meshcat->HasPath(geom_path), should_show);
    }
  }
}

// Full system acceptance test of setting alpha slider values.
TEST_F(MeshcatVisualizerWithIiwaTest, AlphaSlidersSystemCheck) {
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

// Checks whether the given path has been set to a new color beyond it's initial
// color. If so, returns the new color; otherwise, returns nullopt.
std::optional<Rgba> GetColorProperty(const Meshcat& meshcat,
                                     const std::string& path) {
  std::string bytes = meshcat.GetPackedProperty(path, "color");
  if (bytes.empty()) {
    return {};
  }
  msgpack::object_handle oh = msgpack::unpack(bytes.data(), bytes.size());
  auto decoded = oh.get().as<internal::SetPropertyData<std::vector<double>>>();
  EXPECT_EQ(decoded.property, "color");
  EXPECT_EQ(decoded.value.size(), 4);
  if (decoded.value.size() != 4) {
    return {};
  }
  return Rgba(decoded.value[0], decoded.value[1], decoded.value[2],
              decoded.value[3]);
}

// Check the effect that changing alpha sliders has on geometry color.
GTEST_TEST(MeshcatVisualizerTest, AlphaSliderCheckResults) {
  struct Scenario {
    double geometry_alpha{};
    double slider_value{};
    double expected_value{};
  };

  std::vector<Scenario> scenarios{
    // For geometry that is not fully transparent, the alpha set by the slider
    // is geometry alpha * slider value.
    {1.0, 0.6, 0.6},
    {1.0, 1.0, 1.0},
    {0.5, 0.6, 0.6 * 0.5},
    {0.5, 1.0, 0.5},

    // For fully-transparent geometry, the alpha set by the slider is the
    // slider's value.
    {0.0, 0.6, 0.6},
    {0.0, 1.0, 1.0},

    // Note that we do not test setting sliders to 0.0 because that's outside
    // the slider range and is also orthogonal to the logic we're testing.
  };

  for (auto scenario : scenarios) {
    // Load a simple model with one geometry.
    auto meshcat = std::make_shared<Meshcat>();
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
    multibody::Parser(&plant).AddModelsFromUrl(
        "package://drake/geometry/render/test/box.sdf");
    plant.Finalize();

    // Update the single geometry's alpha to scenario.geometry_alpha.
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
    IllustrationProperties new_props(*old_props);
    new_props.UpdateProperty("phong", "diffuse", Rgba{1.0, 1.0, 1.0,
                             scenario.geometry_alpha});
    scene_graph.AssignRole(*plant.get_source_id(), geom_id, new_props,
                           RoleAssign::kReplace);

    // Create the visualizer.
    MeshcatVisualizerParams params;
    params.prefix = "visualizer";
    params.enable_alpha_slider = true;
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                            params);
    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();

    // Publish the initial geometry. The alpha slider remains at its default
    // value of 100%.
    const std::string geom_path =
        fmt::format("visualizer/box/box/{}", geom_id.get_value());
    diagram->ForcedPublish(*context);

    // If the geometry had a zero alpha, it's promoted to the default (100%).
    // If the geometry already had a non-zero alpha, it remains unchanged.
    const std::optional<Rgba> init_color =
        GetColorProperty(*meshcat, geom_path);
    if (scenario.geometry_alpha == 0.0) {
      EXPECT_TRUE(init_color.has_value());
      EXPECT_EQ(init_color.value_or(Rgba{}).a(), 1.0);
    } else {
      EXPECT_FALSE(init_color.has_value());
    }

    // If the test case specifies a <100% alpha, move the slider and republish.
    if (scenario.slider_value != 1.0) {
      meshcat->SetSliderValue("visualizer α", scenario.slider_value);
      diagram->ForcedPublish(*context);
    }

    // Check to see whether the visualizer adjusted the alpha value with a
    // SetProperty call subsequent to the original SetObject call.
    const bool requires_alpha_update =
        scenario.geometry_alpha != scenario.expected_value;
    const std::optional<Rgba> final_color =
        GetColorProperty(*meshcat, geom_path);
    EXPECT_EQ(final_color.has_value(), requires_alpha_update);
    if (final_color.has_value()) {
      EXPECT_EQ(final_color->r(), 1.0);
      EXPECT_EQ(final_color->g(), 1.0);
      EXPECT_EQ(final_color->b(), 1.0);
      EXPECT_EQ(final_color->a(), scenario.expected_value);
    }
  }
}

void Sleep(double seconds) {
  auto millis = static_cast<int64_t>(seconds * 1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

GTEST_TEST(MeshcatVisualizerTest, RealtimeRate) {
  // Set up a simulation with a visualizer. To avoid any potential ambiguity
  // around publish event timing, we'll configure the visualizer to publish at
  // 1024 Hz but we'll manually step time at 1000 Hz. This guarantees that
  // exactly one publish event has been triggered after each one of our steps
  // (as long as we don't advance past 42 ms).
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  auto meshcat = std::make_shared<Meshcat>();
  MeshcatVisualizerParams params;
  params.publish_period = 1.0 / 1024;
  auto* meshcat_visualizer = &MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat, params);
  systems::Simulator<double> simulator(builder.Build());

  // Bootstrap the realtime rate calculator.
  simulator.AdvanceTo(0.002);
  EXPECT_GT(meshcat->GetRealtimeRate(), 0.0);

  // After sleeping for much more wall time (>= 0.500) than sim time (0.001) and
  // then taking exactly one more step, the rate should be quite slow (< 1/500).
  Sleep(0.5);
  simulator.AdvanceTo(0.003);
  const double slow_rate = meshcat->GetRealtimeRate();
  EXPECT_LE(slow_rate, 0.002);

  // When we reset the calculator before stepping, the rate does not update.
  meshcat_visualizer->ResetRealtimeRateCalculator();
  simulator.AdvanceTo(0.004);
  EXPECT_EQ(meshcat->GetRealtimeRate(), slow_rate);

  // One more step causes an update. (The new reported rate will almost
  // certainly be faster than slow_rate, but we don't want to rely on the
  // kernel's details of process scheduling, or else we could be flaky.)
  simulator.AdvanceTo(0.005);
  EXPECT_NE(meshcat->GetRealtimeRate(), slow_rate);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
