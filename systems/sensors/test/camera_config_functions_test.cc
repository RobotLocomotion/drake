#include "drake/systems/sensors/camera_config_functions.h"

#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/sim_rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using drake::geometry::FrameId;
using drake::geometry::render::ColorRenderCamera;
using drake::geometry::render::DepthRenderCamera;
using drake::geometry::Rgba;
using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::FixedOffsetFrame;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::schema::Transform;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmBuses;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::sensors::ImageToLcmImageArrayT;
using drake::systems::sensors::RgbdSensor;
using drake::yaml::SaveYamlString;
using Eigen::Vector3d;

/* Simply makes a config where none of the values are defaulted. */
CameraConfig MakeConfig() {
  // These values are *supposed* to be different from the default values.
  CameraConfig config{.width = 320,
                      .height = 240,
                      .focal = CameraConfig::FocalLength{.x = 470.0},
                      .center_x = 237,
                      .center_y = 233,
                      .clipping_near = 0.075,
                      .clipping_far = 17.5,
                      .z_near = 0.15,
                      .z_far = 4.75,
                      .X_PB = Transform{RigidTransformd{Vector3d::UnitX()}},
                      .X_BC = Transform{RigidTransformd{Vector3d::UnitX()}},
                      .X_BD = Transform{RigidTransformd{Vector3d::UnitX()}},
                      .renderer_name = "test_renderer",
                      .renderer_class = "RenderEngineVtk",
                      .background = Rgba(0.25, 0.5, 0.75),
                      .name = "test_camera",
                      .fps = 17,
                      .rgb = false,
                      .depth = true,
                      .show_rgb = true,
                      .do_compress = false,
                      .lcm_bus = "test_lcm_bus"};
  // drake::scheme::Transform cannot be constructed with a base frame.
  config.X_PB.base_frame = "test_frame";
  return config;
}

/* A reality check that MakeConfig() produces a config that is *completely*
 different from the default. */
GTEST_TEST(CameraConfigTest, NonDefault) {
  const std::string full_data = SaveYamlString(MakeConfig());
  const std::string differential_data =
      SaveYamlString<CameraConfig>(MakeConfig(), {}, CameraConfig{});
  EXPECT_EQ(full_data, differential_data);
}

/* Returns a pointer to the named instance of TargetSystem (if it exists). */
template <typename TargetSystem>
const TargetSystem* GetSystem(const DiagramBuilder<double>& builder,
                              const std::string& name) {
  for (const auto* system : builder.GetSystems()) {
    if (system->get_name() == name) {
      const TargetSystem* result = dynamic_cast<const TargetSystem*>(system);
      EXPECT_NE(result, nullptr);
      return result;
    }
  }
  return nullptr;
}

class CameraConfigFunctionsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // We're not simulating -- so creating a continuous plant is fine.
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder_, 0);

    // Populate builder with sufficient stuff.
    const auto& body = plant_->AddRigidBody(
        "test_body", SpatialInertia<double>::MakeUnitary());
    body_frame_id_ = plant_->GetBodyFrameIdOrThrow(body.index());
    plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "test_frame", body, RigidTransformd()));
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  FrameId body_frame_id_;
};

/* If the CameraConfig has neither .rgb nor .depth set to true, no systems
 should be instantiated. */
TEST_F(CameraConfigFunctionsTest, EarlyExit) {
  CameraConfig config;
  // Regardless of what the default config is, we'll guarantee that we're not
  // requesting rgb or depth publication.
  config.rgb = config.depth = false;
  const int system_count = static_cast<int>(builder_.GetSystems().size());
  ApplyCameraConfig(config, &builder_);
  // We haven't added anything to the builder.
  EXPECT_EQ(builder_.GetSystems().size(), system_count);
}

// The tests below require that the default CameraConfig renders at least one
// image. This test puts a guard on that property.
GTEST_TEST(CameraConfigFunctionsTestAux, DefaultConfigRenders) {
  const CameraConfig config;
  EXPECT_TRUE(config.rgb || config.depth);
}

/* If base frame is not defined for X_PB, the sensor must be posed relative to
 the world. */
TEST_F(CameraConfigFunctionsTest, ParentBaseFrameDefaultToWorld) {
  CameraConfig config;
  config.X_PB.base_frame = std::nullopt;
  ApplyCameraConfig(config, &builder_);

  const auto* sensor =
      GetSystem<RgbdSensor>(builder_, "rgbd_sensor_preview_camera");
  ASSERT_NE(sensor, nullptr);
  EXPECT_EQ(sensor->parent_frame_id(), scene_graph_->world_frame_id());
}

/* If base frame *is* given in X_PB, the sensor must be posed relative to
 that frame. */
TEST_F(CameraConfigFunctionsTest, ParentBaseFrameSpecified) {
  CameraConfig config;
  config.X_PB.base_frame = "test_frame";
  ApplyCameraConfig(config, &builder_);

  const auto* sensor =
      GetSystem<RgbdSensor>(builder_, "rgbd_sensor_preview_camera");
  ASSERT_NE(sensor, nullptr);
  // Although we've declared it to be relative to a *frame*, there is no
  // geometry::FrameId associated with the frame. So, instead, RgbdSensor
  // references the body to which the named frame is affixed.
  EXPECT_EQ(sensor->parent_frame_id(), body_frame_id_);
  // We don't test that the camera is posed relative to the *body* frame
  // correctly because that is covered by the tests for `SimRgbdSensor`.
}

/* If base frame is given in X_PB, but does not name an existing frame, we
 throw. */
TEST_F(CameraConfigFunctionsTest, InvalidParentBaseFrame) {
  CameraConfig config;
  config.X_PB.base_frame = "invalid_frame";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyCameraConfig(config, &builder_),
      ".*invalid_frame.*");
}

/* Confirms that render engine is handled correctly.
  - If a render engine is named that hasn't been previously created, a new
    engine is added and the sensor is assigned to it.
  - If a previously existing engine is named, no new engine is created and
    the existing engine is shared.
  This is independent of image type, so, we'll use rgb. */
TEST_F(CameraConfigFunctionsTest, RenderEngine) {
  ASSERT_EQ(scene_graph_->RendererCount(), 0);

  CameraConfig config;
  ApplyCameraConfig(config, &builder_);
  ASSERT_EQ(scene_graph_->RendererCount(), 1);

  const auto* sensor1 =
      GetSystem<RgbdSensor>(builder_, "rgbd_sensor_preview_camera");
  ASSERT_NE(sensor1, nullptr);
  EXPECT_EQ(sensor1->color_render_camera().core().renderer_name(),
            config.renderer_name);
  EXPECT_EQ(sensor1->depth_render_camera().core().renderer_name(),
            config.renderer_name);

  // Now add second camera which uses the same name.
  size_t previous_system_count = builder_.GetSystems().size();
  config.name = config.name + "_the_other_one";
  ApplyCameraConfig(config, &builder_);

  // No new render engine added.
  ASSERT_EQ(scene_graph_->RendererCount(), 1);
  // New RgbdSensor added.
  EXPECT_GT(builder_.GetSystems().size(), previous_system_count);
  previous_system_count = builder_.GetSystems().size();
  const auto* sensor2 = GetSystem<RgbdSensor>(
      builder_, "rgbd_sensor_preview_camera_the_other_one");
  ASSERT_NE(sensor2, nullptr);
  EXPECT_EQ(sensor2->color_render_camera().core().renderer_name(),
            config.renderer_name);
  EXPECT_EQ(sensor2->depth_render_camera().core().renderer_name(),
            config.renderer_name);

  // Third camera uses a unique name creates a unique render engine.
  config.name = "just_for_test";
  config.renderer_name = "just_for_test";
  ApplyCameraConfig(config, &builder_);
  ASSERT_EQ(scene_graph_->RendererCount(), 2);
  // New RgbdSensor added.
  EXPECT_GT(builder_.GetSystems().size(), previous_system_count);
  const auto* sensor3 =
      GetSystem<RgbdSensor>(builder_, "rgbd_sensor_just_for_test");
  ASSERT_NE(sensor3, nullptr);
  EXPECT_EQ(sensor3->color_render_camera().core().renderer_name(),
            config.renderer_name);
  EXPECT_EQ(sensor3->depth_render_camera().core().renderer_name(),
            config.renderer_name);
}

// Confirms that all of the parameters in CameraConfig are present in the final
// configuration. This excludes the following parameters:
//  - renderer_name (tested above)
//  - X_PB.base_frame (tested above)
//  - rgb and depth (tested above)
//  - background (short of rendering, there's no easy way to observe this. But
//    it will be immediately obvious to users who specify this value if it
//    doesn't propagate.)
//  - do_compress (there's simply no easy way to observe this.)
// TODO(SeanCurtis-TRI): Actually render an image and confirm that, when
// compressed, the message size is smaller than for the uncompressed image
// and the background color is as expected.
TEST_F(CameraConfigFunctionsTest, AllParametersCount) {
  CameraConfig config = MakeConfig();
  // We just need one image type so that we get a sensor; confirm that our
  // "not default" config file has at least one image enabled.
  ASSERT_EQ(config.depth || config.rgb, true);

  // Prepare the LCM dictionary, based on the bus name from MakeConfig().
  LcmBuses lcm_buses;
  DRAKE_DEMAND(config.lcm_bus != "default");
  DrakeLcm non_default_lcm;
  lcm_buses.Add(config.lcm_bus, &non_default_lcm);

  // Add the camera and then read back its properties to confirm.
  ApplyCameraConfig(config, &builder_, &lcm_buses);
  const auto* sensor =
      GetSystem<RgbdSensor>(builder_, "rgbd_sensor_test_camera");
  ASSERT_NE(sensor, nullptr);

  const ColorRenderCamera& color = sensor->color_render_camera();
  const DepthRenderCamera& depth = sensor->depth_render_camera();

  // Camera intrinsics.
  EXPECT_EQ(color.core().intrinsics().width(), config.width);
  EXPECT_EQ(depth.core().intrinsics().width(), config.width);
  EXPECT_EQ(color.core().intrinsics().height(), config.height);
  EXPECT_EQ(depth.core().intrinsics().height(), config.height);
  EXPECT_EQ(color.core().intrinsics().focal_x(), config.focal_x());
  EXPECT_EQ(depth.core().intrinsics().focal_x(), config.focal_x());
  EXPECT_EQ(color.core().intrinsics().focal_y(), config.focal_y());
  EXPECT_EQ(depth.core().intrinsics().focal_y(), config.focal_y());
  EXPECT_EQ(color.core().intrinsics().center_x(), *config.center_x);
  EXPECT_EQ(depth.core().intrinsics().center_x(), *config.center_x);
  EXPECT_EQ(color.core().intrinsics().center_y(), *config.center_y);
  EXPECT_EQ(depth.core().intrinsics().center_y(), *config.center_y);

  // Clipping.
  EXPECT_EQ(color.core().clipping().near(), config.clipping_near);
  EXPECT_EQ(depth.core().clipping().near(), config.clipping_near);
  EXPECT_EQ(color.core().clipping().far(), config.clipping_far);
  EXPECT_EQ(depth.core().clipping().far(), config.clipping_far);

  // Depth.
  EXPECT_EQ(depth.depth_range().min_depth(), config.z_near);
  EXPECT_EQ(depth.depth_range().max_depth(), config.z_far);

  // Show window.
  EXPECT_EQ(color.show_window(), config.show_rgb);

  // Name.
  EXPECT_THAT(sensor->get_name(), ::testing::HasSubstr(config.name));

  // Render engine name.
  EXPECT_EQ(color.core().renderer_name(), config.renderer_name);
  EXPECT_EQ(depth.core().renderer_name(), config.renderer_name);

  // Publishing rate.
  const auto* publisher = GetSystem<LcmPublisherSystem>(
      builder_, "LcmPublisherSystem(DRAKE_RGBD_CAMERA_IMAGES_test_camera)");
  ASSERT_NE(publisher, nullptr);
  EXPECT_DOUBLE_EQ(publisher->get_publish_period(), 1.0 / config.fps);

  // Publishing destination.
  const DrakeLcmInterface& actual_lcm =
      const_cast<LcmPublisherSystem*>(publisher)->lcm();
  EXPECT_TRUE(&actual_lcm == &non_default_lcm);
}

// The user can pass a plant and scene_graph explicitly.
TEST_F(CameraConfigFunctionsTest, SubsystemPointers) {
  // We'll prove that the arguments are obeyed by using non-standard names,
  // and checking that nothing throws.
  plant_->set_name("Is it secret?");
  scene_graph_->set_name("Is it safe?");
  EXPECT_NO_THROW(ApplyCameraConfig(
      CameraConfig{}, &builder_, {}, plant_, scene_graph_));
}

// Confirms that if only rgb is specified, only rgb is published.
TEST_F(CameraConfigFunctionsTest, PublishingRgb) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = true;
  config.depth = false;

  ApplyCameraConfig(config, &builder_);

  // Rgb and depth ports.
  const auto* images =
      GetSystem<ImageToLcmImageArrayT>(builder_, "image_to_lcm_test_camera");
  ASSERT_NE(images, nullptr);
  EXPECT_THROW(images->GetInputPort("depth"), std::exception);
  EXPECT_NO_THROW(images->GetInputPort("rgb"));
}

// Confirms that if only depth is specified, only depth is published.
TEST_F(CameraConfigFunctionsTest, PublishingDepth) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = false;
  config.depth = true;

  ApplyCameraConfig(config, &builder_);

  // Rgb and depth ports.
  const auto* images =
      GetSystem<ImageToLcmImageArrayT>(builder_, "image_to_lcm_test_camera");
  ASSERT_NE(images, nullptr);
  EXPECT_NO_THROW(images->GetInputPort("depth"));
  EXPECT_THROW(images->GetInputPort("rgb"), std::exception);
}

// Confirms that if both rgb and depth are specified, both are published.
TEST_F(CameraConfigFunctionsTest, PublishingRgbAndDepth) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = true;
  config.depth = true;

  ApplyCameraConfig(config, &builder_);

  // Rgb and depth ports.
  const auto* images =
      GetSystem<ImageToLcmImageArrayT>(builder_, "image_to_lcm_test_camera");
  ASSERT_NE(images, nullptr);
  EXPECT_NO_THROW(images->GetInputPort("depth"));
  EXPECT_NO_THROW(images->GetInputPort("rgb"));
}

// ApplyCameraConfig doesn't have its own validation logic, but it is
// responsible for invoking validation logic. Smoke test to show that validation
// is happening -- a bad config causes the function to throw.
TEST_F(CameraConfigFunctionsTest, Validation) {
  CameraConfig config = MakeConfig();
  config.fps = -10;
  EXPECT_THROW(
      ApplyCameraConfig(config, &builder_),
      std::exception);
}

// When the user requests a non-standard LCM bus, it is an error to omit an
// LcmBuses object from the argument list.
TEST_F(CameraConfigFunctionsTest, BadLcmBus) {
  CameraConfig config;
  config.lcm_bus = "special_request";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyCameraConfig(config, &builder_),
      ".*non-default.*special_request.*");
}

// Confirms that the render engine implementation follows the requested type
// (when supported). We already know that the config gets validated, so we
// don't need to test cases where an invalid renderer_class value is passed.
// However, we do have to worry about requesting RenderEngineGl when it isn't
// available.
TEST_F(CameraConfigFunctionsTest, RenderEngineRequest) {
  // Unspecified class produces RenderEngineVtk.
  const CameraConfig default_config{.renderer_name = "default"};
  ASSERT_FALSE(scene_graph_->HasRenderer(default_config.renderer_name));
  ApplyCameraConfig(default_config, &builder_);
  ASSERT_EQ(NiceTypeName::RemoveNamespaces(scene_graph_->GetRendererTypeName(
                default_config.renderer_name)),
            "RenderEngineVtk");

  // Explicitly specifying *new* VTK class produces VTK render engine.
  const CameraConfig vtk_config{.renderer_name = "vtk_renderer",
                                .renderer_class = "RenderEngineVtk"};
  ASSERT_FALSE(scene_graph_->HasRenderer(vtk_config.renderer_name));
  ApplyCameraConfig(vtk_config, &builder_);
  ASSERT_EQ(NiceTypeName::RemoveNamespaces(scene_graph_->GetRendererTypeName(
                vtk_config.renderer_name)),
            "RenderEngineVtk");

  // Specifying the same config again should not produce a new render engine.
  int current_renderer_count = scene_graph_->RendererCount();
  ApplyCameraConfig(vtk_config, &builder_);
  EXPECT_EQ(current_renderer_count, scene_graph_->RendererCount());

  // Using an existing name but the wrong render engine type throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyCameraConfig(
          CameraConfig{.renderer_name = default_config.renderer_name,
                       .renderer_class = "RenderEngineGl"},
          &builder_),
      ".*The name is already used with a different type.+");

  // Using existing name but *no* render engine uses existing engine. Call to
  // ApplyCameraConfig doesn't throw, and the renderer count doesn't change.
  // This test assumes that this behavior doesn't depend on the type of the
  // RenderEngine.
  current_renderer_count = scene_graph_->RendererCount();
  ApplyCameraConfig(CameraConfig{.renderer_name = "vtk_renderer"}, &builder_);
  EXPECT_EQ(current_renderer_count, scene_graph_->RendererCount());

  // Now explicitly request a new RenderEngineGl -- whether it throws depends
  // on whether GL is available.
  const CameraConfig gl_config{.renderer_name = "gl_renderer",
                               .renderer_class = "RenderEngineGl"};
  if (geometry::kHasRenderEngineGl) {
    ASSERT_FALSE(scene_graph_->HasRenderer(gl_config.renderer_name));
    ApplyCameraConfig(gl_config, &builder_);
    ASSERT_EQ(NiceTypeName::RemoveNamespaces(
                  scene_graph_->GetRendererTypeName(gl_config.renderer_name)),
              "RenderEngineGl");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        ApplyCameraConfig(gl_config, &builder_),
        ".*'RenderEngineGl' is not supported.*");
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
