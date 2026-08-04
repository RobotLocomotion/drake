#include "drake/systems/sensors/camera_config_functions.h"

#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gltf_client/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/rgbd_sensor_async.h"
#include "drake/systems/sensors/sim_rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using drake::geometry::FrameId;
using drake::geometry::RenderEngineGlParams;
using drake::geometry::RenderEngineGltfClientParams;
using drake::geometry::RenderEngineVtkParams;
using drake::geometry::Rgba;
using drake::geometry::SceneGraph;
using drake::geometry::render::ColorRenderCamera;
using drake::geometry::render::DepthRenderCamera;
using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::FixedOffsetFrame;
using drake::multibody::MultibodyPlant;
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
                      .renderer_class = "RenderEngineGltfClient",
                      .background = Rgba(0.25, 0.5, 0.75),
                      .name = "test_camera",
                      .fps = 17,
                      .capture_offset = 0.001,
                      .output_delay = 0.002,
                      .rgb = false,
                      .depth = true,
                      .label = true,
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

class CameraConfigFunctionsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // We're not simulating -- so creating a continuous plant is fine.
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder_, 0);

    // Populate builder with sufficient stuff.
    const auto& body = plant_->AddRigidBody("test_body");
    body_frame_id_ = plant_->GetBodyFrameIdOrThrow(body.index());
    plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "test_frame", body, RigidTransformd()));
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
  FrameId body_frame_id_;
};

/* If the CameraConfig has none of the image types, i.e., rgb, depth, and label,
 set to true, no systems should be instantiated. */
TEST_F(CameraConfigFunctionsTest, EarlyExit) {
  CameraConfig config;
  // Regardless of what the default config is, we'll guarantee that we're not
  // requesting rgb, depth, or label publication.
  config.rgb = config.depth = config.label = false;
  const int system_count = static_cast<int>(builder_.GetSystems().size());
  ApplyCameraConfig(config, &builder_);
  // We haven't added anything to the builder.
  EXPECT_EQ(builder_.GetSystems().size(), system_count);
}

/* The tests below require that the default CameraConfig renders at least one
 image. This test puts a guard on that property. */
GTEST_TEST(CameraConfigFunctionsTestAux, DefaultConfigRenders) {
  const CameraConfig config;
  EXPECT_TRUE(config.rgb || config.depth || config.label);
}

/* If base frame is not defined for X_PB, the sensor must be posed relative to
 the world. */
TEST_F(CameraConfigFunctionsTest, ParentBaseFrameDefaultToWorld) {
  CameraConfig config;
  config.X_PB.base_frame = std::nullopt;
  ApplyCameraConfig(config, &builder_);

  const auto& sensor = builder_.GetDowncastSubsystemByName<RgbdSensor>(
      "rgbd_sensor_preview_camera");
  EXPECT_EQ(sensor.default_parent_frame_id(), scene_graph_->world_frame_id());
}

/* If base frame *is* given in X_PB, the sensor must be posed relative to
 that frame. */
TEST_F(CameraConfigFunctionsTest, ParentBaseFrameSpecified) {
  CameraConfig config;
  config.X_PB.base_frame = "test_frame";
  ApplyCameraConfig(config, &builder_);

  const auto& sensor = builder_.GetDowncastSubsystemByName<RgbdSensor>(
      "rgbd_sensor_preview_camera");
  // Although we've declared it to be relative to a *frame*, there is no
  // geometry::FrameId associated with the frame. So, instead, RgbdSensor
  // references the body to which the named frame is affixed.
  EXPECT_EQ(sensor.default_parent_frame_id(), body_frame_id_);
  // We don't test that the camera is posed relative to the *body* frame
  // correctly because that is covered by the tests for `SimRgbdSensor`.
}

/* If base frame is given in X_PB, but does not name an existing frame, we
 throw. */
TEST_F(CameraConfigFunctionsTest, InvalidParentBaseFrame) {
  CameraConfig config;
  config.X_PB.base_frame = "invalid_frame";
  DRAKE_EXPECT_THROWS_MESSAGE(ApplyCameraConfig(config, &builder_),
                              ".*invalid_frame.*");
}

/* Confirms that renderer_name is handled correctly. Specifically, it examines
 the configuration of the scene graph and diagram when a camera has successfully
 been added with a *unique* RenderEngine.

 For a non-unique RenderEngine (i.e., when CameraConfig::renderer_name names a
 previously existing engine), see RendererNameReuse.

 We'll test this on a single type of RenderEngine. */
TEST_F(CameraConfigFunctionsTest, RendererClassBasic) {
  ASSERT_EQ(scene_graph_->RendererCount(), 0);

  // This will instantiate a RenderEngineVtk.
  CameraConfig config{.renderer_name = "test_name"};
  ApplyCameraConfig(config, &builder_);
  // An engine has been added.
  ASSERT_EQ(scene_graph_->RendererCount(), 1);

  // An RgbdSensor has been added, with renderer_name set correctly.
  const auto& sensor1 = builder_.GetDowncastSubsystemByName<RgbdSensor>(
      "rgbd_sensor_preview_camera");
  EXPECT_EQ(sensor1.default_color_render_camera().core().renderer_name(),
            config.renderer_name);
  EXPECT_EQ(sensor1.default_depth_render_camera().core().renderer_name(),
            config.renderer_name);

  // Another camera with unique renderer name simply gets added.
  config.name = "just_for_test";
  config.renderer_name = "just_for_test";
  EXPECT_NO_THROW(ApplyCameraConfig(config, &builder_));
  ASSERT_EQ(scene_graph_->RendererCount(), 2);
  // New RgbdSensor added.
  const auto& sensor3 = builder_.GetDowncastSubsystemByName<RgbdSensor>(
      "rgbd_sensor_just_for_test");
  EXPECT_EQ(sensor3.default_color_render_camera().core().renderer_name(),
            config.renderer_name);
  EXPECT_EQ(sensor3.default_depth_render_camera().core().renderer_name(),
            config.renderer_name);
}

/* Exercises the various ways to specify the various RenderEngine classes - via
 class name or parameters. Simply a regression test to make sure the spellings
 work. */
TEST_F(CameraConfigFunctionsTest, RendererClassVariant) {
  int renderer_count = 0;

  // Add VTK by name.
  {
    const CameraConfig config{.renderer_name = "vtk_name",
                              .renderer_class = "RenderEngineVtk"};
    ApplyCameraConfig(config, &builder_);
    EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith("RenderEngineVtk"));
  }
  // Add VTK by parameters.
  {
    const CameraConfig config{.renderer_name = "vtk_params",
                              .renderer_class = RenderEngineVtkParams{}};
    ApplyCameraConfig(config, &builder_);
    EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith("RenderEngineVtk"));
  }

  // Add glTF client by name.
  {
    const CameraConfig config{.renderer_name = "gltf_client_name",
                              .renderer_class = "RenderEngineGltfClient"};
    ApplyCameraConfig(config, &builder_);
    EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith("RenderEngineGltfClient"));
  }
  // Add glTF client by parameters.
  {
    const CameraConfig config{.renderer_name = "gltf_client_params",
                              .renderer_class = RenderEngineGltfClientParams{}};
    ApplyCameraConfig(config, &builder_);
    EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith("RenderEngineGltfClient"));
  }

  if (geometry::kHasRenderEngineGl) {
    // Add GL by name.
    {
      const CameraConfig config{.renderer_name = "gl_name",
                                .renderer_class = "RenderEngineGl"};
      ApplyCameraConfig(config, &builder_);
      EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
      EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                  testing::EndsWith("RenderEngineGl"));
    }
    // Add GL by parameters.
    {
      const CameraConfig config{.renderer_name = "gl_params",
                                .renderer_class = RenderEngineGlParams{}};
      ApplyCameraConfig(config, &builder_);
      EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
      EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                  testing::EndsWith("RenderEngineGl"));
    }
  }
}

/* Confirms the logic for when a renderer name is reused across CameraConfig
 instances. The behavior depends on the configuration of the pre-existing
 render engine and the `renderer_class` value of the new configuration.

  - renderer_class is empty (the default value).
    - The camera uses the existing engine with the given name.
  - renderer_class simply names the same *type* of RenderEngine.
    - The camera uses the existing engine with the given name.
  - renderer_class provides parameters that match the existing engine's.
    - The camera uses the existing engine with the given name.
  - renderer_class provides the name of a *different* type of RenderEngine.
    - Throws.
  - renderer_class provides parameters for the same type, but with different
    values.
    - Throws.

 When it doesn't throw, it should instantiate appropriate systems in the
 diagram builder. */
TEST_F(CameraConfigFunctionsTest, RendererNameReuse) {
  auto perform_test = [this](const auto& parameters, const auto& mutator,
                             const std::string& renderer_class_name) {
    int renderer_count = scene_graph_->RendererCount();
    int system_count = ssize(builder_.GetSystems());

    CameraConfig config{
        .renderer_name = renderer_class_name + "_renderer",
        .renderer_class = parameters,
        .name = renderer_class_name + "_initial_params_succeeds"};
    ApplyCameraConfig(config, &builder_);
    EXPECT_EQ(scene_graph_->RendererCount(), ++renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith(renderer_class_name));
    EXPECT_GT(ssize(builder_.GetSystems()), system_count);
    system_count = ssize(builder_.GetSystems());

    // Camera config shares renderer name but has unspecified renderer_class;
    // it shares renderer with previous engine.
    config.name = renderer_class_name + "_shared_name_default_class";
    config.renderer_class = {};
    EXPECT_NO_THROW(ApplyCameraConfig(config, &builder_));
    EXPECT_EQ(scene_graph_->RendererCount(), renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith(renderer_class_name));
    EXPECT_GT(ssize(builder_.GetSystems()), system_count);
    system_count = ssize(builder_.GetSystems());

    // Camera config shares renderer name and has identical renderer_class
    // parameters; it shares renderer with previous engine.
    config.name = renderer_class_name + "_second_params_succeeds";
    EXPECT_NO_THROW(ApplyCameraConfig(config, &builder_));
    EXPECT_EQ(scene_graph_->RendererCount(), renderer_count);
    EXPECT_THAT(scene_graph_->GetRendererTypeName(config.renderer_name),
                testing::EndsWith(renderer_class_name));
    EXPECT_GT(ssize(builder_.GetSystems()), system_count);
    system_count = ssize(builder_.GetSystems());

    // Camera config shares renderer name and simply names the same renderer
    // class to the same type; it shares the previous engine.
    config.name = renderer_class_name + "_class_name_succeeds";
    config.renderer_class = renderer_class_name;
    EXPECT_NO_THROW(ApplyCameraConfig(config, &builder_));
    EXPECT_EQ(scene_graph_->RendererCount(), renderer_count);
    EXPECT_GT(ssize(builder_.GetSystems()), system_count);
    system_count = ssize(builder_.GetSystems());

    // Changing the parameters at all but keeping the same name angers the gods.
    const auto mutated_parameters = mutator(parameters);
    CameraConfig mutated_config = config;
    mutated_config.renderer_class = mutated_parameters;
    DRAKE_EXPECT_THROWS_MESSAGE(ApplyCameraConfig(mutated_config, &builder_),
                                ".*named renderer already exists and doesn't "
                                "match the given parameters*.");

    // Camera config using the name of a different class is angry.
    config.name = renderer_class_name + "_wrong_class_name_throws";
    config.renderer_class = renderer_class_name == "RenderEngineVtk"
                                ? "RenderEngineGltfClient"
                                : "RenderEngineVtk";
    DRAKE_EXPECT_THROWS_MESSAGE(
        ApplyCameraConfig(config, &builder_),
        ".*The name is already used with a different type.*.");
  };

  {
    SCOPED_TRACE("Vtk");

    perform_test(
        RenderEngineVtkParams(),
        [](const RenderEngineVtkParams& params) {
          RenderEngineVtkParams new_params(params);
          new_params.shadow_map_size += 1;
          return new_params;
        },
        "RenderEngineVtk");
  }

  {
    SCOPED_TRACE("GltfClient");
    perform_test(
        RenderEngineGltfClientParams(),
        [](const RenderEngineGltfClientParams& params) {
          RenderEngineGltfClientParams new_params(params);
          new_params.verbose = !new_params.verbose;
          return new_params;
        },
        "RenderEngineGltfClient");
  }

  if (geometry::kHasRenderEngineGl) {
    SCOPED_TRACE("Gl");
    perform_test(
        RenderEngineGlParams(),
        [](const RenderEngineGlParams& params) {
          RenderEngineGlParams new_params(params);
          new_params.default_diffuse =
              new_params.default_diffuse.scale_rgb(0.5);
          return new_params;
        },
        "RenderEngineGl");
  }
}

// TODO(SeanCurtis-TRI): We'd like to verify that the .background value is used
// when RenderEngineVtk or RenderEngineGl are specified by class name. However,
// we don't have any straightforward way to introspect a SceneGraph model
// RenderEngine. We can get one from a QueryObject, but that's quite cumbersome.
// Solving this problem also empowers solving the problem where we detect that
// a set of engine parameters matches the parameters of a previously
// instantiated engine, allowing us to relax our "parameters must come only on
// the first shared renderer name" rule.

// Confirms that all of the parameters in CameraConfig are present in the final
// configuration. This excludes the following parameters:
//  - renderer_name (tested above)
//  - X_PB.base_frame (tested above)
//  - rgb, depth, and label (tested above)
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
  ASSERT_EQ(config.depth || config.rgb || config.label, true);
  // We want a non-async camera.
  config.output_delay = 0.0;

  // Prepare the LCM dictionary, based on the bus name from MakeConfig().
  LcmBuses lcm_buses;
  DRAKE_DEMAND(config.lcm_bus != "default");
  DrakeLcm non_default_lcm;
  lcm_buses.Add(config.lcm_bus, &non_default_lcm);

  // Add the camera and then read back its properties to confirm.
  ApplyCameraConfig(config, &builder_, &lcm_buses);
  const auto& sensor = builder_.GetDowncastSubsystemByName<RgbdSensor>(
      "rgbd_sensor_test_camera");

  const ColorRenderCamera& color = sensor.default_color_render_camera();
  const DepthRenderCamera& depth = sensor.default_depth_render_camera();

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
  EXPECT_THAT(sensor.get_name(), ::testing::HasSubstr(config.name));

  // Render engine name.
  EXPECT_EQ(color.core().renderer_name(), config.renderer_name);
  EXPECT_EQ(depth.core().renderer_name(), config.renderer_name);

  // Publishing rate.
  const auto& publisher =
      builder_.GetDowncastSubsystemByName<LcmPublisherSystem>(
          "LcmPublisherSystem(DRAKE_RGBD_CAMERA_IMAGES_test_camera)");
  EXPECT_DOUBLE_EQ(publisher.get_publish_period(), 1.0 / config.fps);
  EXPECT_DOUBLE_EQ(publisher.get_publish_offset(), config.capture_offset);

  // Publishing destination.
  const DrakeLcmInterface& actual_lcm =
      const_cast<LcmPublisherSystem&>(publisher).lcm();
  EXPECT_TRUE(&actual_lcm == &non_default_lcm);
}

// Confirms that the async camera settings are all properly obeyed.
TEST_F(CameraConfigFunctionsTest, AsyncCamera) {
  CameraConfig config;
  config.name = "test_camera";
  config.fps = 10.0;
  config.capture_offset = 0.002;
  config.output_delay = 0.03;

  ApplyCameraConfig(config, &builder_);

  const auto& sensor = builder_.GetDowncastSubsystemByName<RgbdSensorAsync>(
      "rgbd_sensor_test_camera");
  EXPECT_EQ(sensor.fps(), config.fps);
  EXPECT_EQ(sensor.capture_offset(), config.capture_offset);
  EXPECT_EQ(sensor.output_delay(), config.output_delay);

  const auto& publisher =
      builder_.GetDowncastSubsystemByName<LcmPublisherSystem>(
          "LcmPublisherSystem(DRAKE_RGBD_CAMERA_IMAGES_test_camera)");
  EXPECT_EQ(publisher.get_publish_period(), 1.0 / config.fps);
  EXPECT_NEAR(publisher.get_publish_offset(),
              config.capture_offset + config.output_delay,
              // TODO(jwnimmer-tri) Once the implementation doesn't need a fudge
              // factor anymore, we should expect exact equality here.
              1e-4);
}

// The user can pass a plant and scene_graph explicitly.
TEST_F(CameraConfigFunctionsTest, SubsystemPointers) {
  // We'll prove that the arguments are obeyed by using non-standard names,
  // and checking that nothing throws.
  plant_->set_name("Is it secret?");
  scene_graph_->set_name("Is it safe?");
  EXPECT_NO_THROW(
      ApplyCameraConfig(CameraConfig{}, &builder_, {}, plant_, scene_graph_));
}

// Confirms that if only rgb is specified, only rgb is published.
TEST_F(CameraConfigFunctionsTest, PublishingRgb) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = true;
  config.depth = false;
  config.label = false;

  ApplyCameraConfig(config, &builder_);

  // Check image ports.
  const auto& images =
      builder_.GetDowncastSubsystemByName<ImageToLcmImageArrayT>(
          "image_to_lcm_test_camera");
  EXPECT_THROW(images.GetInputPort("label"), std::exception);
  EXPECT_THROW(images.GetInputPort("depth"), std::exception);
  EXPECT_NO_THROW(images.GetInputPort("rgb"));
}

// Confirms that if only depth is specified, only depth is published.
TEST_F(CameraConfigFunctionsTest, PublishingDepth) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = false;
  config.depth = true;
  config.label = false;

  ApplyCameraConfig(config, &builder_);

  // Check image ports.
  const auto& images =
      builder_.GetDowncastSubsystemByName<ImageToLcmImageArrayT>(
          "image_to_lcm_test_camera");
  EXPECT_THROW(images.GetInputPort("label"), std::exception);
  EXPECT_NO_THROW(images.GetInputPort("depth"));
  EXPECT_THROW(images.GetInputPort("rgb"), std::exception);
}

// Confirms that if only label is specified, only label is published.
TEST_F(CameraConfigFunctionsTest, PublishingLabel) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = false;
  config.depth = false;
  config.label = true;

  ApplyCameraConfig(config, &builder_);

  // Check image ports.
  const auto& images =
      builder_.GetDowncastSubsystemByName<ImageToLcmImageArrayT>(
          "image_to_lcm_test_camera");
  EXPECT_NO_THROW(images.GetInputPort("label"));
  EXPECT_THROW(images.GetInputPort("depth"), std::exception);
  EXPECT_THROW(images.GetInputPort("rgb"), std::exception);
}

// Confirms that if all the image types are specified, all are published.
TEST_F(CameraConfigFunctionsTest, PublishingRgbDepthAndLabel) {
  CameraConfig config = MakeConfig();
  config.lcm_bus = "default";
  config.rgb = true;
  config.depth = true;
  config.label = true;

  ApplyCameraConfig(config, &builder_);

  // Rgb and depth ports.
  const auto& images =
      builder_.GetDowncastSubsystemByName<ImageToLcmImageArrayT>(
          "image_to_lcm_test_camera");
  EXPECT_NO_THROW(images.GetInputPort("label"));
  EXPECT_NO_THROW(images.GetInputPort("depth"));
  EXPECT_NO_THROW(images.GetInputPort("rgb"));
}

// ApplyCameraConfig doesn't have its own validation logic, but it is
// responsible for invoking validation logic. Smoke test to show that validation
// is happening -- a bad config causes the function to throw.
TEST_F(CameraConfigFunctionsTest, Validation) {
  CameraConfig config = MakeConfig();
  config.fps = -10;
  EXPECT_THROW(ApplyCameraConfig(config, &builder_), std::exception);
}

// When the user requests a non-standard LCM bus, it is an error to omit an
// LcmBuses object from the argument list.
TEST_F(CameraConfigFunctionsTest, BadLcmBus) {
  CameraConfig config;
  config.lcm_bus = "special_request";
  DRAKE_EXPECT_THROWS_MESSAGE(ApplyCameraConfig(config, &builder_),
                              ".*non-default.*special_request.*");
}

// The user can opt-out of LCM, in which case only the camera system is added.
// LcmBuses object from the argument list.
TEST_F(CameraConfigFunctionsTest, NullLcmBus) {
  LcmBuses lcm_buses;
  DrakeLcm default_lcm(LcmBuses::kLcmUrlMemqNull);
  lcm_buses.Add("default", &default_lcm);

  // Add the camera (only).
  const CameraConfig config;
  ApplyCameraConfig(config, &builder_, &lcm_buses);

  // Check that no LCM-related objects are created.
  for (const auto* system : builder_.GetSystems()) {
    const std::string& name = system->get_name();
    // Allow the MbP basics.
    if (name == "plant" || name == "scene_graph") {
      continue;
    }
    // Allow the camera itself.
    if (name == "rgbd_sensor_preview_camera") {
      continue;
    }
    GTEST_FAIL() << name << " should not exist in the diagram";
  }
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
  ASSERT_EQ(NiceTypeName::RemoveNamespaces(
                scene_graph_->GetRendererTypeName(vtk_config.renderer_name)),
            "RenderEngineVtk");

  // Specifying the same config again should not produce a new render engine.
  int current_renderer_count = scene_graph_->RendererCount();
  ApplyCameraConfig(vtk_config, &builder_);
  EXPECT_EQ(current_renderer_count, scene_graph_->RendererCount());

  // The implementation now always attempts to instantiate the configured
  // render engine (possibly throwing it out if it's unneeded). In this case,
  // we'll fail with an "already used" error if RenderEngineGl is available and
  // "not supported" if it isn't.
  if (geometry::kHasRenderEngineGl) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        ApplyCameraConfig(
            CameraConfig{.renderer_name = default_config.renderer_name,
                         .renderer_class = "RenderEngineGl"},
            &builder_),
        ".*The name is already used with a different type.+");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        ApplyCameraConfig(
            CameraConfig{.renderer_name = default_config.renderer_name,
                         .renderer_class = "RenderEngineGl"},
            &builder_),
        ".*'RenderEngineGl' is not supported.*");
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
