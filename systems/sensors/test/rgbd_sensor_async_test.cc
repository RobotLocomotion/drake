#include "drake/systems/sensors/rgbd_sensor_async.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

// This file contains specific tests of the System dynamics and API details, but
// mostly does not inspect the actual rendered images. Instead, an end-to-end
// acceptance test of RgbdSensorAsync lives in rgbd_sensor_async_gl_test.cc.

namespace drake {
namespace systems {
namespace sensors {
namespace {

using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::PerceptionProperties;
using geometry::SceneGraph;
using geometry::Sphere;
using geometry::internal::DummyRenderEngine;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderLabel;
using math::RigidTransform;
using multibody::AddMultibodyPlantSceneGraph;
using systems::DiagramBuilder;
using systems::EventCollection;
using systems::UnrestrictedUpdateEvent;

/* A render engine for unit testing that always outputs images that are entirely
filled with the clear color, empty label, or too-far depth measurement. */
class SimpleRenderEngine final : public DummyRenderEngine {
 public:
  static constexpr uint8_t kClearColor = 66;
  static constexpr double kClearDepth32 =
      ImageTraits<PixelType::kDepth32F>::kTooFar;
  static constexpr uint16_t kClearDepth16 =
      ImageTraits<PixelType::kDepth16U>::kTooFar;
  static const RenderLabel& kClearLabel;

  SimpleRenderEngine() { this->set_force_accept(true); }

 private:
  std::unique_ptr<RenderEngine> DoClone() const final {
    return std::make_unique<SimpleRenderEngine>(*this);
  }

  void DoRenderColorImage(const ColorRenderCamera& camera,
                          ImageRgba8U* output) const final {
    DummyRenderEngine::DoRenderColorImage(camera, output);
    *output = ImageRgba8U(camera.core().intrinsics().width(),
                          camera.core().intrinsics().height(), kClearColor);
  }

  void DoRenderDepthImage(const DepthRenderCamera& camera,
                          ImageDepth32F* output) const final {
    DummyRenderEngine::DoRenderDepthImage(camera, output);
    *output = ImageDepth32F(camera.core().intrinsics().width(),
                            camera.core().intrinsics().height(), kClearDepth32);
  }

  void DoRenderLabelImage(const ColorRenderCamera& camera,
                          ImageLabel16I* output) const final {
    DummyRenderEngine::DoRenderLabelImage(camera, output);
    *output = ImageLabel16I(camera.core().intrinsics().width(),
                            camera.core().intrinsics().height(), kClearLabel);
  }
};

const RenderLabel& SimpleRenderEngine::kClearLabel = RenderLabel::kEmpty;

constexpr char kRendererName[] = "renderer_name";

/* A test fixture for the basics of the async sensor. */
class RgbdSensorAsyncTest : public ::testing::Test {
 public:
  RgbdSensorAsyncTest()
      : ::testing::Test(),
        // N.B. This is using arbitrary yet different intrinsics for color vs.
        // depth.
        color_camera_({kRendererName, {64, 48, M_PI / 4}, {0.1, 10.0}, {}},
                      false),
        depth_camera_({kRendererName, {32, 24, M_PI / 6}, {0.1, 10.0}, {}},
                      {0.1, 10}) {}

  /* Checks that all four image output ports of the given system are producing a
  "default" image (i.e., zero width and height), and the output_time is NaN. */
  void ExpectDefaultImages(const System<double>& system,
                           const Context<double>& context) {
    SCOPED_TRACE(fmt::format("... at context.time = {}", context.get_time()));
    const auto& color_image =
        system.GetOutputPort("color_image").Eval<ImageRgba8U>(context);
    const auto& label_image =
        system.GetOutputPort("label_image").Eval<ImageLabel16I>(context);
    const auto& depth32_image =
        system.GetOutputPort("depth_image_32f").Eval<ImageDepth32F>(context);
    const auto& depth16_image =
        system.GetOutputPort("depth_image_16u").Eval<ImageDepth16U>(context);
    const Eigen::VectorXd& image_time =
        system.GetOutputPort("image_time").Eval(context);
    ASSERT_EQ(color_image.width(), 0);
    ASSERT_EQ(color_image.height(), 0);
    ASSERT_EQ(label_image.width(), 0);
    ASSERT_EQ(label_image.height(), 0);
    ASSERT_EQ(depth32_image.width(), 0);
    ASSERT_EQ(depth32_image.height(), 0);
    ASSERT_EQ(depth16_image.width(), 0);
    ASSERT_EQ(depth16_image.height(), 0);
    ASSERT_EQ(image_time.size(), 1);
    EXPECT_THAT(image_time(0), testing::IsNan());
  }

  /* Checks that all four image output ports of the given system are producing a
  "clear" image of the correct size, and the output_time is expected_time. */
  void ExpectClearImages(const System<double>& system,
                         const Context<double>& context, double expected_time) {
    SCOPED_TRACE(fmt::format("... at context.time = {}", context.get_time()));

    const auto& color_image =
        system.GetOutputPort("color_image").Eval<ImageRgba8U>(context);
    const auto& label_image =
        system.GetOutputPort("label_image").Eval<ImageLabel16I>(context);
    const auto& depth32_image =
        system.GetOutputPort("depth_image_32f").Eval<ImageDepth32F>(context);
    const auto& depth16_image =
        system.GetOutputPort("depth_image_16u").Eval<ImageDepth16U>(context);
    const Eigen::VectorXd& image_time =
        system.GetOutputPort("image_time").Eval(context);

    const auto& color_intrinsics = color_camera_.core().intrinsics();
    ASSERT_EQ(color_image.width(), color_intrinsics.width());
    ASSERT_EQ(color_image.height(), color_intrinsics.height());
    ASSERT_EQ(label_image.width(), color_intrinsics.width());
    ASSERT_EQ(label_image.height(), color_intrinsics.height());

    const auto& depth_intrinsics = depth_camera_.core().intrinsics();
    ASSERT_EQ(depth32_image.width(), depth_intrinsics.width());
    ASSERT_EQ(depth32_image.height(), depth_intrinsics.height());
    ASSERT_EQ(depth16_image.width(), depth_intrinsics.width());
    ASSERT_EQ(depth16_image.height(), depth_intrinsics.height());

    auto to_vector = [](const auto& image) {
      return std::vector(image.at(0, 0), image.at(0, 0) + image.size());
    };
    ASSERT_THAT(to_vector(color_image),
                ::testing::Each(SimpleRenderEngine::kClearColor));
    ASSERT_THAT(to_vector(label_image),
                ::testing::Each(SimpleRenderEngine::kClearLabel));
    ASSERT_THAT(to_vector(depth32_image),
                ::testing::Each(SimpleRenderEngine::kClearDepth32));
    ASSERT_THAT(to_vector(depth16_image),
                ::testing::Each(SimpleRenderEngine::kClearDepth16));

    ASSERT_EQ(image_time.size(), 1);
    EXPECT_EQ(image_time(0), expected_time);
  }

 protected:
  ColorRenderCamera color_camera_;
  DepthRenderCamera depth_camera_;
};

TEST_F(RgbdSensorAsyncTest, ConstructorAndSimpleAccessors) {
  SceneGraph<double> scene_graph;
  const FrameId parent_id = SceneGraph<double>::world_frame_id();
  const RigidTransform<double> X_PB(Eigen::Vector3d(1, 2, 3));
  const double fps = 4;
  const double capture_offset = 0.001;
  const double output_delay = 0.200;
  const bool render_label_image = true;
  const RgbdSensorAsync dut(&scene_graph, parent_id, X_PB, fps, capture_offset,
                            output_delay, color_camera_, depth_camera_,
                            render_label_image);

  EXPECT_EQ(dut.parent_id(), parent_id);
  EXPECT_EQ(dut.X_PB().GetAsMatrix34(), X_PB.GetAsMatrix34());
  EXPECT_EQ(dut.fps(), fps);
  EXPECT_EQ(dut.capture_offset(), capture_offset);
  EXPECT_EQ(dut.output_delay(), output_delay);
  EXPECT_TRUE(dut.color_camera().has_value());
  EXPECT_TRUE(dut.depth_camera().has_value());

  EXPECT_EQ(dut.get_input_port().get_name(), "geometry_query");
  EXPECT_EQ(dut.color_image_output_port().get_name(), "color_image");
  EXPECT_EQ(dut.depth_image_32F_output_port().get_name(), "depth_image_32f");
  EXPECT_EQ(dut.depth_image_16U_output_port().get_name(), "depth_image_16u");
  EXPECT_EQ(dut.label_image_output_port().get_name(), "label_image");
  EXPECT_EQ(dut.body_pose_in_world_output_port().get_name(),
            "body_pose_in_world");

  Simulator<double> simulator(dut);
  simulator.Initialize();
}

// Confirm that we fail-fast when geometry changes post-initialze.
TEST_F(RgbdSensorAsyncTest, GeometryVersionFailFast) {
  // Prepare a full simulation.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0);
  plant.Finalize();
  scene_graph.AddRenderer(kRendererName,
                          std::make_unique<SimpleRenderEngine>());
  const FrameId world = SceneGraph<double>::world_frame_id();
  const RigidTransform<double> X;
  const double fps = 4;
  const double capture_offset = 0.001;
  const double output_delay = 0.200;
  const auto* dut = builder.AddSystem<RgbdSensorAsync>(
      &scene_graph, world, X, fps, capture_offset, output_delay, color_camera_,
      std::nullopt);
  builder.Connect(scene_graph.get_query_output_port(), dut->get_input_port());
  Simulator<double> simulator(builder.Build());

  // Trigger the initialiation event. This sets up the Worker.
  simulator.Initialize();

  // Add a new geometry to the context after initialization.
  Context<double>* scene_graph_context =
      &scene_graph.GetMyMutableContextFromRoot(
          &simulator.get_mutable_context());
  const GeometryId geometry_id = scene_graph.RegisterGeometry(
      scene_graph_context, plant.get_source_id().value(), world,
      std::make_unique<GeometryInstance>(X, Sphere(0.1), "sphere"));
  PerceptionProperties properties;
  properties.UpdateProperty("label", "id", RenderLabel(1));
  scene_graph.AssignRole(scene_graph_context, plant.get_source_id().value(),
                         geometry_id, properties);

  // Simulating again is an error (for now). Ideally, we'd adapt to the new
  // geometry but for now at least we fail-fast.
  DRAKE_EXPECT_THROWS_MESSAGE(simulator.AdvanceTo(0.002),
                              ".*must manually call.*Initialize.*");
}

TEST_F(RgbdSensorAsyncTest, ConstructorColorOnly) {
  SceneGraph<double> scene_graph;
  const FrameId parent_id = SceneGraph<double>::world_frame_id();
  const RigidTransform<double> X_PB;
  const double fps = 4;
  const double capture_offset = 0.001;
  const double output_delay = 0.200;
  const bool render_label_image = false;
  const RgbdSensorAsync dut(&scene_graph, parent_id, X_PB, fps, capture_offset,
                            output_delay, color_camera_, std::nullopt,
                            render_label_image);

  EXPECT_TRUE(dut.color_camera().has_value());
  EXPECT_NO_THROW(dut.color_image_output_port());

  EXPECT_FALSE(dut.depth_camera().has_value());
  EXPECT_THROW(dut.depth_image_32F_output_port(), std::exception);
  EXPECT_THROW(dut.depth_image_16U_output_port(), std::exception);
  EXPECT_THROW(dut.label_image_output_port(), std::exception);

  Simulator<double> simulator(dut);
  simulator.Initialize();
}

TEST_F(RgbdSensorAsyncTest, ConstructorDepthOnly) {
  SceneGraph<double> scene_graph;
  const FrameId parent_id = SceneGraph<double>::world_frame_id();
  const RigidTransform<double> X_PB;
  const double fps = 4;
  const double capture_offset = 0.001;
  const double output_delay = 0.200;
  const bool render_label_image = false;
  const RgbdSensorAsync dut(&scene_graph, parent_id, X_PB, fps, capture_offset,
                            output_delay, std::nullopt, depth_camera_,
                            render_label_image);

  EXPECT_TRUE(dut.depth_camera().has_value());
  EXPECT_NO_THROW(dut.depth_image_32F_output_port());
  EXPECT_NO_THROW(dut.depth_image_16U_output_port());

  EXPECT_FALSE(dut.color_camera().has_value());
  EXPECT_THROW(dut.color_image_output_port(), std::exception);
  EXPECT_THROW(dut.label_image_output_port(), std::exception);

  Simulator<double> simulator(dut);
  simulator.Initialize();
}

// Sanity check the basic plumbing and event scheduling by rendering an empty
// scene. Initially there is no image output because time has not advanced far
// enough yet, so all output ports evaluate to their default value (i.e., all
// pixels are zero). After enough time has passed, the output port evaluation
// will provide rendered images filled with a uniform non-zero background color,
// i.e., the "clear" color.
TEST_F(RgbdSensorAsyncTest, RenderBackgroundColor) {
  // Add the plant, scene_graph, and renderer.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0);
  scene_graph.AddRenderer(kRendererName,
                          std::make_unique<SimpleRenderEngine>());

  // Add the RgbdSensorAsync and export all of its output ports.
  const FrameId parent_id = SceneGraph<double>::world_frame_id();
  const RigidTransform<double> X_PB(Eigen::Vector3d(1, 2, 3));
  const double fps = 4;
  const double capture_offset = 0.001;
  const double output_delay = 0.200;
  const bool render_label_image = true;
  const auto* dut = builder.AddSystem<RgbdSensorAsync>(
      &scene_graph, parent_id, X_PB, fps, capture_offset, output_delay,
      color_camera_, depth_camera_, render_label_image);
  builder.Connect(scene_graph.get_query_output_port(), dut->get_input_port());
  for (OutputPortIndex i{0}; i < dut->num_output_ports(); ++i) {
    const auto& output_port = dut->get_output_port(i);
    builder.ExportOutput(output_port, output_port.get_name());
  }

  // Prepare the simulation.
  plant.Finalize();
  Simulator<double> simulator(builder.Build());

  // The dut has two main kinds of events, 'tick' and 'tock'. The tick events
  // snapshot the geometry kinematics and then launch the async render task. The
  // tock events wait for the async render task to finish and then set the
  // output port to the new image.
  //
  // In this test we'll Start the simulation at time == 100ms to cover the case
  // where the first tock event happens without any prior tick event. Here's an
  // event summary:
  //
  // -   1ms scheduled tick event, but we've set up simulator to skip over this
  // - 100ms initialize event (this is the start time of simulation)
  // - 201ms tock event, but there is no async render task yet so it's a no-op
  // - 251ms tick event, launches async render task
  // - 451ms tock event, sets the output images
  simulator.get_mutable_context().SetTime(0.100);

  // Prior to initialization, the output image is empty.
  ExpectDefaultImages(simulator.get_system(), simulator.get_context());

  // The output image is still empty as we pass through various initialization
  // and update events.
  for (double time : {0.101, 0.202, 0.252, 0.450}) {
    simulator.AdvanceTo(time);
    ExpectDefaultImages(simulator.get_system(), simulator.get_context());
  }

  // After the second tock event, we see the engine's "clear" color.
  simulator.AdvanceTo(0.452);
  ExpectClearImages(simulator.get_system(), simulator.get_context(), 0.251);

  // Likewise, the pose port finally has the correct value.
  EXPECT_EQ(simulator.get_system()
                .GetOutputPort("body_pose_in_world")
                .Eval<RigidTransform<double>>(simulator.get_context())
                .translation(),
            X_PB.translation());

  // Nuke the state back to default so that we can test the recovery logic for a
  // missing initialization event during a *tick*.
  // - 451ms reset state to default; therefore the output is default as well
  // - 501ms tick; output is still default
  // - 702ms tock; output is back to clear
  simulator.get_mutable_context().get_mutable_state().SetFrom(
      simulator.get_system().CreateDefaultContext()->get_state());
  ExpectDefaultImages(simulator.get_system(), simulator.get_context());
  simulator.AdvanceTo(0.502);
  ExpectDefaultImages(simulator.get_system(), simulator.get_context());
  simulator.AdvanceTo(0.702);
  ExpectClearImages(simulator.get_system(), simulator.get_context(), 0.501);

  // Nuke the state back to default so that we can test the recovery logic for a
  // missing initialization event during a *tock*.
  // - 751ms tick
  // - 752ms reset state to default; therefore the output is default as well
  // - 952ms tock; output is still default (no tick)
  // - 1001ms tick; output is still default
  // - 1201ms tock; output is finally back to clear
  simulator.AdvanceTo(0.752);
  simulator.get_mutable_context().get_mutable_state().SetFrom(
      simulator.get_system().CreateDefaultContext()->get_state());
  for (double time : {0.952, 1.002}) {
    simulator.AdvanceTo(time);
    ExpectDefaultImages(simulator.get_system(), simulator.get_context());
  }
  simulator.AdvanceTo(1.202);
  ExpectClearImages(simulator.get_system(), simulator.get_context(), 1.001);

  // Now the wheels really come off the cart. Advance time to fire a *tick*,
  // then fast forward over the matching tock, then manually fire the next tick
  // to avoid the Simulator interlock that a call to Initialize must happen
  // inbetween SetTime and AdranceTo. With all these hijinks, a user would
  // probably never encounter this, but it's a necessary test case to cover the
  // error handling logic for two Worker::Start() calls back-to-back.
  // - 1251ms tick
  // - 1500ms new set time (skipping the tock at 1451ms)
  // - 1501ms tick
  simulator.AdvanceTo(1.252);
  simulator.get_mutable_context().SetTime(1.500);
  auto events = simulator.get_system().AllocateCompositeEventCollection();
  double next_time = simulator.get_system().CalcNextUpdateTime(
      simulator.get_context(), events.get());
  EXPECT_EQ(next_time, 1.501);
  const EventCollection<UnrestrictedUpdateEvent<double>>& update_events =
      events->get_unrestricted_update_events();
  EXPECT_TRUE(update_events.HasEvents());
  auto next_state_out = simulator.get_context().CloneState();
  EXPECT_TRUE(simulator.get_system()
                  .CalcUnrestrictedUpdate(simulator.get_context(),
                                          update_events, next_state_out.get())
                  .succeeded());
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
