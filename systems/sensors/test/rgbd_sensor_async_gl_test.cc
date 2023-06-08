#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/rgbd_sensor_async.h"

// This is an end-to-end acceptance test of the RgbdSensorAsync.
//
// We run a representative simulation with a discrete sensor alongside the async
// sensor, with both sensors using the same fps. We presume that the discrete
// sensor is correct (from its own unit tests). Other than the configured output
// delay, the streams of images from the async sensor should be identical to the
// discrete sensor.
//
// The discrete sensor is configured to sample and hold at these times:
//  Sample
//    0 ms
//  250 ms
//  500 ms
//  750 ms
//     etc
//
// The async sensor is configured to sample and output at these times:
//  Sample  Output
//    0 ms   125ms
//  250 ms   375ms
//  500 ms   625ms
//  750 ms   875ms
//     etc
//
// Therefore, if we log the image output ports at 200ms, 450ms, etc. they should
// be identical for both sensors.

namespace drake {
namespace systems {
namespace sensors {
namespace {

using drake::multibody::AddMultibodyPlantSceneGraph;
using geometry::FrameId;
using geometry::MakeRenderEngineGl;
using geometry::RenderEngineGlParams;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using math::RigidTransform;
using systems::DiagramBuilder;

constexpr char kRendererName[] = "renderer_name";

GTEST_TEST(RgbdSensorAsyncGlTest, CompareAsyncToDiscrete) {
  // Add the plant, scene_graph, and renderer.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0);
  RenderEngineGlParams render_params;
  scene_graph.AddRenderer(kRendererName, MakeRenderEngineGl(render_params));

  // XXX Load some stuff.
  plant.Finalize();

  // Attach the cameras to a moving frame using a non-identity transform.
  // XXX use a moving frame.
  const FrameId parent_id = geometry::SceneGraph<double>::world_frame_id();
  const RigidTransform<double> X_PB(Eigen::Vector3d{0.1, -0.2, 0.3});

  // These settings are common to both cameras.
  // Choose some arbitrary but distinctive settings.
  const ColorRenderCamera color_camera(
      {kRendererName, {320, 240, M_PI / 4}, {0.1, 10.0}, {}});
  const DepthRenderCamera depth_camera(
      {kRendererName, {160, 120, M_PI / 6}, {0.1, 10.0}, {}}, {0.1, 10});
  const double fps = 4.0;
  const bool render_label_image = true;

  // Add the async sensor (the device under test).
  const double async_capture_offset = 0.0;
  const double async_output_delay = 0.125;
  const auto* sensor_async = builder.AddSystem<RgbdSensorAsync>(
      &scene_graph, parent_id, X_PB, fps, async_capture_offset,
      async_output_delay, color_camera, depth_camera, render_label_image);
  builder.Connect(scene_graph.get_query_output_port(),
                  sensor_async->get_input_port());

  // Add the non-async (discrete) sensor that will provide our reference images.
  // It will run at 8 Hz, capturing images at the following times:
  const double discrete_period = 1.0 / fps;
  const auto* sensor_discrete = builder.AddSystem<RgbdSensorDiscrete>(
      std::make_unique<RgbdSensor>(parent_id, X_PB, color_camera, depth_camera),
      discrete_period, render_label_image);
  builder.Connect(scene_graph.get_query_output_port(),
                  sensor_discrete->query_object_input_port());

  // Prepare the output directory.
  std::string outputs_dir;
  if (const char* bazel_dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    outputs_dir = bazel_dir;
  } else {
    outputs_dir = temp_directory();
    log()->info("Images will be written to {}", outputs_dir);
  }

  // Log all of the images. The 200ms offset is explained in the file overview.
  const std::string file_name_format = outputs_dir + "/{port_name}_{count}.png";
  const double image_writer_offset = 0.200;
  auto* image_writer = builder.AddSystem<ImageWriter>();
  auto connect_writer = [&](std::string port_name, PixelType pixel_type) {
    const auto& writer_port_async = image_writer->DeclareImageInputPort(
        pixel_type, fmt::format("{}_async", port_name), file_name_format,
        1.0 / fps, image_writer_offset);
    const auto& writer_port_discrete = image_writer->DeclareImageInputPort(
        pixel_type, fmt::format("{}_discrete", port_name), file_name_format,
        1.0 / fps, image_writer_offset);
    builder.Connect(sensor_async->GetOutputPort(port_name), writer_port_async);
    builder.Connect(sensor_discrete->GetOutputPort(port_name),
                    writer_port_discrete);
  };
  connect_writer("color_image", PixelType::kRgba8U);
  connect_writer("label_image", PixelType::kLabel16I);
  connect_writer("depth_image_16u", PixelType::kDepth16U);

  // Run the simulation.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(2.0);

  // XXX check the image outputs.
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
