#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_pose.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/rgbd_sensor_async.h"
#include "drake/systems/sensors/rgbd_sensor_discrete.h"

// This is an end-to-end acceptance test of the RgbdSensorAsync.
//
// More specific test cases tests of the System dynamics and API details are
// covered by rgbd_sensor_async_test.cc. In particular, the exact timings of
// capture offset and output delay are checked there; this test program is
// solely focused on the image contents, not the event timing.
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
//
// The images are written to disk for manual inspection in case of failures; see
// drake/bazel-testlogs/systems/sensors/rgbd_sensor_async_gl_test/test.outputs/.

namespace drake {
namespace systems {
namespace sensors {
namespace {

using Eigen::Vector3d;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::MakeRenderEngineGl;
using geometry::RenderEngineGlParams;
using geometry::SourceId;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using math::RigidTransform;
using math::RollPitchYaw;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::LeafSystem;
using trajectories::PiecewisePose;

// TODO(#16294) Replace this with AbstractLogSink from systems/primitives once
// that class exists.
template <typename T>
class AbstractLogSink final : public LeafSystem<T> {
 public:
  static_assert(std::is_same_v<T, double>);

  explicit AbstractLogSink(const AbstractValue& model_value) {
    this->DeclareAbstractInputPort("u0", model_value);

    // See the comment atop this file to understand the period & offset.
    const double period = 0.250;
    const double offset = 0.200;
    this->DeclarePeriodicPublishEvent(period, offset,
                                      &AbstractLogSink<T>::WriteToLog);
  }

  void WriteToLog(const Context<T>& context) const {
    const AbstractValue& value =
        this->get_input_port().template Eval<AbstractValue>(context);
    values_.emplace_back(value.Clone());
  }

  const std::vector<copyable_unique_ptr<AbstractValue>>& values() const {
    return values_;
  }

 private:
  // In general, logging events to a member field is unsuitable for the systems
  // framework, but in the narrow use case of this unit test it's fine.
  mutable std::vector<copyable_unique_ptr<AbstractValue>> values_;
};

/* Outputs the given trajectory as a FramePoseVector for use by SceneGraph. */
class FramePoseTrajectorySource final : public LeafSystem<double> {
 public:
  FramePoseTrajectorySource(FrameId frame_id, const PiecewisePose<double>& traj)
      : frame_id_(frame_id), traj_(traj) {
    this->DeclareAbstractOutputPort("frame_pose_vector",
                                    &FramePoseTrajectorySource::CalcOutput);
  }

 private:
  void CalcOutput(const Context<double>& context,
                  FramePoseVector<double>* output) const {
    *output = {{frame_id_, traj_.GetPose(context.get_time())}};
  }

  const FrameId frame_id_;
  const PiecewisePose<double> traj_;
};

constexpr char kRendererName[] = "renderer_name";

GTEST_TEST(RgbdSensorAsyncGlTest, CompareAsyncToDiscrete) {
  // Add the plant, scene_graph, and renderer.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  RenderEngineGlParams render_params;
  scene_graph.AddRenderer(kRendererName, MakeRenderEngineGl(render_params));

  // Load some models.
  Parser(&plant).AddModelsFromUrl(
      "package://drake/systems/sensors/test/"
      "rgbd_sensor_async_gl_test.dmd.yaml");
  plant.Finalize();

  // Add a moving frame for the cameras. We'll use a non-MbP frame so that it is
  // easy to prescribe the trajectory (without the need for inverse dynamics).
  // This starts out looking at the side of the bin, and then swoops up and tips
  // down to look inside.
  constexpr double start_time = 0.0;
  constexpr double end_time = 2.0;
  const RigidTransform<double> X_WC_start(RollPitchYaw<double>{0.0, 0.0, 0.0},
                                          Vector3d{-1.5, 0.0, 0.3});
  const RigidTransform<double> X_WC_end(RollPitchYaw<double>{0.0, M_PI_2, 0.0},
                                        Vector3d{0.0, 0.0, 1.5});
  const PiecewisePose<double> camera_traj = PiecewisePose<double>::MakeLinear(
      {start_time, end_time}, {X_WC_start, X_WC_end});
  const SourceId camera_kinematics_source =
      scene_graph.RegisterSource("camera_kinematics");
  const FrameId camera_kinematics_frame = scene_graph.RegisterFrame(
      camera_kinematics_source, GeometryFrame{"camera_frame"});
  const auto* camera_kinematics =
      builder.AddNamedSystem<FramePoseTrajectorySource>(
          "camera_pose", camera_kinematics_frame, camera_traj);
  builder.Connect(camera_kinematics->get_output_port(),
                  scene_graph.get_source_pose_port(camera_kinematics_source));

  // These settings are common to both cameras. Choose some arbitrary but
  // distinctive settings. (In particular, X_PB rotates us to look along +X
  // using -Z as image-down, plus a few millimeters of translational offset.)
  const FrameId parent_id = camera_kinematics_frame;
  const RigidTransform<double> X_PB(RollPitchYaw<double>{-M_PI_2, 0, -M_PI_2},
                                    Vector3d{0.001, 0.002, 0.003});
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
  // Refer to the file overview comment for details of image capture timings.
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
    log()->info("Images will be written to {}",
                "drake/bazel-testlogs/systems/sensors/"
                "rgbd_sensor_async_gl_test/test.outputs/");
  } else {
    outputs_dir = temp_directory();
    log()->info("Images will be written to {}", outputs_dir);
  }

  // Log all of the images. The 200ms offset is explained in the file overview.
  const std::string file_name_format = outputs_dir + "/{port_name}_{count}.png";
  const double image_writer_offset = 0.200;
  auto* image_writer = builder.AddSystem<ImageWriter>();
  const std::vector<std::pair<std::string, PixelType>> ports{
      {"color_image", PixelType::kRgba8U},
      {"label_image", PixelType::kLabel16I},
      {"depth_image_16u", PixelType::kDepth16U}};
  for (const auto& [port_name, pixel_type] : ports) {
    const std::string async_name = fmt::format("{}_async", port_name);
    const std::string discrete_name = fmt::format("{}_discrete", port_name);

    // Log the images to memory for test comparison.
    std::unique_ptr<AbstractValue> image;
    switch (pixel_type) {
      case PixelType::kRgba8U:
        image = std::make_unique<Value<Image<PixelType::kRgba8U>>>();
        break;
      case PixelType::kLabel16I:
        image = std::make_unique<Value<Image<PixelType::kLabel16I>>>();
        break;
      case PixelType::kDepth16U:
        image = std::make_unique<Value<Image<PixelType::kDepth16U>>>();
        break;
      default:
        GTEST_FAIL();
        return;
    }
    const auto* logger_async =
        builder.AddNamedSystem<AbstractLogSink<double>>(async_name, *image);
    const auto* logger_discrete =
        builder.AddNamedSystem<AbstractLogSink<double>>(discrete_name, *image);
    builder.Connect(sensor_async->GetOutputPort(port_name),
                    logger_async->get_input_port());
    builder.Connect(sensor_discrete->GetOutputPort(port_name),
                    logger_discrete->get_input_port());

    // Log the images to disk for offline inspection by humans.
    const auto& writer_port_async = image_writer->DeclareImageInputPort(
        pixel_type, async_name, file_name_format, 1.0 / fps,
        image_writer_offset);
    const auto& writer_port_discrete = image_writer->DeclareImageInputPort(
        pixel_type, discrete_name, file_name_format, 1.0 / fps,
        image_writer_offset);
    builder.Connect(sensor_async->GetOutputPort(port_name), writer_port_async);
    builder.Connect(sensor_discrete->GetOutputPort(port_name),
                    writer_port_discrete);
  }

  // Run the simulation.
  const auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  simulator.AdvanceTo(end_time);
  log()->info("Simulation rate was {:.3}x realtime",
              simulator.get_actual_realtime_rate());

  // Check the image outputs.
  auto image_eq = [](PixelType pixel_type, const AbstractValue& actual,
                     const AbstractValue& expected) -> bool {
    switch (pixel_type) {
      case PixelType::kRgba8U:
        return actual.template get_value<Image<PixelType::kRgba8U>>() ==
               expected.template get_value<Image<PixelType::kRgba8U>>();
      case PixelType::kLabel16I:
        return actual.template get_value<Image<PixelType::kLabel16I>>() ==
               expected.template get_value<Image<PixelType::kLabel16I>>();
        break;
      case PixelType::kDepth16U:
        return actual.template get_value<Image<PixelType::kDepth16U>>() ==
               expected.template get_value<Image<PixelType::kDepth16U>>();
      default:
        DRAKE_UNREACHABLE();
    }
  };
  for (const auto& [port_name, pixel_type] : ports) {
    const std::vector<copyable_unique_ptr<AbstractValue>>& async_values =
        diagram
            ->template GetDowncastSubsystemByName<AbstractLogSink>(
                fmt::format("{}_async", port_name))
            .values();
    const std::vector<copyable_unique_ptr<AbstractValue>>& discrete_values =
        diagram
            ->template GetDowncastSubsystemByName<AbstractLogSink>(
                fmt::format("{}_discrete", port_name))
            .values();
    ASSERT_EQ(async_values.size(), 8);
    ASSERT_EQ(discrete_values.size(), 8);
    for (int i = 0; i < 8; ++i) {
      // If anything here fails, look at the logged images to see the problem.
      SCOPED_TRACE(fmt::format("pixel_type = {}, i = {}",
                               static_cast<int>(pixel_type), i));
      // Check that the async and discrete images are identical.
      ASSERT_TRUE(image_eq(pixel_type, *async_values[i], *discrete_values[i]));
      // Check that the images are actually different every frame.
      if (i > 0) {
        ASSERT_FALSE(
            image_eq(pixel_type, *discrete_values[i], *discrete_values[i - 1]));
      }
    }
  }

  // Check the pose outputs.
  // TODO(#19198) Cross-check the pose outputs between the two sensors.
  // We can't do that today because the discrete sensor is broken.
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
