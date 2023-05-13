#include <filesystem>
#include <future>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/sensors/image_writer.h"

namespace {
enum AsyncMode {
  /* No asynchrony; run serially. */
  kNone,
  /* Use std::async(policy=std::launch::async). */
  kTask,
  /* Use std::thread. */
  kThread,
};
std::string_view to_string(AsyncMode mode) {
  switch (mode) {
    case kNone:
      return "kNone";
    case kTask:
      return "kTask";
    case kThread:
      return "kThread";
  }
  DRAKE_UNREACHABLE();
}
}  // namespace
DRAKE_FORMATTER_AS(, , AsyncMode, x, to_string(x))

namespace fs = std::filesystem;

namespace drake {
namespace geometry {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using planning::RobotDiagramBuilder;
using render::ClippingRange;
using render::ColorRenderCamera;
using render::RenderCameraCore;
using render::RenderEngine;
using systems::Context;
using systems::Diagram;
using systems::System;
using systems::sensors::CameraInfo;
using systems::sensors::ImageRgba8U;

struct Params {
  /* Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(async_mode));
    a->Visit(DRAKE_NVP(num_workers));
    a->Visit(DRAKE_NVP(num_repeats));
  }

  AsyncMode async_mode{kNone};
  int num_workers{1};
  int num_repeats{1};

  friend std::ostream& operator<<(std::ostream& os, const Params& params) {
    os << yaml::SaveJsonString(params);
    return os;
  }
};

class ThreadTest : public testing::TestWithParam<Params> {};

TEST_P(ThreadTest, Run) {
  const auto& [async_mode, num_workers, num_repeats] = GetParam();
  DRAKE_DEMAND(num_workers > 0);
  DRAKE_DEMAND(num_repeats > 0);

  // Create the plant and scene_graph.
  auto builder = std::make_unique<RobotDiagramBuilder<double>>();

  if (async_mode != kNone) {
    // Add a never-used render engine. Because adding geometry activates the GL
    // context, the first render engine we create ends up crashing when used in
    // a separate thread, so we must build one to throw away.
    // TODO(jwnimmer-tri) Fix RenderEngineGl to remove this hazard.
    builder->scene_graph().AddRenderer("gl[dummy]", MakeRenderEngineGl());
  }

  // Prepare the cameras, one per thread.
  std::vector<std::string> renderer_names;
  for (int i = 0; i < num_workers; ++i) {
    renderer_names.push_back(fmt::format("gl[{}]", i));
    builder->scene_graph().AddRenderer(renderer_names.back(),
                                       MakeRenderEngineGl());
  }

  // The pose of the *camera* body frame C in the world frame: looking straight
  // down from 3m above the ground.
  const RigidTransformd X_WC(
      RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                      AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
      {0, 0, 3.0});
  // The pose of the *sensor* frame S in the camera body frame.
  RigidTransformd X_CS;
  const ColorRenderCamera camera_params(RenderCameraCore(
      "color", CameraInfo(640, 480, 2.0), ClippingRange(0.25, 10.0), X_CS));

  // Add model(s).
  builder->parser().AddModelsFromUrl(
      "package://drake/geometry/render/test/meshes/box.obj");

  // Finalize and create per-thread diagram contexts.
  std::unique_ptr<Diagram<double>> mutable_diagram = builder->Build();
  const Diagram<double>& const_diagram = *mutable_diagram;
  std::vector<std::unique_ptr<Context<double>>> contexts;
  for (int i = 0; i < num_workers; ++i) {
    contexts.push_back(const_diagram.CreateDefaultContext());
  }

  // The worker functor for the i'th camera.
  MatrixX<ImageRgba8U> images(num_workers, num_repeats);
  std::atomic<int> num_errors{0};
  auto work = [&images, &num_errors, num_repeats = num_repeats, &renderer_names,
               &X_WC, &camera_params, &const_diagram, &contexts](int i) {
    Context<double>& diagram_context = *contexts.at(i);
    const std::string& renderer_name = renderer_names.at(i);
    try {
      for (int j = 0; j < num_repeats; ++j) {
        // Invalidate the state to force re-calculation of the output.
        diagram_context.get_mutable_state();
        // Grab the QueryObject.
        const auto& scene_graph =
            const_diagram.GetSubsystemByName("scene_graph");
        const auto& query_object =
            scene_graph.GetOutputPort("query")
                .template Eval<QueryObject<double>>(
                    scene_graph.GetMyContextFromRoot(diagram_context));
        // Obtain our uniquely-owned RenderEngine (and so, we can const_cast).
        RenderEngine* const renderer = const_cast<RenderEngine*>(
            query_object.GetRenderEngineByName(renderer_name));
        DRAKE_THROW_UNLESS(renderer != nullptr);
        // Render an image.
        ImageRgba8U& image = images(i, j);
        image = ImageRgba8U(camera_params.core().intrinsics().width(),
                            camera_params.core().intrinsics().height());
        renderer->UpdateViewpoint(
            X_WC * camera_params.core().sensor_pose_in_camera_body());
        renderer->RenderColorImage(camera_params, &image);
      }
    } catch (std::exception& e) {
      drake::log()->error("Worker {} exception: {}", i, e.what());
      ++num_errors;
    }
  };

  // Render on multiple threads concurrently.
  switch (async_mode) {
    case kNone:
      drake::log()->info("Runnning {} workers serially", num_workers);
      break;
    case kTask:
      drake::log()->info("Launching {} std::async workers", num_workers);
      break;
    case kThread:
      drake::log()->info("Launching {} std::thread workers", num_workers);
      break;
  }
  std::vector<std::future<void>> futures;
  std::vector<std::thread> threads;
  for (int i = 0; i < num_workers; ++i) {
    switch (async_mode) {
      case kNone:
        work(i);
        break;
      case kTask:
        futures.push_back(std::async(std::launch::async, work, i));
        break;
      case kThread:
        threads.push_back(std::thread(work, i));
        break;
    }
  }

  // Wait for all workers to finish.
  for (auto& thread : threads) {
    thread.join();
  }
  for (auto& future : futures) {
    future.get();
  }

  // Save the first image for offline inspection.
  // See bazel-testlogs/geometry/render_gl/thread_test/test.outputs/output.zip.
  const std::string filename = fmt::format("image-{}.png", async_mode);
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    SaveToPng(images(0, 0), fs::path(dir) / filename);
  }

  // Check for errors.
  EXPECT_EQ(num_errors, 0);

  // Check that all output images are identical.
  const ImageRgba8U& image_00 = images(0, 0);
  for (int i = 0; i < num_workers; ++i) {
    SCOPED_TRACE(i);
    for (int j = 0; j < num_repeats; ++j) {
      SCOPED_TRACE(j);
      const ImageRgba8U& image_ij = images(i, j);
      ASSERT_EQ(image_ij, image_00);
    }
  }

  // Check that the output image has the clear color along with other color(s).
  int num_clear_pixels = 0;
  int num_box_pixels = 0;
  for (int x = 0; x < image_00.width(); ++x) {
    for (int y = 0; y < image_00.height(); ++y) {
      const uint8_t* rgba = image_00.at(x, y);
      if (std::vector<uint8_t>(rgba, rgba + 4) ==
          std::vector<uint8_t>{204, 229, 255, 255}) {
        ++num_clear_pixels;
      } else {
        ++num_box_pixels;
      }
    }
  }
  const double num_pixels = image_00.width() * image_00.height();
  EXPECT_GT(num_clear_pixels / num_pixels, 0.75);
  EXPECT_GT(num_box_pixels / num_pixels, 0.05);
}

constexpr Params kParams[] = {
    {.async_mode = kNone, .num_workers = 1, .num_repeats = 2},
    {.async_mode = kTask, .num_workers = 3, .num_repeats = 2},
    {.async_mode = kThread, .num_workers = 3, .num_repeats = 2},
};

INSTANTIATE_TEST_SUITE_P(All, ThreadTest, testing::ValuesIn(kParams));

}  // namespace
}  // namespace geometry
}  // namespace drake
