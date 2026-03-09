#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/render_gl/factory.h"
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
using render::ClippingRange;
using render::ColorRenderCamera;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
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

// This test case directly calls into RenderEngineGl to create images, without
// relying on the SceneGraph and/or GeometryState infrastructure.
TEST_P(ThreadTest, CallRenderEngineDirectly) {
  const auto& [async_mode, num_workers, num_repeats_unpacked] = GetParam();
  const int num_repeats = num_repeats_unpacked;  // Required for lambda capture.
  DRAKE_DEMAND(num_workers > 0);
  DRAKE_DEMAND(num_repeats > 0);

  // Prepare the geometry.
  const GeometryInstance instance(
      RigidTransformd{},
      Mesh(FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj"), 1),
      "box");
  const PerceptionProperties perception_properties = []() {
    PerceptionProperties result;
    result.AddProperty("label", "id", RenderLabel::kDontCare);
    return result;
  }();

  // Prepare the camera settings.
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

  // The worker functor for the i'th camera.
  MatrixX<ImageRgba8U> images(num_workers, num_repeats);
  std::atomic<int> num_errors{0};
  auto work = [&images, &num_errors, num_repeats, &instance,
               &perception_properties, &X_WC, &camera_params](int i) {
    try {
      // TODO(jwnimmer-tri) For the test to pass, we need to create a separate
      // engine within each worker thread. We should try to adjust the engine
      // so that we can call a clone of one canonical engine instead of making
      // new ones from scratch.
      std::unique_ptr<RenderEngine> renderer = MakeRenderEngineGl();
      renderer->RegisterVisual(instance.id(), instance.shape(),
                               perception_properties, instance.pose(),
                               false /* needs update */);
      for (int j = 0; j < num_repeats; ++j) {
        ImageRgba8U& image = images(i, j);
        image = ImageRgba8U(camera_params.core().intrinsics().width(),
                            camera_params.core().intrinsics().height());
        renderer->UpdateViewpoint(
            X_WC * camera_params.core().sensor_pose_in_camera_body());
        renderer->RenderColorImage(camera_params, &image);
      }
    } catch (std::exception& e) {
      // Letting an exception leak out of a std::thread leads to obscure error
      // messages. Catching, logging, and tallying is a much nicer outcome.
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
    {.async_mode = kNone, .num_workers = 3, .num_repeats = 2},
    {.async_mode = kTask, .num_workers = 3, .num_repeats = 2},
    {.async_mode = kThread, .num_workers = 3, .num_repeats = 2},
};

INSTANTIATE_TEST_SUITE_P(All, ThreadTest, testing::ValuesIn(kParams));

}  // namespace
}  // namespace geometry
}  // namespace drake
