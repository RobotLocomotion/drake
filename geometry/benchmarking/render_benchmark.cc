#include <unistd.h>

#include "fmt/format.h"
#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

#include "drake/common/filesystem.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace render {

/* See render_benchmark_doxygen.h for discussion of this benchmark.  */

// Friend class for accessing RenderEngine's protected/private functionality.
class RenderEngineTester {
 public:
  RenderEngineTester() = delete;

  static void SetDefaultLightPosition(const Vector3<double>& X_DL,
                                      RenderEngine* renderer) {
    renderer->SetDefaultLightPosition(X_DL);
  }
};

namespace render_benchmark {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::SaveToPng;
using systems::sensors::SaveToTiff;

DEFINE_string(save_image_path, "",
              "Enables saving rendered images in the given location");
DEFINE_bool(show_window, false, "Whether to display the rendered images");

// Default sphere array sizes.
const int kCols = 4;
const double kRadius = 0.5;
const double kZSpherePosition = -4.;

// Default camera properties.
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

class RenderEngineBenchmark : public benchmark::Fixture {
 public:
  RenderEngineBenchmark() {
    material_.AddProperty("phong", "diffuse", sphere_rgba_);
    material_.AddProperty("label", "id", RenderLabel::kDontCare);
  }

  using benchmark::Fixture::SetUp;
  void SetUp(const ::benchmark::State&) { depth_cameras_.clear(); }

  /* Set up the scene using the VTK render engine.
   @param sphere_count Number of spheres to include in the render.
   @param camera_count Number of cameras to include in the render.
   @param width Width of the render image.
   @param height Height of the render image.
   */
  void SetupVtkRender(const int sphere_count, const int camera_count,
                      const int width, const int height) {
    RenderEngineVtkParams params{{}, {}, bg_rgb_};
    renderer_ = MakeRenderEngineVtk(params);
    SetupScene(sphere_count, camera_count, width, height);
  }

  /* Parse arguments from the benchmark state.
   @return A tuple representing the sphere count, camera count, width, and
           height.  */
  static std::tuple<int, int, int, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), state.range(1), state.range(2),
                           state.range(3));
  }

  /* Helper function for generating the image path name based on the benchmark
   arguments and file format. The benchmark state is assumed to have 4 arguments
   representing the sphere count, camera count, width, and height.  */
  static std::string image_path_name(const std::string& test_name,
                                     const benchmark::State& state,
                                     const std::string& format) {
    DRAKE_DEMAND(!FLAGS_save_image_path.empty());
    filesystem::path save_path = FLAGS_save_image_path;
    auto [sphere_count, camera_count, width, height] = ReadState(state);
    return save_path.append(fmt::format("{}_{}_{}_{}_{}.{}", test_name,
                                        sphere_count, camera_count, width,
                                        height, format));
  }

  void SetupScene(const int sphere_count, const int camera_count,
                  const int width, const int height) {
    // Set up the camera to point down the z-axis.
    RigidTransformd X_WC{
        RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                        AngleAxisd(-M_PI_2, Vector3d::UnitZ())}};
    renderer_->UpdateViewpoint(X_WC);

    // Add the spheres in an array formation.
    const int rows = (sphere_count - 1) / kCols + 1;
    // By default the spheres are centered at the origin, so to center the
    // array overall, the row positions are offset by the total number of rows
    // while the column positions are offset by up to kCols.
    const double row_offset = (rows - 1) / 2.;
    const double col_offset = (std::min(sphere_count, kCols) - 1) / 2.;
    int spheres_remaining = sphere_count;
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < kCols; ++j) {
        // This break can only occur when the outer loop is on its last
        // iteration since there are no spheres remaining. Thus this break is
        // sufficient to exit out of both of these nested loops.
        if (spheres_remaining == 0) break;

        GeometryId geometry_id = GeometryId::get_new_id();
        Sphere sphere{kRadius};
        renderer_->RegisterVisual(geometry_id, sphere, material_,
                                  RigidTransformd::Identity(),
                                  true /* needs update */);
        RigidTransformd X_WV{
            Vector3d{i - row_offset, j - col_offset, kZSpherePosition}};
        poses_.insert({geometry_id, X_WV});
        --spheres_remaining;
      }
    }
    renderer_->UpdatePoses(poses_);

    // Add a background object behind the spheres for capturing shadows.
    renderer_->RegisterVisual(
        GeometryId::get_new_id(), Box(2.5, 2.5, 0.1), material_,
        RigidTransformd{Vector3d{-1, 1, kZSpherePosition - (2 * kRadius)}},
        false);

    // Add the cameras.
    for (int i = 0; i < camera_count; ++i) {
      depth_cameras_.emplace_back(RenderCameraCore{"unused" + std::to_string(i),
                                                   {width, height, kFovY},
                                                   {0.01, 100.0},
                                                   {}},
                                  DepthRange{kZNear, kZFar});
    }

    // Offset the light from its default position shared with the camera, i.e.
    // (0, 0, 1), so that shadows can be seen in the render.
    // TODO(tehbelinda): This is using a stop-gap mechanism for configuring
    // light position. Swap it to use a light declaration API when it is
    // introduced.
    RenderEngineTester::SetDefaultLightPosition(Vector3d{0.5, 0.5, 1},
                                                renderer_.get());

    // Set up the different image types.
    color_image_ = ImageRgba8U(width, height);
    depth_image_ = ImageDepth32F(width, height);
    label_image_ = ImageLabel16I(width, height);
  }

  // Keep track of the paths to the saved images. We use a static set because
  // Google Benchmark runs these benchmarks multiple times with unique instances
  // of the fixture, and we want to avoid duplicate path names.
  static std::set<std::string> saved_image_paths;

  std::unique_ptr<RenderEngine> renderer_;
  std::vector<DepthRenderCamera> depth_cameras_;
  PerceptionProperties material_;
  ImageRgba8U color_image_;
  ImageDepth32F depth_image_;
  ImageLabel16I label_image_;
  const Vector3d bg_rgb_{200 / 255., 0, 250 / 255.};
  const Eigen::Vector4d sphere_rgba_{0, 0.8, 0.5, 1};
  std::unordered_map<GeometryId, RigidTransformd> poses_;
};
std::set<std::string> RenderEngineBenchmark::saved_image_paths;

BENCHMARK_DEFINE_F(RenderEngineBenchmark, VtkColor)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  auto [sphere_count, camera_count, width, height] = ReadState(state);
  SetupVtkRender(sphere_count, camera_count, width, height);
  for (auto _ : state) {
    for (int i = 0; i < camera_count; ++i) {
      const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                        FLAGS_show_window);
      renderer_->RenderColorImage(color_cam, &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    const std::string path_name = image_path_name("VtkColor", state, "png");
    SaveToPng(color_image_, path_name);
    saved_image_paths.insert(path_name);
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, VtkColor)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})     // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({4, 1, 640, 480})     // 4 spheres, 1 camera, 640 width, 480 height.
    ->Args({8, 1, 640, 480})     // 8 spheres, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480})    // 1 sphere, 10 cameras, 640 width, 480 height.
    ->Args({1, 1, 320, 240})     // 1 sphere, 1 camera, 320 width, 240 height.
    ->Args({1, 1, 1280, 960})    // 1 sphere, 1 camera, 1280 width, 960 height.
    ->Args({1, 1, 2560, 1920});  // 1 sphere, 1 camera, 2560 width, 1920 height.

BENCHMARK_DEFINE_F(RenderEngineBenchmark, VtkDepth)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  auto [sphere_count, camera_count, width, height] = ReadState(state);
  SetupVtkRender(sphere_count, camera_count, width, height);
  for (auto _ : state) {
    for (int i = 0; i < camera_count; ++i) {
      renderer_->RenderDepthImage(depth_cameras_[i], &depth_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    const std::string path_name = image_path_name("VtkDepth", state, "tiff");
    SaveToTiff(depth_image_, path_name);
    saved_image_paths.insert(path_name);
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, VtkDepth)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})    // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480});  // 1 sphere, 10 cameras, 640 width, 480 height.

BENCHMARK_DEFINE_F(RenderEngineBenchmark, VtkLabel)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  auto [sphere_count, camera_count, width, height] = ReadState(state);
  SetupVtkRender(sphere_count, camera_count, width, height);
  for (auto _ : state) {
    for (int i = 0; i < camera_count; ++i) {
      const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                        FLAGS_show_window);
      renderer_->RenderLabelImage(color_cam, &label_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    const std::string path_name = image_path_name("VtkLabel", state, "png");
    SaveToPng(label_image_, path_name);
    saved_image_paths.insert(path_name);
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, VtkLabel)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})    // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480});  // 1 sphere, 10 cameras, 640 width, 480 height.

void Cleanup() {
  if (!RenderEngineBenchmark::saved_image_paths.empty()) {
    std::cout << "Saved rendered images to:" << std::endl;
    for (const auto& path : RenderEngineBenchmark::saved_image_paths) {
      std::cout << fmt::format(" - {}", path) << std::endl;
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_benchmark
}  // namespace render
}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  drake::geometry::render::render_benchmark::internal::Cleanup();
}
