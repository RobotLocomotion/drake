#include <unistd.h>

#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/render/render_engine_ospray_factory.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace render_benchmark {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::Box;
using geometry::Sphere;
using geometry::render::DepthCameraProperties;
using geometry::render::MakeRenderEngineOspray;
using geometry::render::MakeRenderEngineVtk;
using geometry::render::OsprayMode;
using geometry::render::RenderEngine;
using geometry::render::RenderEngineOsprayParams;
using geometry::render::RenderEngineVtkParams;
using geometry::render::RenderLabel;
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
DEFINE_int32(samples_per_pixel, 1,
             "Number of illumination samples per pixel when path tracing");

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
  void SetUp(const ::benchmark::State&) { cameras_.clear(); }

  using benchmark::Fixture::TearDown;
  void TearDown(const ::benchmark::State&) {}

  /** Set up the scene using the VTK render engine.
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

  /** Set up the scene using the Ospray render engine.
   @param sphere_count Number of spheres to include in the render.
   @param camera_count Number of cameras to include in the render.
   @param width Width of the render image.
   @param height Height of the render image.
   @param mode The OsprayMode, i.e. ray tracing or path tracing.
   @param use_shadows Whether to render shadows when in ray tracing mode
                      (`OsprayMode::kRayTracer`). Ignored in path tracing mode.
   */
  void SetupOsprayRender(const int sphere_count, const int camera_count,
                         const int width, const int height, OsprayMode mode,
                         bool use_shadows) {
    RenderEngineOsprayParams params{
        mode, {}, bg_rgb_, FLAGS_samples_per_pixel, use_shadows};
    renderer_ = MakeRenderEngineOspray(params);
    SetupScene(sphere_count, camera_count, width, height);
  }

  // Helper function for generating the image path name based on the benchmark
  // arguments and file format.
  static std::string image_path_name(const std::string& test_name,
                                     const benchmark::State& state,
                                     const std::string& format) {
    filesystem::path save_path = FLAGS_save_image_path.empty()
                                     ? temp_directory()
                                     : FLAGS_save_image_path;
    return save_path.append(test_name + "_" +
                            std::to_string(state.range(0)) + "_" +
                            std::to_string(state.range(1)) + "_" +
                            std::to_string(state.range(2)) + "_" +
                            std::to_string(state.range(3)) + "." + format);
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
      cameras_.emplace_back(width, height, kFovY, "unused" + std::to_string(i),
                            kZNear, kZFar);
    }

    // Offset the light from it's default position shared with the camera, i.e.
    // (0, 0, 1), so that shadows can be seen in the render. Currently there is
    // no light declaration support so we need to configure it manually.
    renderer_->SetDefaultLightPosition(Vector3d{0.5, 0.5, 1});

    // Set up the different image types.
    color_image_ = ImageRgba8U(width, height);
    depth_image_ = ImageDepth32F(width, height);
    label_image_ = ImageLabel16I(width, height);
  }

  std::unique_ptr<RenderEngine> renderer_;
  std::vector<DepthCameraProperties> cameras_;
  PerceptionProperties material_;
  ImageRgba8U color_image_;
  ImageDepth32F depth_image_;
  ImageLabel16I label_image_;
  const Vector3d bg_rgb_{200 / 255., 0, 250 / 255.};
  const Eigen::Vector4d sphere_rgba_{0, 0.8, 0.5, 1};
  std::unordered_map<GeometryId, RigidTransformd> poses_;
};

BENCHMARK_DEFINE_F(RenderEngineBenchmark, VtkColor)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupVtkRender(state.range(0), state.range(1), state.range(2),
                 state.range(3));
  for (auto _ : state) {
    for (int i = 0; i < state.range(1); ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    SaveToPng(color_image_, image_path_name("VtkColor", state, "png"));
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
  SetupVtkRender(state.range(0), state.range(1), state.range(2),
                 state.range(3));
  for (auto _ : state) {
    for (int i = 0; i < state.range(1); ++i) {
      renderer_->RenderDepthImage(cameras_[i], &depth_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    SaveToTiff(depth_image_, image_path_name("VtkDepth", state, "tiff"));
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, VtkDepth)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})    // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480});  // 1 sphere, 10 cameras, 640 width, 480 height.

BENCHMARK_DEFINE_F(RenderEngineBenchmark, VtkLabel)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupVtkRender(state.range(0), state.range(1), state.range(2),
                 state.range(3));
  for (auto _ : state) {
    for (int i = 0; i < state.range(1); ++i) {
      renderer_->RenderLabelImage(cameras_[i], FLAGS_show_window,
                                  &label_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    SaveToPng(label_image_, image_path_name("VtkLabel", state, "png"));
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, VtkLabel)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})    // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480});  // 1 sphere, 10 cameras, 640 width, 480 height.

BENCHMARK_DEFINE_F(RenderEngineBenchmark, OsprayRayColor)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupOsprayRender(state.range(0), state.range(1), state.range(2),
                    state.range(3), OsprayMode::kRayTracer, true);
  for (auto _ : state) {
    for (int i = 0; i < state.range(1); ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    SaveToPng(color_image_, image_path_name("OsprayRayColor", state, "png"));
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, OsprayRayColor)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})     // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({4, 1, 640, 480})     // 4 spheres, 1 camera, 640 width, 480 height.
    ->Args({8, 1, 640, 480})     // 8 spheres, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480})    // 1 sphere, 10 cameras, 640 width, 480 height.
    ->Args({1, 1, 320, 240})     // 1 sphere, 1 camera, 320 width, 240 height.
    ->Args({1, 1, 1280, 960})    // 1 sphere, 1 camera, 1280 width, 960 height.
    ->Args({1, 1, 2560, 1920});  // 1 sphere, 1 camera, 2560 width, 1920 height.

BENCHMARK_DEFINE_F(RenderEngineBenchmark, OsprayRayColorShadowsOff)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupOsprayRender(state.range(0), state.range(1), state.range(2),
                    state.range(3), OsprayMode::kRayTracer, false);
  for (auto _ : state) {
    for (int i = 0; i < state.range(1); ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    SaveToPng(color_image_,
              image_path_name("OsprayRayColorShadowsOff", state, "png"));
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, OsprayRayColorShadowsOff)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})    // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({4, 1, 640, 480})    // 4 spheres, 1 camera, 640 width, 480 height.
    ->Args({8, 1, 640, 480})    // 8 spheres, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480});  // 1 sphere, 10 cameras, 640 width, 480 height.

BENCHMARK_DEFINE_F(RenderEngineBenchmark, OsprayPathColor)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupOsprayRender(state.range(0), state.range(1), state.range(2),
                    state.range(3), OsprayMode::kPathTracer, true);
  for (auto _ : state) {
    // NOTE: The ospray renderer has a quirk; if no poses update subsequent
    // render passes seem to accumulate into the same frame buffer, improving
    // the image quality the same as simply doing more passes.
    renderer_->UpdatePoses(poses_);
    for (int i = 0; i < state.range(1); ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    SaveToPng(color_image_, image_path_name("OsprayPathColor", state, "png"));
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, OsprayPathColor)
    ->Unit(benchmark::kMillisecond)
    ->Args({1, 1, 640, 480})    // 1 sphere, 1 camera, 640 width, 480 height.
    ->Args({1, 10, 640, 480});  // 1 sphere, 10 cameras, 640 width, 480 height.

}  // namespace
}  // namespace internal
}  // namespace render_benchmark
}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
