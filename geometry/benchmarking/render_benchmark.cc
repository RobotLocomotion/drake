#include <unistd.h>

#include "fmt/format.h"
#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

#include "drake/common/filesystem.h"
#include "drake/geometry/render/render_engine_ospray_factory.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace render {

/** @defgroup render_engine_benchmarks Render Engine Benchmarks
 @ingroup render_benchmarks

 The benchmark consists of a scene with a ground box, one or more spheres
 floating above the plane, and one or more cameras above the spheres, looking
 down at the spheres.

 If there are multiple spheres, they are positioned in a regular grid positioned
 at a uniform height above the ground plane. Increasing the number of spheres
 provides an approximate measure of how the renderer performs with increased
 scene complexity.

 If there are multiple cameras, they are all at the same position, looking in
 the same direction, with the same intrinsic properties. In other words, each
 should produce the same output image. This provides a measure of the
 scalability as a simulation includes an increasing number of cameras.

 The output image can be configured to an arbitrary size. For both RenderEngine
 implementations, larger images take more time.

 <h2>Running the benchmark</h2>

 The benchmark can be executed as:

 ```
 bazel run //geometry/benchmarking:render_benchmark
 ```

 The output will be something akin to:

 ```
Run on (12 X 4400 MHz CPU s)
CPU Caches:
  L1 Data 32K (x6)
  L1 Instruction 32K (x6)
  L2 Unified 256K (x6)
  L3 Unified 12288K (x1)
Load Average: 9.75, 4.65, 3.26
***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
------------------------------------------------------------------------------------------------------
Benchmark                                                            Time             CPU   Iterations
------------------------------------------------------------------------------------------------------
RenderEngineBenchmark/VtkColor/1/1/640/480                        1.11 ms         1.08 ms          514
RenderEngineBenchmark/VtkColor/4/1/640/480                        1.05 ms         1.05 ms          684
RenderEngineBenchmark/VtkColor/8/1/640/480                        1.10 ms         1.09 ms          610
RenderEngineBenchmark/VtkColor/1/10/640/480                       10.5 ms         10.1 ms           67
RenderEngineBenchmark/VtkColor/1/1/320/240                       0.391 ms        0.390 ms         1567
RenderEngineBenchmark/VtkColor/1/1/1280/960                       3.13 ms         3.13 ms          223
RenderEngineBenchmark/VtkColor/1/1/2560/1920                      12.2 ms         12.2 ms           49
RenderEngineBenchmark/VtkDepth/1/1/640/480                        1.35 ms         1.35 ms          484
RenderEngineBenchmark/VtkDepth/1/10/640/480                       13.0 ms         13.0 ms           50
RenderEngineBenchmark/VtkLabel/1/1/640/480                        1.45 ms         1.45 ms          464
RenderEngineBenchmark/VtkLabel/1/10/640/480                       26.1 ms         25.3 ms           37
RenderEngineBenchmark/OsprayRayColor/1/1/640/480                  23.4 ms         22.7 ms           33
RenderEngineBenchmark/OsprayRayColor/4/1/640/480                  28.8 ms         27.4 ms           23
RenderEngineBenchmark/OsprayRayColor/8/1/640/480                  34.4 ms         32.9 ms           17
RenderEngineBenchmark/OsprayRayColor/1/10/640/480                  193 ms          174 ms            3
RenderEngineBenchmark/OsprayRayColor/1/1/320/240                  5.16 ms         5.09 ms          120
RenderEngineBenchmark/OsprayRayColor/1/1/1280/960                 67.9 ms         65.2 ms           11
RenderEngineBenchmark/OsprayRayColor/1/1/2560/1920                 283 ms          267 ms            2
RenderEngineBenchmark/OsprayRayColorShadowsOff/1/1/640/480        15.7 ms         15.7 ms           43
RenderEngineBenchmark/OsprayRayColorShadowsOff/4/1/640/480        20.0 ms         19.6 ms           30
RenderEngineBenchmark/OsprayRayColorShadowsOff/8/1/640/480        25.2 ms         25.2 ms           28
RenderEngineBenchmark/OsprayRayColorShadowsOff/1/10/640/480        162 ms          159 ms            4
RenderEngineBenchmark/OsprayPathColor/1/1/640/480                 36.2 ms         35.3 ms           20
RenderEngineBenchmark/OsprayPathColor/1/10/640/480                 351 ms          345 ms            2
 ```

 Additional configuration is possible via the following flags:
 - __save_image_path__: Enables saving the rendered images in the given
   location. Defaults to no saving.
 - __show_window__: Whether to display the rendered images. Defaults to false.
 - __samples_per_pixel__: The number of illumination samples per pixel when path
   tracing with RenderEngineOspray. Higher numbers introduce higher quality at
   increased cost. Defaults to 1.

 For example:
 ```
 bazel run //geometry/benchmarking:render_benchmark -- --save_image_path="/tmp" --show_window=true --samples_per_pixel=100
 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 RenderEngineBenchmark/TestName/camera_count/sphere_count/image_width/image_height
 ```

   - __TestName__: One of
     - __VtkColor__: Renders the color image from RenderEngineVtk.
     - __VtkDepth__: Renders the depth image from RenderEngineVtk.
     - __VtkLabel__: Renders the label image from RenderEngineVtk.
     - __OsprayRayColor__: Renders the color image from RenderEngineOspray with
       ray-traced shadows.
     - __OsprayRayColorShadowsOff__: Renders the color image from
       RenderEngineOspray without any shadows -- most closely approximates the
       RenderEngineVtk color image.
     - __OsprayPathColor__: Renders the color image from RenderEngineOspray with
       path-traced global illumination (with only a single sample per pixel by
       default, unless configured using --samples_per_pixel).
   - __camera_count__: Simply the number of independent cameras being rendered.
     The cameras are all co-located (same position, same view direction) so
     they each render the same image.
   - __sphere_count__: The total number of spheres.
   - __image width__ and __image_height__: The dimensions of the output image
     in pixels.

 The `Time` and `CPU` columns are measures of the average time it took to create
 a single frame for all the specified cameras. The `Iterations` indicates how
 often the action was performed to compute the average value. For more
 information see the [google benchmark
 documentation](https://github.com/google/benchmark).

 Now we can analyze the example output and draw some example inferences (not a
 complete set of valid inferences):

   - Frame cost scales linearly with the number of cameras.
     - For RenderEngineVtk a 10X increase in number of cameras leads to a 10X
       increase in time for both color and depth, but a (roughly) 20X increase
       for label.
     - RenderEngineVtk also increased a factor of 10X when path-tracing the
       scene when we increased the number of cameras by a factor of 10X.
   - The number of objects in the scene has an apparently negligible impact on
     RenderEngineVtk, but a noticeable impact on RenderEngineOspray.
 */

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

  /** Parse arguments from the benchmark state.
   @return A tuple representing the sphere count, camera count, width, and
           height.  */
  static std::tuple<int, int, int, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), state.range(1), state.range(2),
                           state.range(3));
  }

  /** Helper function for generating the image path name based on the benchmark
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
      cameras_.emplace_back(width, height, kFovY, "unused" + std::to_string(i),
                            kZNear, kZFar);
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
  std::vector<DepthCameraProperties> cameras_;
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
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
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
      renderer_->RenderDepthImage(cameras_[i], &depth_image_);
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
      renderer_->RenderLabelImage(cameras_[i], FLAGS_show_window,
                                  &label_image_);
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

BENCHMARK_DEFINE_F(RenderEngineBenchmark, OsprayRayColor)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  auto [sphere_count, camera_count, width, height] = ReadState(state);
  SetupOsprayRender(sphere_count, camera_count, width, height,
                    OsprayMode::kRayTracer, true);
  for (auto _ : state) {
    for (int i = 0; i < camera_count; ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    const std::string path_name =
        image_path_name("OsprayRayColor", state, "png");
    SaveToPng(color_image_, path_name);
    saved_image_paths.insert(path_name);
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
  auto [sphere_count, camera_count, width, height] = ReadState(state);
  SetupOsprayRender(sphere_count, camera_count, width, height,
                    OsprayMode::kRayTracer, false);
  for (auto _ : state) {
    for (int i = 0; i < camera_count; ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    const std::string path_name =
        image_path_name("OsprayRayColorShadowsOff", state, "png");
    SaveToPng(color_image_, path_name);
    saved_image_paths.insert(path_name);
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
  auto [sphere_count, camera_count, width, height] = ReadState(state);
  SetupOsprayRender(sphere_count, camera_count, width, height,
                    OsprayMode::kPathTracer, true);
  for (auto _ : state) {
    // NOTE: The ospray renderer has a quirk; if no poses update, subsequent
    // render passes seem to accumulate into the same frame buffer, artificially
    // improving the image quality, like increasing the number of per-pixel
    // samples.
    renderer_->UpdatePoses(poses_);
    for (int i = 0; i < camera_count; ++i) {
      renderer_->RenderColorImage(cameras_[i], FLAGS_show_window,
                                  &color_image_);
    }
  }
  if (!FLAGS_save_image_path.empty()) {
    const std::string path_name =
        image_path_name("OsprayPathColor", state, "png");
    SaveToPng(color_image_, path_name);
    saved_image_paths.insert(path_name);
  }
}
BENCHMARK_REGISTER_F(RenderEngineBenchmark, OsprayPathColor)
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
