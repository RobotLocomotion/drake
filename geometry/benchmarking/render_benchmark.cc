#include <unistd.h>

#include "fmt/format.h"
#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

#include "drake/common/filesystem.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace render {

/* See render_benchmark_doxygen.h for discussion of this benchmark.  */

/* Friend class for accessing RenderEngine's protected/private functionality. */
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
const double kZSpherePosition = -4.;

// Default camera properties.
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

/* The render engines generally supported by this benchmark; not all
 renderers are supported by all operating systems.  */
enum class EngineType { Vtk, Gl };

/* Creates a render engine of the given type with the given background color.
 For each supported render engine type, this must be specialized. (See below.
 */
template <EngineType engine_type>
std::unique_ptr<RenderEngine> MakeEngine(const Vector3d& bg_rgb) {
  throw std::runtime_error("Not implemented!");
}

class RenderBenchmark : public benchmark::Fixture {
 public:
  RenderBenchmark() {
    material_.AddProperty("phong", "diffuse", sphere_rgba_);
    material_.AddProperty("label", "id", RenderLabel::kDontCare);
  }

  using benchmark::Fixture::SetUp;
  void SetUp(const ::benchmark::State&) { depth_cameras_.clear(); }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void ColorImage(::benchmark::State& state, const std::string& name) {
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    auto [sphere_count, camera_count, width, height] = ReadState(state);
    SetupScene(sphere_count, camera_count, width, height, renderer.get());
    ImageRgba8U color_image(width, height);

    /* To account for RenderEngine implementations that do extraordinary work
     in their first invocations, we perform a couple of render passes in order
     to warm start the engine and actually measure its steady state performance.
     */
    for (int i = 0; i < 2; ++i) {
      const ColorRenderCamera color_cam(depth_cameras_[0].core(),
                                        FLAGS_show_window);
      renderer->RenderColorImage(color_cam, &color_image);
    }

    /* Now the timed loop. */
    for (auto _ : state) {
      renderer->UpdatePoses(poses_);
      for (int i = 0; i < camera_count; ++i) {
        const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                          FLAGS_show_window);
        renderer->RenderColorImage(color_cam, &color_image);
      }
    }
    if (!FLAGS_save_image_path.empty()) {
      const std::string path_name = image_path_name(name, state, "png");
      SaveToPng(color_image, path_name);
      saved_image_paths.insert(path_name);
    }
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void DepthImage(::benchmark::State& state, const std::string& name) {
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    auto [sphere_count, camera_count, width, height] = ReadState(state);
    SetupScene(sphere_count, camera_count, width, height, renderer.get());
    ImageDepth32F depth_image(width, height);

    /* To account for RenderEngine implementations that do extraordinary work
     in their first invocations, we perform a couple of render passes in order
     to warm start the engine and actually measure its steady state performance.
     */
    for (int i = 0; i < 2; ++i) {
      renderer->RenderDepthImage(depth_cameras_[0], &depth_image);
    }

    /* Now the timed loop. */
    for (auto _ : state) {
      renderer->UpdatePoses(poses_);
      for (int i = 0; i < camera_count; ++i) {
        renderer->RenderDepthImage(depth_cameras_[i], &depth_image);
      }
    }
    if (!FLAGS_save_image_path.empty()) {
      const std::string path_name = image_path_name(name, state, "tiff");
      SaveToTiff(depth_image, path_name);
      saved_image_paths.insert(path_name);
    }
  }

  template <EngineType engine_type>
  // NOLINTNEXTLINE(runtime/references)
  void LabelImage(::benchmark::State& state, const std::string& name) {
    auto renderer = MakeEngine<engine_type>(bg_rgb_);
    auto [sphere_count, camera_count, width, height] = ReadState(state);
    SetupScene(sphere_count, camera_count, width, height, renderer.get());
    ImageLabel16I label_image(width, height);

    /* To account for RenderEngine implementations that do extraordinary work
     in their first invocations, we perform a couple of render passes in order
     to warm start the engine and actually measure its steady state performance.
     */
    for (int i = 0; i < 2; ++i) {
      const ColorRenderCamera color_cam(depth_cameras_[0].core(),
                                        FLAGS_show_window);
      renderer->RenderLabelImage(color_cam, &label_image);
    }

    /* Now the timed loop. */
    for (auto _ : state) {
      renderer->UpdatePoses(poses_);
      for (int i = 0; i < camera_count; ++i) {
        const ColorRenderCamera color_cam(depth_cameras_[i].core(),
                                          FLAGS_show_window);
        renderer->RenderLabelImage(color_cam, &label_image);
      }
    }
    if (!FLAGS_save_image_path.empty()) {
      const std::string path_name = image_path_name(name, state, "png");
      SaveToPng(label_image, path_name);
      saved_image_paths.insert(path_name);
    }
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
    const auto [sphere_count, camera_count, width, height] = ReadState(state);
    return save_path.append(fmt::format("{}_{}_{}_{}_{}.{}", test_name,
                                        sphere_count, camera_count, width,
                                        height, format));
  }

  /* Computes a compact array of spheres which will remain in view. */
  void AddSphereArray(int sphere_count, const RenderCameraCore& core,
                      RenderEngine* engine) {
    /* We assume the camera is located at (0, 0, c.z) pointing in the -Wz
     direction. We further assume that the camera's "up" direction points in the
     +Wy direction. All spheres will be placed on a plane at z = s.z. Given the
     camera field of view and w/h aspect ratio, we can determine the visible
     rectangle at s.z.

         c.z      s.z         Simple geometry.
          ┆        ┆          Right triangle with height d = s.z - c.z and angle
          ┆        ╱          θ = fov_y / 2.
          ┆      ╱ ┆          hₛ/2 = d * tan(θ)
          ┆    ╱   ┆ hₛ/2       hₛ = 2d * tan(θ)
          ┆  ╱     ┆          wₛ = hₛ * w / h
          ┆╱_θ_____┆______    (w, h) is the size of the image sensor giving us
           ╲ θ     ┆          the camera's aspect ratio.
             ╲     ┆
               ╲   ┆
                 ╲ ┆
                   ╲
                   ┆
      This gives us the measure of the rectangle we need to fit all spheres
      into. We want all spheres to be visible so that it affects the rendering
      cost. */
    const double aspect_ratio = core.intrinsics().width() /
                                static_cast<double>(core.intrinsics().height());
    const double theta_2 = core.intrinsics().fov_y() / 2.0;
    const double d = -kZSpherePosition;  // c.z = 0.
    const double h = 2 * d * std::tan(theta_2);
    /* Given the measure of the rectangle, we need to place the spheres: this
     includes determining radius and position. We'll place the N spheres in a
     rectangular grid with R rows and C columns. It must be the case that
     RC ≥ N. With the constraint that we want C/R "as close" to w/h as possible.
     We define "as close as possible" as the maximum C/R ≤ w/h. For notation
     convenience we'll define α = w/h.

       RC ≥ N  --> C/R ≥ N/R²  (C, R, and N are all positive).
       C/R ≥ N/R² and C/R ≤ α --> N/R² ≤ C/R ≤ α
       N/R² ≤ α
       N/α ≤ R²
       √(N/α) ≤ R

     We'll use the formula above to find our initial guess for the number of
     rows. We'll increment row to the last value that satisfies C/R ≤ w/h. */
    const double N = static_cast<double>(sphere_count);
    int rows = static_cast<int>(std::max(std::sqrt(N / aspect_ratio), 1.0));
    int cols = static_cast<int>(std::ceil(N / rows));
    while (static_cast<double>(cols) / rows > aspect_ratio) {
      ++rows;
      cols = static_cast<int>(std::ceil(N / rows));
    }

    /* Because we've required C/R ≤ w/h, we know we'll always be fitting the
     number of rows to the height of the image. The radius value we want
     satisfies R * 2*radius = h. radius = h / 2R. */
    const double distance = h / (2 * rows);
    /* We make the actual radius *slightly* smaller so there's some space
     between the spheres. */
    Sphere sphere{distance * 0.95};
    auto add_sphere = [this, engine, &sphere](const Vector3d& p_WS) {
      GeometryId geometry_id = GeometryId::get_new_id();
      engine->RegisterVisual(geometry_id, sphere, material_,
                             RigidTransformd::Identity(),
                             true /* needs update */);
      poses_.insert({geometry_id, RigidTransformd{p_WS}});
    };

    int count = 0;
    double y = -(rows - 1) * distance;
    for (int r = 0; r < rows; ++r) {
      double x = -(cols - 1) * distance;
      for (int c = 0; c < cols; ++c) {
        add_sphere(Vector3d{x, y, kZSpherePosition});
        ++count;
        if (count >= sphere_count) break;
        x += 2 * distance;
      }
      if (count >= sphere_count) break;
      y += 2 * distance;
    }
  }

  void SetupScene(const int sphere_count, const int camera_count,
                  const int width, const int height, RenderEngine* engine) {
    // Set up the camera so that Cz = -Wz, Cx = Wx, and Cy = -Wy. The camera
    // will look down the Wz axis and have the image U direction aligned with
    // the Wx direction.
    const Vector3d Cx_W{1, 0, 0};
    const Vector3d Cy_W{0, -1, 0};
    const Vector3d Cz_W{0, 0, -1};
    RigidTransformd X_WC{
        RotationMatrixd::MakeFromOrthonormalColumns(Cx_W, Cy_W, Cz_W)};
    engine->UpdateViewpoint(X_WC);

    // Add the cameras.
    for (int i = 0; i < camera_count; ++i) {
      depth_cameras_.emplace_back(RenderCameraCore{"unused" + std::to_string(i),
                                                   {width, height, kFovY},
                                                   {0.01, 100.0},
                                                   {}},
                                  DepthRange{kZNear, kZFar});
    }
    AddSphereArray(sphere_count, depth_cameras_[0].core(), engine);

    // Offset the light from its default position shared with the camera, i.e.
    // (0, 0, 1), so that shadows can be seen in the render.
    // TODO(SeanCurtis-TRI) This is using a stop-gap mechanism for configuring
    // light position. Swap it to use a light declaration API when it is
    // introduced.
    RenderEngineTester::SetDefaultLightPosition(Vector3d{0.5, 0.5, 1},
                                                engine);
  }

  // Keep track of the paths to the saved images. We use a static set because
  // Google Benchmark runs these benchmarks multiple times with unique instances
  // of the fixture, and we want to avoid duplicate path names.
  static std::set<std::string> saved_image_paths;

  std::vector<DepthRenderCamera> depth_cameras_;
  PerceptionProperties material_;
  const Vector3d bg_rgb_{200 / 255., 0, 250 / 255.};
  const Rgba sphere_rgba_{0, 0.8, 0.5, 1};
  std::unordered_map<GeometryId, RigidTransformd> poses_;
};
std::set<std::string> RenderBenchmark::saved_image_paths;

/* These macros serve the purpose of allowing compact and *consistent*
 declarations of benchmarks. The goal is to create a benchmark for each
 renderer type (e.g., Vtk, Gl) combined with each image type (Color, Depth, and
 Label). Each benchmark instance should be executed using the same parameters.

 These macros guarantee that a benchmark is declared, dispatches the right
 benchmark harness and is executed with a common set of parameters.

 The macro is invoked as follows:

   MAKE_BENCHMARK(Foo, ImageType)

 such that there must be a `RenderEngineFoo` type (e.g., RenderEngineVtk) and
 ImageType must be one of (Color, Depth, or Label). Capitalization matters.
 There must also be a specialization of MakeEngine<EngineType::Foo> (see below).

 N.B. The macro STR converts a single macro parameter into a string and we use
 it to make a string out of the concatenation of two macro parameters (i.e., we
 get FooColor out of the parameters Foo and Color).

 The parameters are 4-tuples of: sphere count, camera count, image width, and
 image height. */
#define STR(s) #s
#define MAKE_BENCHMARK(Renderer, ImageT) \
BENCHMARK_DEFINE_F(RenderBenchmark, Renderer##ImageT) \
    (benchmark::State&state) { \
  ImageT##Image<EngineType::Renderer>(state, STR(Renderer##ImageT)); \
} \
BENCHMARK_REGISTER_F(RenderBenchmark, Renderer##ImageT) \
    ->Unit(benchmark::kMillisecond) \
    ->Args({1, 1, 640, 480}) \
    ->Args({12, 1, 640, 480}) \
    ->Args({120, 1, 640, 480}) \
    ->Args({240, 1, 640, 480}) \
    ->Args({480, 1, 640, 480}) \
    ->Args({1200, 1, 640, 480}) \
    ->Args({1, 10, 640, 480}) \
    ->Args({1200, 10, 640, 480}) \
    ->Args({1, 1, 320, 240}) \
    ->Args({1, 1, 1280, 960}) \
    ->Args({1, 1, 2560, 1920}) \
    ->Args({1200, 1, 320, 240}) \
    ->Args({1200, 1, 1280, 960}) \
    ->Args({1200, 1, 2560, 1920})


template <>
std::unique_ptr<RenderEngine> MakeEngine<EngineType::Vtk>(
    const Vector3d& bg_rgb) {
  RenderEngineVtkParams params{{}, {}, bg_rgb};
  return MakeRenderEngineVtk(params);
}

MAKE_BENCHMARK(Vtk, Color);
MAKE_BENCHMARK(Vtk, Depth);
MAKE_BENCHMARK(Vtk, Label);

#ifdef RENDER_ENGINE_GL_SUPPORTED
template <>
std::unique_ptr<RenderEngine> MakeEngine<EngineType::Gl>(
    const Vector3d& bg_rgb) {
  RenderEngineGlParams params;
  params.default_clear_color.set(bg_rgb[0], bg_rgb[1], bg_rgb[2], 1.0);
  return MakeRenderEngineGl(params);
}

MAKE_BENCHMARK(Gl, Color);
MAKE_BENCHMARK(Gl, Depth);
MAKE_BENCHMARK(Gl, Label);
#endif

void Cleanup() {
  if (!RenderBenchmark::saved_image_paths.empty()) {
    std::cout << "Saved rendered images to:" << std::endl;
    for (const auto& path : RenderBenchmark::saved_image_paths) {
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
