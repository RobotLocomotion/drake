#include <filesystem>
#include <future>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/geometry/render_gl/internal_texture_library.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

class TextureLibraryTester {
 public:
  static int NumTextures(const TextureLibrary& library) {
    return ssize(library.textures_);
  }
};

}  // namespace internal
}  // namespace render_gl

namespace {

namespace fs = std::filesystem;

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ClippingRange;
using render::ColorRenderCamera;
using render::RenderCameraCore;
using render::RenderEngine;
using render_gl::internal::OpenGlContext;
using render_gl::internal::TextureLibrary;
using render_gl::internal::TextureLibraryTester;
using systems::sensors::CameraInfo;
using systems::sensors::ImageRgba8U;
using systems::sensors::SaveToPng;

/* We should be able to safely call GetTextureId in multiple threads. */
GTEST_TEST(ThreadSafetyTest, TextureLibrary) {
  TextureLibrary library;
  ASSERT_EQ(TextureLibraryTester::NumTextures(library), 0);

  const std::string image_name =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");

  std::atomic<int> num_errors{0};
  auto work = [&library, &image_name, &num_errors] (int i) {
    try {
      std::optional<GLuint> id = library.GetTextureId(image_name);
      if (!id.has_value()) {
        throw std::runtime_error("no valid texture id returned.");
      }
    } catch (std::exception& e) {
      log()->error("Worker {} unexpected error: {}", i, e.what());
      ++num_errors;
    }
  };

  std::vector<std::future<void>> futures;
  futures.push_back(std::async(std::launch::async, work, 0));
  futures.push_back(std::async(std::launch::async, work, 1));
  futures.push_back(std::async(std::launch::async, work, 2));

  for (auto& future : futures) {
    future.get();
  }

  /* All invocations successfully returned an id. */
  ASSERT_EQ(num_errors, 0);
  /* There is still only a single texture in the library. */
  EXPECT_EQ(TextureLibraryTester::NumTextures(library), 1);
}

/* Tests two properties regarding OpenGlContexts and threads:
 - A context bound in one thread cannot be bound in another (see `source`
   below).
 - Cloned contexts created in one thread can be bound in other threads, even
   even if the original context is bound in a different thread. */
GTEST_TEST(ThreadSafetyTest, OpenGlContext) {
  std::vector<OpenGlContext> contexts;
  /* The first is the "source" and we'll add two that are clones of it. */
  contexts.emplace_back(false /* debug */);
  contexts.emplace_back(contexts.front());
  contexts.emplace_back(contexts.front());
  const OpenGlContext& source = contexts.front();

  /* We can bind the source in the main thread, no problem. */
  ASSERT_NO_THROW(source.MakeCurrent());
  ASSERT_TRUE(source.IsCurrent());

  std::atomic<int> num_errors{0};
  auto work = [&num_errors, &contexts](int i) {
    try {
      const OpenGlContext& context = contexts.at(i);
      context.MakeCurrent();
      if (i == 0) {
        /* Successfully binding contexts[0] (aka source) *should've* been an
         error. Make it into one we can recognize. */
        throw std::runtime_error("Worker 0 should not be able to bind the "
                                 "source context in a worker thread.");
      }
      /* We assume that if we get here context.IsCurrent() reports true. */
    } catch (std::exception& e) {
      /* We expect a single exception as evidence of correctness: 0 can't make
       context current. Everything else is an error. */
      using std::string_view;
      if (!(i == 0 &&
            e.what() ==
                string_view("Error making an OpenGL context current"))) {
        log()->error("Worker {} exception: {}", i, e.what());
        ++num_errors;
      }
    }
  };

  std::vector<std::future<void>> futures;
  /* Worker 0 will attempt to erroneously rebind `source`. */
  futures.push_back(std::async(std::launch::async, work, 0));
  futures.push_back(std::async(std::launch::async, work, 1));
  futures.push_back(std::async(std::launch::async, work, 2));

  for (auto& future : futures) {
    future.get();
  }

  ASSERT_EQ(num_errors, 0);
}

/* Helper for variant resolution. */
template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

/* Adds the given `shape` as an *anchored* geometry to the given `engine` at the
 given position `p_WS` with the given `diffuse` definition. */
void AddShape(const Shape& shape, const Vector3d& p_WS,
              std::variant<Rgba, std::string> diffuse, RenderEngine* engine) {
  PerceptionProperties material;
  material.AddProperty("label", "id", render::RenderLabel::kDontCare);
  std::visit(overloaded{[&material](const Rgba& rgba) {
                          material.AddProperty("phong", "diffuse", rgba);
                        },
                        [&material](const std::string& diffuse_map) {
                          material.AddProperty("phong", "diffuse_map",
                                               diffuse_map);
                        }},
             diffuse);
  engine->RegisterVisual(GeometryId::get_new_id(), shape, material,
                         math::RigidTransformd(p_WS), false /* needs update */);
}

/* We use Vector4<ImageRgba8U::T> to compare the pixel values in an image with
 an expected Rgba value - should be vector of four ints in the range 0-255. */
Vector4<ImageRgba8U::T> RgbaVector(const Rgba& rgba) {
  auto to_T = [](double c) {
    return static_cast<ImageRgba8U::T>(c * 255 + 0.5);
  };
  return Vector4<ImageRgba8U::T>(to_T(rgba.r()), to_T(rgba.g()), to_T(rgba.b()),
                                 to_T(rgba.a()));
}

Vector4<ImageRgba8U::T> RgbaVector(const ImageRgba8U::T* data) {
  return Vector4<ImageRgba8U::T>(data[0], data[1], data[2], data[3]);
}

/* RenderEngineGl has to handle its OpenGlContext correctly to benefit from
 the context's thread-independence and OpenGL object sharing. It does some extra
 clean up work.

 This test confirms that clones of RenderEngineGl does enough clean up. We
 assume that creating RenderEngineGl instances from scratch must create
 OpenGlContexts from scratch as well, so we don't explicitly test it.

 The independence is tested similarly to the OpenGlContext above; one instance
 is bound in the main thread. Its clones should be functional in their own
 threads.

 The scene is composed of multiple shape types (to make sure that what is
 claimed to be shared *is* shared and what needs to be patched after cloning
 is patched). One of the shapes will be textured to show that textures survive
 cloning as well.

      ┌───────────────────────┐
      │                       │
      │          Cyl          │  - Red half space filling the background.
      │                       │  - Blue cylinder at 12 o'clock.
      │     Sph       Box     │  - Green (via texture) box at 3 o'clock.
      │                       │  - Yellow capsule at 6 o'clock.
      │          Cap          │  - Orange sphere at 9 o'clock.
      │                       │
      └───────────────────────┘

 N.B. If you set `show_window` to true in the camera settings, the camera will
 be flipped vertically (the cylinder and capsule will change positions). This
 is a documented property of how `show_window` works with RenderEngineGl (see
 render_gl/factory.h).

 To test the correctness of the rendered engine, we'll sample the color on
 each shape. The rendered images are saved to the test outputs.

 What this doesn't test:

   - Depth and label images. We assume that if the two color shaders get
     handled properly during the clone, then the depth and label shader programs
     do as well.
   - Dynamic geometry. All of the geometries used are registered as *anchored*
     for convenience (we can pose them when adding them). The logic for posing
     the dynamic geometries doesn't depend on the OpenGlContext, so other tests
     on correct posing of dynamic geometries are sufficient. */
GTEST_TEST(ThreadSafetyTest, RenderEngineGl) {
  std::unique_ptr<RenderEngine> source = MakeRenderEngineGl();

  const Rgba red(1, 0, 0);
  const Rgba blue(0, 0, 1);
  const Rgba orange(1, 0.5, 0);
  // The green of the texture box.png.
  const Rgba green(4 / 255.0, 241 / 255.0, 33 / 255.0);
  const Rgba yellow(1, 1, 0);

  /* Background. */
  AddShape(HalfSpace(), Vector3d(0, 0, -0.25), red, source.get());
  /* 12 o'clock. */
  AddShape(Cylinder(0.25, 0.25), Vector3d(0, 0.5, 0), blue, source.get());
  /* 3 o'clock. */
  AddShape(Box(0.5, 0.5, 0.5), Vector3d(0.5, 0, 0),
           FindResourceOrThrow("drake/geometry/render/test/meshes/box.png"),
           source.get());
  /* 6 o'clock. */
  AddShape(Capsule(0.25, 0.25), Vector3d(0, -0.5, 0), yellow, source.get());
  /* 9 o'clock. */
  AddShape(Sphere(0.25), Vector3d(-0.5, 0, 0), orange, source.get());

  /* Pose of camera body in the world: Looking straight down from 3m above the
   ground. Wy and Wx point to the top and right of the image, respectively. */
  const RigidTransformd X_WC(
      RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                      AngleAxisd(M_PI, Vector3d::UnitZ())},
      {0, 0, 3});
  source->UpdateViewpoint(X_WC);

  /* Create the clones. */
  constexpr int kCloneCount = 3;
  std::vector<std::unique_ptr<RenderEngine>> renderers;
  for (int i = 0; i < kCloneCount; ++i) {
    renderers.push_back(source->Clone());
    renderers.back()->UpdateViewpoint(X_WC);
  }

  /* Create some renderings. */
  const ColorRenderCamera camera(RenderCameraCore(
      "color", CameraInfo(640, 480, M_PI_4), ClippingRange(0.01, 100.0), {}));

  /* Sample the image at known locations to see if we have the expected colors.
   */
  auto check_image = [w = camera.core().intrinsics().width(),
                      h = camera.core().intrinsics().height(), red, blue,
                      orange, green, yellow](const ImageRgba8U& image) {
    /* For the cylinder and box, the camera-facing face is big and flat and we
     can sample the image in a large area and get the expected material color.
     For the sphere and the capsule (orange and yellow, respectively), we
     need to be more precise. */

    // clang-format off
    EXPECT_TRUE(CompareMatrices(RgbaVector(image.at(w / 2, h / 2)),
                                RgbaVector(red), 1)) << "Half space";
    EXPECT_TRUE(CompareMatrices(RgbaVector(image.at(w / 2, h / 4)),
                                RgbaVector(blue), 1)) << "Cylinder";
    EXPECT_TRUE(CompareMatrices(RgbaVector(image.at(w * 0.625, h / 2)),
                                RgbaVector(green), 1)) << "Box";
    EXPECT_TRUE(CompareMatrices(RgbaVector(image.at(w / 2, h * 0.73)),
                                RgbaVector(yellow), 1)) << "Capsule";
    EXPECT_TRUE(CompareMatrices(RgbaVector(image.at(w * 0.334, h / 2)),
                                RgbaVector(orange), 1)) << "Sphere";
    // clang-format on
  };

  /* Render from the main thread with the source engine; this should bind its
   OpenGl context to the main thread. If any of the clones shared contexts
   with it, they should be unable to bind their contexts in *other* threads. */
  ImageRgba8U source_image(camera.core().intrinsics().width(),
                           camera.core().intrinsics().height());
  ASSERT_NO_THROW(source->RenderColorImage(camera, &source_image));
  check_image(source_image);
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    SaveToPng(source_image, fs::path(dir) / "source.png");
  }

  /* The multi-threaded work function; render and check the image. */
  std::atomic<int> num_errors{0};
  auto work = [&num_errors, &renderers, &check_image, &camera](int i) {
    try {
      RenderEngine& renderer = *renderers.at(i);
      ImageRgba8U image(camera.core().intrinsics().width(),
                        camera.core().intrinsics().height());
      bool rendered = true;
      try {
        renderer.RenderColorImage(camera, &image);
      } catch (std::exception& e) {
        log()->error("Worker {} exception: {}", i, e.what());
        rendered = false;
      }
      if (rendered) {
        /* The image actually rendered, test the contents. */
        SCOPED_TRACE(fmt::format("Worker {}", i));
        check_image(image);
        if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
          SaveToPng(image, fs::path(dir) / fmt::format("worker{}.png", i));
        }
      } else {
        ++num_errors;
      }
    } catch (std::exception& e) {
      ++num_errors;
      log()->error("Unexpected non-rendering error for worker {}: {}", i,
                   e.what());
    }
  };

  std::vector<std::future<void>> futures;
  for (int i = 0; i < kCloneCount; ++i) {
    futures.push_back(std::async(std::launch::async, work, i));
  }

  for (auto& future : futures) {
    future.get();
  }

  ASSERT_EQ(num_errors, 0);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
