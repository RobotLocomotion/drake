#include <future>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/internal_opengl_context.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

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
using render_gl::internal::OpenGlContext;
using systems::sensors::CameraInfo;
using systems::sensors::ImageRgba8U;

// TODO(SeanCurtis-TRI): Test thread safety of TextureLibrary.

// Tests two properties regarding OpenGlContexts and threads:
// - A context bound in one thread cannot be bound in another (see `source`
//   below).
// - Cloned contexts created in one thread can be bound in other threads, even
//   even if the original context is bound in different thread.
GTEST_TEST(OpenGlContextTest, ThreadSafety) {
  std::vector<OpenGlContext> contexts;
  // The first is the "source" and we'll add two that are clones of it.
  contexts.emplace_back(false /* debug */);
  contexts.emplace_back(contexts.front());
  contexts.emplace_back(contexts.front());
  const OpenGlContext& source = contexts.front();

  // We can bind this in the main thread, no problem.
  ASSERT_NO_THROW(source.MakeCurrent());
  ASSERT_TRUE(source.IsCurrent());

  std::atomic<int> num_errors{0};
  auto work = [&num_errors, &contexts](int i) {
    try {
      const OpenGlContext& context = contexts.at(i);
      context.MakeCurrent();
      if (i == 0) {
        // Successfully binding contexts[0] (aka source) *should've" been an
        // error.
        throw std::runtime_error("Worker 0 should not be able to bind the "
                                 "source context in a worker thread.");
      }
      if (!context.IsCurrent()) {
        // MakeCurrent() didn't really work; also an error.
        throw std::runtime_error("OpenGlContext is not the current thread.");
      }
    } catch (std::exception& e) {
      // The single exception we expect (0 can't make context current) is
      // ignored. Everything else is an error.
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
  // Worker 0 will attempt to erroneously rebind `source`.
  futures.push_back(std::async(std::launch::async, work, 0));
  futures.push_back(std::async(std::launch::async, work, 1));
  futures.push_back(std::async(std::launch::async, work, 2));

  for (auto& future : futures) {
    future.get();
  }

  ASSERT_EQ(num_errors, 0);
}

// RenderEngineGl has to handle its OpenGlContext correctly to benefit from
// its thread-independence and OpenGL object sharing.
//
// This test confirms that clones of RenderEngineGl properly clone contexts. We
// assume that creating RenderEngineGl instances from scratch must create
// OpenGlContexts from scratch as well, so we don't explicitly test it.
//
// The independence is tested similarly to the OpenGlContext above; one instance
// is bound in the main thread. Its clones should be independently functional
// in its own threads.
//
// Furthermore, the images each produce should include the same textured box --
// proof that OpenGl objects (texture and mesh data) are shared across all
// engines.
GTEST_TEST(RenderEngineGlTest, ThreadSafety) {
  std::unique_ptr<RenderEngine> source_engine = MakeRenderEngineGl();

  // Add a single textured, object to the source engine; it should be renderable
  // by each cloned engine.
  PerceptionProperties material;
  material.AddProperty("label", "id", render::RenderLabel::kDontCare);
  material.AddProperty(
      "phong", "diffuse_map",
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png"));
  source_engine->RegisterVisual(GeometryId::get_new_id(), Box(1, 1, 1),
                                material, math::RigidTransformd(),
                                false /* needs update */);

  // Pose of camera body in the world: Looking straight down from 3m above the
  // ground.
  const RigidTransformd X_WC(
      RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                      AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
      {0, 0, 3.0});
  source_engine->UpdateViewpoint(X_WC);

  // Create the clones.
  std::vector<std::unique_ptr<RenderEngine>> renderers;
  for (int i = 0; i < 2; ++i) {
    renderers.push_back(source_engine->Clone());
    renderers.back()->UpdateViewpoint(X_WC);
  }

  // Create some renderings.
  const ColorRenderCamera camera(RenderCameraCore(
      "color", CameraInfo(640, 480, 2.0), ClippingRange(0.25, 10.0), {}));

  // Render from the main thread with the source engine; this should bind its
  // OpenGl context to the main thread. If any of the clones shared contexts
  // with it, they should be unable to bind their contexts in *other* threads.
  ImageRgba8U source_image(camera.core().intrinsics().width(),
                           camera.core().intrinsics().height());
  ASSERT_NO_THROW(source_engine->RenderColorImage(camera, &source_image));

  // Simply look to see if we have an image with the material color in the
  // center. If present, the box should guarantee it.
  auto check_image =
      [w = camera.core().intrinsics().width(),
       h = camera.core().intrinsics().height()](const ImageRgba8U& image) {
        const int x = w / 2;
        const int y = h / 2;
        EXPECT_NEAR(image.at(x, y)[0], 4, 1);
        EXPECT_NEAR(image.at(x, y)[1], 241, 1);
        EXPECT_NEAR(image.at(x, y)[2], 33, 1);
      };

  check_image(source_image);

  // The multi-threaded work function; render and check the image.
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
        // No rendering error occurred; test the images.
        SCOPED_TRACE(fmt::format("Worker {}", i));
        check_image(image);
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
  for (int i = 0; i < 2; ++i) {
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
