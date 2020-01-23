#include <gtest/gtest.h>

#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

using systems::sensors::ImageDepth32F;

// Default camera properties.
const int kWidth = 640;
const int kHeight = 480;
const double kZNear = 0.5;
const double kZFar = 5.;
const double kFovY = M_PI_4;

// Tests the temporary dummy implementation of RenderEngineGl produces the
// expected static depth image.
GTEST_TEST(RenderEngineGl, DummyRenderDepthImage) {
  std::unique_ptr<RenderEngine> engine = MakeRenderEngineGl();
  const DepthCameraProperties camera = {kWidth,   kHeight, kFovY,
                                        "unused", kZNear,  kZFar};
  ImageDepth32F depth(kWidth, kHeight);
  engine->RenderDepthImage(camera, &depth);

  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      EXPECT_EQ(depth.at(u, v)[0], 0.5);
    }
  }
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
