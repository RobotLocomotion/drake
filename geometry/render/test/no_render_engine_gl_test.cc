#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/render_engine_gl.h"

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

// Tests that attempting to use the unsupported RenderEngineGl throws.
GTEST_TEST(RenderEngineGl, NoRenderEngineGlSupport) {
  RenderEngineGl engine;
  const DepthCameraProperties camera = {kWidth,   kHeight, kFovY,
                                        "unused", kZNear,  kZFar};
  ImageDepth32F depth(kWidth, kHeight);
  DRAKE_EXPECT_THROWS_MESSAGE(engine.RenderDepthImage(camera, &depth),
                              std::runtime_error,
                              "RenderEngineGl was not compiled. You'll need to "
                              "use a different render engine.");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
