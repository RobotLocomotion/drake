#include <gtest/gtest.h>

#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

// Tests the temporary dummy implementation of RenderEngineGl doesn't crash.
GTEST_TEST(RenderEngineGl, DummyRenderDepthImage) {
  std::unique_ptr<RenderEngine> engine = MakeRenderEngineGl();
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
