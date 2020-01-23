#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

// Tests that attempting to use the unsupported RenderEngineGl throws.
GTEST_TEST(RenderEngineGl, NoRenderEngineGlSupport) {
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRenderEngineGl(), std::runtime_error,
                              "RenderEngineGl was not compiled. You'll need to "
                              "use a different render engine.");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
