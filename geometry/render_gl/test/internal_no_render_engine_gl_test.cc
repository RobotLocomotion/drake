#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gl/factory.h"

namespace drake {
namespace geometry {
namespace {

// Tests that attempting to use the unsupported RenderEngineGl throws.
GTEST_TEST(RenderEngineGl, NoRenderEngineGlSupport) {
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRenderEngineGl(),
                              "RenderEngineGl was not compiled. You'll need to "
                              "use a different render engine.");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
