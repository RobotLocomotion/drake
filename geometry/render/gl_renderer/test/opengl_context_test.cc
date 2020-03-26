#include "drake/geometry/render/gl_renderer/opengl_context.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

// Tests that an OpenGL context can be obtained.
GTEST_TEST(OpenGlContext, GetContext) {
  OpenGlContext opengl_context{};
  EXPECT_FALSE(glIsEnabled(GL_DEBUG_OUTPUT));
  opengl_context.MakeCurrent();
  // Test switching contexts and enabling debug mode.
  OpenGlContext debug_context{true};
  EXPECT_TRUE(glIsEnabled(GL_DEBUG_OUTPUT));
  debug_context.MakeCurrent();
  // Enable something on the current context.
  EXPECT_FALSE(glIsEnabled(GL_BLEND));
  glEnable(GL_BLEND);
  EXPECT_TRUE(glIsEnabled(GL_BLEND));
  // Back to original context.
  opengl_context.MakeCurrent();
  // The previous enable should have no effect on this context.
  EXPECT_FALSE(glIsEnabled(GL_BLEND));
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
