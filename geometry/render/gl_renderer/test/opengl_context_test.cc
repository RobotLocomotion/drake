#include "drake/geometry/render/gl_renderer/opengl_context.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace render {
namespace gl {
namespace {

// Tests that an OpenGL context can be obtained.
GTEST_TEST(OpenGlContext, GetContext) {
  std::shared_ptr<OpenGlContext> opengl_context(
      std::make_shared<OpenGlContext>());
  EXPECT_TRUE(opengl_context->is_initialized());
  opengl_context->make_current();

  // Tests that a second context doesn't reinitialize GLUT.
  std::shared_ptr<OpenGlContext> multiple_context(
      std::make_shared<OpenGlContext>());
}

}  // namespace
}  // namespace gl
}  // namespace render
}  // namespace geometry
}  // namespace drake
