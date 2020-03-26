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
  opengl_context.MakeCurrent();
  // Tests switching contexts and enabling debug mode.
  OpenGlContext debug_context{true};
  debug_context.MakeCurrent();
  // Back to original context.
  opengl_context.MakeCurrent();
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
