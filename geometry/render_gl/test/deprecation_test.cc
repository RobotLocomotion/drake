// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.

#include <gtest/gtest.h>

#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

GTEST_TEST(RenderGlNamespaceDeprecation, SanityCheck) {
  RenderEngineGlParams params1;
  render::RenderEngineGlParams params2;
  auto engine1 = MakeRenderEngineGl(params1);
  auto engine2 = render::MakeRenderEngineGl(params2);
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
