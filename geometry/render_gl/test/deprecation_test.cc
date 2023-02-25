// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.
// TODO(zachfang): Remove this along with deprecation on 2023-07-01.

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace {

GTEST_TEST(RenderGlNamespaceDeprecation, SanityCheck) {
  const auto has_gl_engine = drake::geometry::render::kHasRenderEngineGl;
  const drake::geometry::render::RenderEngineGlParams params;

  if (has_gl_engine) {
    const auto engine = drake::geometry::render::MakeRenderEngineGl(params);
  } else {
    // We should be able to call the function but expect an exception.
    DRAKE_EXPECT_THROWS_MESSAGE(
        drake::geometry::render::MakeRenderEngineGl(params),
        ".*RenderEngineGl was not compiled*");
  }
}

}  // namespace
