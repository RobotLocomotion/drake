// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.

#include <gtest/gtest.h>

#include "drake/common/unused.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"

namespace {

GTEST_TEST(RenderGlNamespaceDeprecation, SanityCheck) {
  const auto has_gl_engine = drake::geometry::render::kHasRenderEngineGl;
  drake::unused(has_gl_engine);

  const drake::geometry::render::RenderEngineGlParams params;
  const auto engine = drake::geometry::render::MakeRenderEngineGl(params);
}

}  // namespace
