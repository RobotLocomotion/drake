// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.

#include <gtest/gtest.h>

#include "drake/geometry/render/render_engine_vtk.h"
#include "drake/geometry/render/render_engine_vtk_base.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/render/vtk_util.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

GTEST_TEST(RenderVtkPathsDeprecation, SanityCheck) {
  RenderEngineVtkParams params;
  RenderEngineVtk engine1(params);
  render::RenderEngineVtk engine2(params);
  auto engine3 = MakeRenderEngineVtk(params);
  auto engine4 = render::MakeRenderEngineVtk(params);
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
