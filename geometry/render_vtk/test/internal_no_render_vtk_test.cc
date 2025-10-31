#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_vtk/factory.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace {

GTEST_TEST(RenderEngineVtkDisabledTest, ExceptionMessage) {
  const RenderEngineVtkParams params;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRenderEngineVtk(params),
                              "RenderEngineVtk was not compiled.*");
}

}  // namespace
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
