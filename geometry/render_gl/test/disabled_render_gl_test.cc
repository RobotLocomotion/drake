#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gl/factory.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace {

GTEST_TEST(RenderEngineGlDisabledTest, ExceptionMessage) {
  const RenderEngineGlParams params;
  DRAKE_EXPECT_THROWS_MESSAGE(MakeRenderEngineGl(params),
                              "RenderEngineGl was not compiled.*");
}

}  // namespace
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
