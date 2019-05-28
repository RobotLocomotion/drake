#include "drake/geometry/internal_geometry.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(InternalGeometryTest, RenderIndexAccess) {
  InternalGeometry geometry;

  const std::string renderer_name{"valid"};
  EXPECT_FALSE(geometry.render_index(renderer_name));
  RenderIndex index(2);
  EXPECT_NO_THROW(geometry.set_render_index(renderer_name, index));
  ASSERT_TRUE(geometry.render_index(renderer_name));
  EXPECT_EQ(geometry.render_index(renderer_name), index);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
