#include "drake/geometry/internal_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

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

// Confirms that redundantly setting properties causes an exception to be
// thrown.
GTEST_TEST(InternalGeometryTest, RedundantPropertyAssignment) {
  InternalGeometry geometry;

  EXPECT_FALSE(geometry.has_proximity_role());
  EXPECT_NO_THROW(geometry.SetRole(ProximityProperties()));
  EXPECT_TRUE(geometry.has_proximity_role());
  DRAKE_EXPECT_THROWS_MESSAGE(geometry.SetRole(ProximityProperties()),
                              std::logic_error,
                              "Geometry already has proximity role assigned");
  EXPECT_TRUE(geometry.has_proximity_role());

  EXPECT_FALSE(geometry.has_illustration_role());
  EXPECT_NO_THROW(geometry.SetRole(IllustrationProperties()));
  EXPECT_TRUE(geometry.has_illustration_role());
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry.SetRole(IllustrationProperties()), std::logic_error,
      "Geometry already has illustration role assigned");
  EXPECT_TRUE(geometry.has_illustration_role());

  EXPECT_FALSE(geometry.has_perception_role());
  EXPECT_NO_THROW(geometry.SetRole(PerceptionProperties()));
  EXPECT_TRUE(geometry.has_perception_role());
  DRAKE_EXPECT_THROWS_MESSAGE(geometry.SetRole(PerceptionProperties()),
                              std::logic_error,
                              "Geometry already has perception role assigned");
  EXPECT_TRUE(geometry.has_perception_role());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
