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

// Confirms that properties get set properly.
GTEST_TEST(InternalGeometryTest, PropertyAssignment) {
  InternalGeometry geometry;

  EXPECT_FALSE(geometry.has_proximity_role());
  EXPECT_NO_THROW(geometry.SetRole(ProximityProperties()));
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

// Tests the removal of proximity and illustration roles -- the removal is
// identical for both. Perception requires special treatment (see below).
GTEST_TEST(InternalGeometryTest, RemoveRole_NonPerception) {
  // Configure a geometry with all roles; we assume from previous unit tests
  // that the geometry's state is correct.
  InternalGeometry geometry;
  geometry.SetRole(ProximityProperties());
  geometry.SetRole(IllustrationProperties());

  // Case: Remove proximity, illustration persists.
  EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));

  // Case: Redundant removal of role is a no-op.
  EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));

  // Case: Remove illustration, no roles exist.
  EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));

  // Case: Redundant removal of role is a no-op.
  EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
}

// Test the perception role removal. This is its own unique test because it has
// special, per-renderer logic that doesn't apply to either proximity or
// illustration.
GTEST_TEST(InternalGeometryTest, RemovePerceptionRole) {
  const std::string renderer1("renderer1");
  const std::string renderer2("renderer2");

  // Configure a geometry with all roles; we assume from previous unit tests
  // that the geometry's state is correct.
  InternalGeometry geometry;
  const RenderIndex index1(10);
  const RenderIndex index2(20);
  geometry.set_render_index(renderer1, index1);
  geometry.set_render_index(renderer2, index2);
  geometry.SetRole(PerceptionProperties());

  // Case: Remove render index for a non-existent render engine; old index is
  // still valid and there are still perception properties.
  EXPECT_NO_THROW(geometry.ClearRenderIndex("invalid"));
  EXPECT_EQ(geometry.render_index(renderer1), index1);
  EXPECT_EQ(geometry.render_index(renderer2), index2);
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove render index for valid render engine; no index exists and it
  // still has perception properties.
  EXPECT_NO_THROW(geometry.ClearRenderIndex(renderer1));
  EXPECT_EQ(geometry.render_index(renderer1), nullopt);
  EXPECT_EQ(geometry.render_index(renderer2), index2);
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove perception properties while there is still a valid render
  // index. All render indices are cleared.
  EXPECT_NO_THROW(geometry.RemovePerceptionRole());
  EXPECT_EQ(geometry.render_index(renderer1), nullopt);
  EXPECT_EQ(geometry.render_index(renderer2), nullopt);
  EXPECT_FALSE(geometry.has_role(Role::kPerception));

  // Case: Removing render index leaves the geometry with *no* render indices,
  // but it *still* has perception properties.
  geometry.set_render_index(renderer1, index1);
  geometry.SetRole(PerceptionProperties());
  // Confirm it's wired up for renderer1.
  EXPECT_EQ(geometry.render_index(renderer1), index1);
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  EXPECT_NO_THROW(geometry.ClearRenderIndex(renderer1));
  EXPECT_EQ(geometry.render_index(renderer1), nullopt);
  EXPECT_TRUE(geometry.has_role(Role::kPerception));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
