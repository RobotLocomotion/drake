#include "drake/geometry/internal_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

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
  geometry.set_renderer(renderer1);
  geometry.set_renderer(renderer2);
  geometry.SetRole(PerceptionProperties());
  EXPECT_TRUE(geometry.in_renderer(renderer1));
  EXPECT_TRUE(geometry.in_renderer(renderer2));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove from a non-existent render engine; its perception role
  // configuration should remain unchanged.
  EXPECT_NO_THROW(geometry.ClearRenderer("invalid"));
  EXPECT_TRUE(geometry.in_renderer(renderer1));
  EXPECT_TRUE(geometry.in_renderer(renderer2));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove from a valid render engine; otherwise unchanged perception
  // role configuration.
  EXPECT_NO_THROW(geometry.ClearRenderer(renderer1));
  EXPECT_FALSE(geometry.in_renderer(renderer1));
  EXPECT_TRUE(geometry.in_renderer(renderer2));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove perception properties while there is still a valid render
  // engine. The remaining valid render engines are cleared.
  EXPECT_NO_THROW(geometry.RemovePerceptionRole());
  EXPECT_FALSE(geometry.in_renderer(renderer1));
  EXPECT_FALSE(geometry.in_renderer(renderer2));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));

  // Case: Removing from last render engine leaves the geometry with perception
  // properties.
  // In preparation, we reassign perception role and render engine.
  geometry.set_renderer(renderer1);
  geometry.SetRole(PerceptionProperties());
  // Confirm it's wired up for renderer1.
  EXPECT_TRUE(geometry.in_renderer(renderer1));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  EXPECT_NO_THROW(geometry.ClearRenderer(renderer1));
  EXPECT_FALSE(geometry.in_renderer(renderer1));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
