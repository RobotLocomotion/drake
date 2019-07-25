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

// Tests the removal of all roles.
GTEST_TEST(InternalGeometryTest, RemoveRole) {
  // Configure a geometry with all roles; we assume from previous unit tests
  // that the geometry's state is correct.
  InternalGeometry geometry;
  geometry.SetRole(ProximityProperties());
  geometry.SetRole(IllustrationProperties());
  geometry.SetRole(PerceptionProperties());

  // Two notes on the structure of this test:
  //  1. As currently formulated, the correctness of the test depends on the
  //     order of these actions. Changing the order can lead to meaningless
  //     test failure.
  //  2. This test doesn't exhaustively test all permutations of removing a
  //     role. (There are 8 unique configurations and 24 total possible removal
  //     invocations.) We assume that the *suggestion* of independence suggested
  //     here is actually true.

  // Case: Remove proximity, other roles persist.
  EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Redundant removal of role is a no-op.
  EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove illustration, only perception remains.
  EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Redundant removal of role is a no-op.
  EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove perception, no roles exist.
  EXPECT_NO_THROW(geometry.RemovePerceptionRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));

  // Case: Redundant removal of role is a no-op.
  EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
