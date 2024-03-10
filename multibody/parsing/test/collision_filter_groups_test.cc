#include "drake/multibody/parsing/collision_filter_groups.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(CollisionFilterGroupsTest, Lifecycle) {
  const CollisionFilterGroups original;
  EXPECT_TRUE(original.empty());

  CollisionFilterGroups copied(original);
  EXPECT_TRUE(copied.empty());

  CollisionFilterGroups donor;
  donor.AddGroup("a", {"b"});
  EXPECT_FALSE(donor.empty());
  CollisionFilterGroups moved(std::move(donor));
  EXPECT_TRUE(donor.empty());
  EXPECT_FALSE(moved.empty());

  CollisionFilterGroups copy_source;

  CollisionFilterGroups copy_assigned;
  copy_assigned.AddGroup("a", {"b"});
  EXPECT_FALSE(copy_assigned.empty());
  copy_assigned = original;
  EXPECT_TRUE(copy_assigned.empty());
  EXPECT_TRUE(original.empty());

  CollisionFilterGroups move_assigned;
  EXPECT_TRUE(move_assigned.empty());
  move_assigned = std::move(moved);
  EXPECT_FALSE(move_assigned.empty());
  EXPECT_TRUE(moved.empty());
}

// Tests that the CollisionFilterGroups can be provisioned and compared for
// equality.
GTEST_TEST(CollisionFilterGroupsTest, ContentsAndComparison) {
  const CollisionFilterGroups nothing;
  EXPECT_TRUE(nothing.empty());
  EXPECT_EQ(nothing, nothing);

  CollisionFilterGroups a_group;
  a_group.AddGroup("a", {"b"});
  EXPECT_FALSE(a_group.empty());
  EXPECT_NE(nothing, a_group);
  EXPECT_EQ(a_group, a_group);

  CollisionFilterGroups a_pair;
  a_pair.AddExclusionPair({"c", "d"});
  EXPECT_FALSE(a_pair.empty());
  EXPECT_NE(nothing, a_pair);
  EXPECT_NE(a_group, a_pair);
  EXPECT_EQ(a_pair, a_pair);

  CollisionFilterGroups stuff;
  stuff.AddGroup("q", {"r", "s"});
  stuff.AddGroup("v", {"w", "x", "y"});
  stuff.AddExclusionPair({"q", "v"});
  EXPECT_FALSE(stuff.empty());
  EXPECT_NE(nothing, stuff);
  EXPECT_NE(a_group, stuff);
  EXPECT_NE(a_pair, stuff);
  EXPECT_EQ(stuff, stuff);

  // Equality compares sorted data, not address or identity.
  CollisionFilterGroups forward;
  CollisionFilterGroups backward;
  forward.AddGroup("m", {"n", "o", "p"});
  forward.AddGroup("k", {"l"});
  backward.AddGroup("k", {"l"});
  backward.AddGroup("m", {"p", "o", "n"});
  forward.AddExclusionPair({"e", "f"});
  forward.AddExclusionPair({"g", "h"});
  backward.AddExclusionPair({"g", "h"});
  backward.AddExclusionPair({"e", "f"});
  EXPECT_EQ(forward, backward);
}

// Tests that the CollisionFilterGroups can be dumped for debugging.
GTEST_TEST(CollisionFilterGroupsTest, Dump) {
  const char expected_dump[] = R"""(
Collision filter groups:
    q
        r
        s
    v
        w
        x
        y
Collision filter exclusion pairs:
    q, v
)""";

  CollisionFilterGroups stuff;
  stuff.AddGroup("q", {"r", "s"});
  stuff.AddGroup("v", {"w", "x", "y"});
  stuff.AddExclusionPair({"q", "v"});
  EXPECT_EQ(fmt::format("{}", stuff), expected_dump);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
