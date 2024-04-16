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

// Tests the MergeCollisionFilterGroups function.
GTEST_TEST(CollisionFilterGroups, Merge) {
  using internal::MergeCollisionFilterGroups;

  // Use some arbitrary reversible conversions.
  auto double_to_int = [](const double& x) { return x * 32; };
  auto int_to_double = [](const int& x) { return x / 32.0; };

  CollisionFilterGroupsBase<int> int_groups1;
  int_groups1.AddGroup(1, {2});
  int_groups1.AddGroup(3, {4});
  int_groups1.AddExclusionPair({1, 3});

  CollisionFilterGroupsBase<int> int_groups2;
  int_groups2.AddGroup(5, {6});
  int_groups2.AddGroup(7, {8});
  int_groups2.AddExclusionPair({5, 7});

  // Cram the two test objects into an object of different type.
  CollisionFilterGroupsBase<double> double_groups;
  MergeCollisionFilterGroups<double, int>(
      &double_groups, int_groups1, int_to_double);
  MergeCollisionFilterGroups<double, int>(
      &double_groups, int_groups2, int_to_double);
  // Recover the data into a new empty group of the original type.
  CollisionFilterGroupsBase<int> int_groups3;
  MergeCollisionFilterGroups<int, double>(
      &int_groups3, double_groups, double_to_int);

  // Compare the recovered data with what was expected.
  CollisionFilterGroupsBase<int> expected;
  expected.AddGroup(1, {2});
  expected.AddGroup(3, {4});
  expected.AddGroup(5, {6});
  expected.AddGroup(7, {8});
  expected.AddExclusionPair({1, 3});
  expected.AddExclusionPair({5, 7});
  EXPECT_EQ(int_groups3, expected);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
