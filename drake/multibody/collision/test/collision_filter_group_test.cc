#include "drake/multibody/collision/collision_filter.h"

#include "drake/multibody/rigid_body.h"

#include "gtest/gtest.h"

// This tests the implementation of the `collision_filter_group` information
// contained in a URDF file.  The full implementation spans multiple drake
// libraries.  These tests seek to validate all aspects of the collision
// filter group functionality: from the underlying data structures, its
// interaction with the RigidBodyTree and with parsing.
namespace DrakeCollision {
namespace test {
namespace  {

// Confirms that adding two groups with the same name causes a *meaningful*
// message to be thrown.
GTEST_TEST(CollisionFilterGroupDefinition, DuplicateGroupNames) {
  CollisionFilterGroupManager<double> manager;
  const std::string group_name = "group1";
  manager.DefineCollisionFilterGroup(group_name);
  try {
    manager.DefineCollisionFilterGroup(group_name);
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg = "Attempting to create duplicate collision "
        "filter group: " + group_name;
    EXPECT_EQ(e.what(), expected_msg);
  }
}

// Confirms that the group ids assigned to the group start at 0 and increase
// by 1 and are assigned in the order the groups are "defined".
GTEST_TEST(CollisionFilterGroupDefinition, CollisionGroupAssignment) {
  CollisionFilterGroupManager<double> manager;
  const int group_count = 10;
  for (int i = 0; i < group_count; ++i) {
    std::string group_name = "group" + std::to_string(i);
    manager.DefineCollisionFilterGroup(group_name);
  }

  for (int i = 0; i < group_count; ++i) {
    std::string group_name = "group" + std::to_string(i);
    EXPECT_EQ(manager.GetGroupId(group_name), i);
  }
}

// Confirms that the attempt to create more than the maximum allowable
// number of collision filter groups throws an exception with a meaningful
// message.
GTEST_TEST(CollisionFilterGroupDefinition, CollisionGroupOverflow) {
  CollisionFilterGroupManager<double> manager;
  for (int i = 0; i < MAX_NUM_COLLISION_FILTER_GROUPS; ++i) {
    std::string group_name = "group" + std::to_string(i);
    manager.DefineCollisionFilterGroup(group_name);
  }
  try {
    manager.DefineCollisionFilterGroup("bad_group");
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg = "Requesting a collision filter group id when"
        " there are no more available. Drake only supports " +
        std::to_string(MAX_NUM_COLLISION_FILTER_GROUPS) +
        " collision filter groups per RigidBodyTree instance.";
    EXPECT_EQ(e.what(), expected_msg);
  }
}

// The same group can be added to a collision group's ignore list multiple
// times.  Although this is undesirable behavior, it is not *bad*.  It has
// the same result as only adding a single.  This test confirms that there
// is no error during the definition process.
GTEST_TEST(CollisionFilterGroupDefinition, RepeatIgnoreName) {
  CollisionFilterGroupManager<double> manager;
  std::string group_name = "group1";
  manager.DefineCollisionFilterGroup(group_name);
  manager.AddCollisionFilterIgnoreTarget(group_name, group_name);
  manager.AddCollisionFilterIgnoreTarget(group_name, group_name);
  // Note: no exception thrown implies success.
}

// The same body can be added to a collision group multiple times.  Although
// this is undesirable behavior, it is not *bad*.  It has
// the same result as only adding a single.  This test confirms that there
// is no error during the definition process.
GTEST_TEST(CollisionFilterGroupDefinition, RepeatAddBody) {
  CollisionFilterGroupManager<double> manager;
  std::string group_name = "group1";
  manager.DefineCollisionFilterGroup(group_name);
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember(group_name, body);
  manager.AddCollisionFilterGroupMember(group_name, body);
  // Note: no exception thrown implies success.
}

// Confirms that adding an ignore group to a non-existant group throws a
// meaningful exception.
GTEST_TEST(CollisionFilterGroupDefinition, AddIgnoreGroupToUndefinedGroup) {
  CollisionFilterGroupManager<double> manager;
  std::string group_name = "group1";
  try {
    manager.AddCollisionFilterIgnoreTarget(group_name, group_name);
  } catch (std::runtime_error& e) {
    std::string expected_msg =
        "Attempting to add an ignored collision filter group to an undefined "
            "collision filter group: Ignoring " +
            group_name + " by " + group_name + ".";
    EXPECT_EQ(e.what(), expected_msg);
  }
}

// Confirms that adding a RigidBody to a non-existant group reports failure.
GTEST_TEST(CollisionFilterGroupDefinition, AddBodyToUndefinedGroup) {
  CollisionFilterGroupManager<double> manager;
  std::string group_name = "group1";
  RigidBody<double> body;
  bool result = manager.AddCollisionFilterGroupMember(group_name, body);
  EXPECT_FALSE(result);
}

//---------------------------------------------------------------------------

// Tests correct bit mask for a body belonging to a single group.
GTEST_TEST(CollisionFilterGroupCompile, SingleGroupMembership) {
  CollisionFilterGroupManager<double> manager;
  const std::string group_name = "group1";
  manager.DefineCollisionFilterGroup(group_name);
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember(group_name, body);
  manager.CompileGroups();
  bitmask expected_group;
  // First group added should be group: 0.  See CollisionGroupAssignment test.
  int expected_id = 0;
  expected_group.set(expected_id);
  EXPECT_EQ(manager.get_group_mask(body), expected_group);
}

// Tests correct bit mask for a body belonging to multiple groups.
GTEST_TEST(CollisionFilterGroupCompile, MultiGroupMembership) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group0");
  manager.DefineCollisionFilterGroup("group1");
  manager.DefineCollisionFilterGroup("group2");
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.AddCollisionFilterGroupMember("group2", body);
  manager.CompileGroups();
  bitmask expected_group;
  expected_group.set(0);
  expected_group.set(2);
  EXPECT_EQ(manager.get_group_mask(body), expected_group);
}

// Tests correct ignore bit mask for a body belonging to a single group.
GTEST_TEST(CollisionFilterGroupCompile, SingleGroupIgnoreSet) {
  CollisionFilterGroupManager<double> manager;
  for (int i = 0; i < 4; ++i) {
    manager.DefineCollisionFilterGroup("group" + std::to_string(i));
  }
  // group 0 ignores itself, 2, & 3
  std::vector<int> ignores = {0, 2, 3};
  for (auto i : ignores) {
    manager.AddCollisionFilterIgnoreTarget("group0",
                                           "group" + std::to_string(i));
  }
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.CompileGroups();
  bitmask expected_ignores;
  for (auto i : ignores) {
    expected_ignores.set(i);
  }
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests correct ignore bit mask for a body belonging to a multiple groups: it
// should be the union of its groups ignores.
GTEST_TEST(CollisionFilterGroupCompile, MultiGroupIgnoreSet) {
  CollisionFilterGroupManager<double> manager;
  for (int i = 0; i < 6; ++i) {
    manager.DefineCollisionFilterGroup("group" + std::to_string(i));
  }
  // group 0 ignores itself, 2, & 3
  std::set<int> ignores0 = {0, 2, 3};
  for (auto i : ignores0) {
    manager.AddCollisionFilterIgnoreTarget("group0",
                                           "group" + std::to_string(i));
  }
  // group 1 ignores 3 & 4
  std::set<int> ignores1 = {3, 4};
  for (auto i : ignores1) {
    manager.AddCollisionFilterIgnoreTarget("group1",
                                           "group" + std::to_string(i));
  }
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.CompileGroups();
  std::vector<int> ignore_union;
  std::set_union(ignores0.begin(), ignores0.end(),
                 ignores1.begin(), ignores1.end(),
                 std::inserter(ignore_union, ignore_union.begin()));
  bitmask expected_ignores;
  for (auto i : ignore_union) {
    expected_ignores.set(i);
  }
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that references to undefined groups are simply ignored.
GTEST_TEST(CollisionFilterGroupCompile, IgnoreNonExistentGroup) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group0");
  // group1 does not exist.
  manager.AddCollisionFilterIgnoreTarget("group0", "group1");

  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.CompileGroups();

  bitmask expected_ignores = NONE_MASK;
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that references to redundant groups behaves the same as a single
// reference.
GTEST_TEST(CollisionFilterGroupCompile, IgnoreRedundantGroup) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group0");
  manager.DefineCollisionFilterGroup("group1");
  // Redundantly adds group 1.
  manager.AddCollisionFilterIgnoreTarget("group0", "group1");
  manager.AddCollisionFilterIgnoreTarget("group0", "group1");

  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.CompileGroups();

  bitmask expected_ignores = NONE_MASK;
  expected_ignores.set(1);
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that adding a body redundantly to a group has the same effect as adding
// it once.
GTEST_TEST(CollisionFilterGroupCompile, AddBodyRedundantly) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group0");
  manager.DefineCollisionFilterGroup("group1");
  // Redundantly adds group 1.
  manager.AddCollisionFilterIgnoreTarget("group0", "group1");

  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.AddCollisionFilterGroupMember("group0", body);
  manager.CompileGroups();

  bitmask expected_ignores = NONE_MASK;
  expected_ignores.set(1);
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that clearing the manager removes all data except the next group.
GTEST_TEST(CollisionFilterGroupCompile, ClearFlushesData) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group0");
  manager.DefineCollisionFilterGroup("group1");
  // Redundantly adds group 1.
  manager.AddCollisionFilterIgnoreTarget("group0", "group1");

  RigidBody<double> body1, body2;
  manager.AddCollisionFilterGroupMember("group0", body1);
  manager.AddCollisionFilterGroupMember("group0", body2);
  manager.CompileGroups();

  manager.Clear();
  // Confirms zero membership and ignore bitmasks for both bodies.
  EXPECT_EQ(manager.get_group_mask(body1), NONE_MASK);
  EXPECT_EQ(manager.get_ignore_mask(body1), NONE_MASK);
  EXPECT_EQ(manager.get_group_mask(body2), NONE_MASK);
  EXPECT_EQ(manager.get_ignore_mask(body2), NONE_MASK);
  // Confirms that the groups have been deleted.
  EXPECT_EQ(manager.GetGroupId("group0"), -1);
  EXPECT_EQ(manager.GetGroupId("group1"), -1);

  // Confirms that the available group counter has continued counting.
  manager.DefineCollisionFilterGroup("group2");
  EXPECT_EQ(manager.GetGroupId("group2"), 2);
}

//---------------------------------------------------------------------------

GTEST_TEST(CollisionFilterGroupRBT, test) {
}
}  // namespace
}  // namespace test
}  // namespace DrakeCollision
