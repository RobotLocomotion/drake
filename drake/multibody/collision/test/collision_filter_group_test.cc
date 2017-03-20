#include "drake/multibody/collision/collision_filter.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/drake_joints.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"

// This tests the implementation of the `collision_filter_group` information
// contained in a URDF file.  The full implementation spans multiple drake
// libraries.  These tests seek to validate all aspects of the collision
// filter group functionality: from the underlying data structures, their
// interactions with the RigidBodyTree, parsing, and in the collision detection
// model.
//
// Because collision filter groups are ultimately represented with bitmasks,
// expected mask values are initialized using binary representations of numbers.
// For example, a bit mask that has groups 0, 1, and 4 set would equate to
// the binary value:
//    Value: 0b10011
//    Group:   43210,
// where groups are enumerated from *right* to *left*.
namespace DrakeCollision {
namespace test {
namespace {

using drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using Eigen::Isometry3d;
using Eigen::VectorXd;
using std::unique_ptr;
using std::move;

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
    std::string expected_msg =
        "Attempting to create duplicate collision "
        "filter group: " +
        group_name + ".";
    EXPECT_EQ(e.what(), expected_msg);
  }
}

// Confirms that the group ids assigned to the group start at 0 and increase
// by 1 and are assigned in the order the groups are "defined".
GTEST_TEST(CollisionFilterGroupDefinition, CollisionGroupAssignment) {
  CollisionFilterGroupManager<double> manager;
  const int group_count = 10;
  for (int i = 0; i < group_count; ++i) {
    std::string group_name = "group" + std::to_string(i + 1);
    manager.DefineCollisionFilterGroup(group_name);
  }

  for (int i = 0; i < group_count; ++i) {
    std::string group_name = "group" + std::to_string(i + 1);
    EXPECT_EQ(manager.GetGroupId(group_name), i + 1);
  }
}

// Confirms that the attempt to create more than the maximum allowable
// number of collision filter groups throws an exception with a meaningful
// message.
GTEST_TEST(CollisionFilterGroupDefinition, CollisionGroupOverflow) {
  CollisionFilterGroupManager<double> manager;
  for (int i = 1; i < kMaxNumCollisionFilterGroups; ++i) {
    std::string group_name = "group" + std::to_string(i);
    manager.DefineCollisionFilterGroup(group_name);
  }
  try {
    manager.DefineCollisionFilterGroup("bad_group");
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg =
        "Requesting a collision filter group id when"
        " there are no more available. Drake only supports " +
        std::to_string(kMaxNumCollisionFilterGroups) +
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

// Confirms that adding an ignore group to a non-existent group throws a
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

// Confirms that adding a RigidBody to a non-existent group reports failure.
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
  // First group added should be group: 1 and 0 (default).
  // See CollisionGroupAssignment test. See file documentation for explanation
  // of binary value.
  bitmask expected_group(0b11);
  EXPECT_EQ(manager.get_group_mask(body), expected_group);
}

// Tests correct bit mask for a body belonging to multiple groups.
GTEST_TEST(CollisionFilterGroupCompile, MultiGroupMembership) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group1");
  manager.DefineCollisionFilterGroup("group2");
  manager.DefineCollisionFilterGroup("group3");
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.AddCollisionFilterGroupMember("group3", body);
  manager.CompileGroups();
  // See file documentation for explanation of binary value.
  bitmask expected_group(0b1011);
  EXPECT_EQ(manager.get_group_mask(body), expected_group);
}

// Tests correct ignore bit mask for a body belonging to a single group.
GTEST_TEST(CollisionFilterGroupCompile, SingleGroupIgnoreSet) {
  CollisionFilterGroupManager<double> manager;
  for (int i = 1; i <= 4; ++i) {
    manager.DefineCollisionFilterGroup("group" + std::to_string(i));
  }
  // group 1 ignores itself, 3, & 4
  std::vector<int> ignores = {1, 3, 4};
  for (auto i : ignores) {
    manager.AddCollisionFilterIgnoreTarget("group1",
                                           "group" + std::to_string(i));
  }
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group1", body);
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
  for (int i = 1; i <= 6; ++i) {
    manager.DefineCollisionFilterGroup("group" + std::to_string(i));
  }
  // group 1 ignores itself, 3, & 4
  std::set<int> ignores0 = {1, 3, 4};
  for (auto i : ignores0) {
    manager.AddCollisionFilterIgnoreTarget("group1",
                                           "group" + std::to_string(i));
  }
  // group 2 ignores 4 & 5
  std::set<int> ignores1 = {4, 5};
  for (auto i : ignores1) {
    manager.AddCollisionFilterIgnoreTarget("group2",
                                           "group" + std::to_string(i));
  }
  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.AddCollisionFilterGroupMember("group2", body);
  manager.CompileGroups();
  std::vector<int> ignore_union;
  std::set_union(ignores0.begin(), ignores0.end(), ignores1.begin(),
                 ignores1.end(),
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
  manager.DefineCollisionFilterGroup("group1");
  // group2 does not exist.
  manager.AddCollisionFilterIgnoreTarget("group1", "group2");

  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.CompileGroups();

  bitmask expected_ignores = kNoneMask;
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that references to redundant groups behave the same as a single
// reference.
GTEST_TEST(CollisionFilterGroupCompile, IgnoreRedundantGroup) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group1");
  manager.DefineCollisionFilterGroup("group2");
  // Redundantly adds group 1.
  manager.AddCollisionFilterIgnoreTarget("group1", "group2");
  manager.AddCollisionFilterIgnoreTarget("group1", "group2");

  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.CompileGroups();

  // See file documentation for explanation of binary value.
  bitmask expected_ignores(0b100);
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that adding a body redundantly to a group has the same effect as adding
// it once.
GTEST_TEST(CollisionFilterGroupCompile, AddBodyRedundantly) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group1");
  manager.DefineCollisionFilterGroup("group2");
  // Redundantly adds group 1.
  manager.AddCollisionFilterIgnoreTarget("group1", "group2");

  RigidBody<double> body;
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.AddCollisionFilterGroupMember("group1", body);
  manager.CompileGroups();

  // See file documentation for explanation of binary value.
  bitmask expected_ignores(0b100);
  EXPECT_EQ(manager.get_ignore_mask(body), expected_ignores);
}

// Tests that clearing the manager removes all data except the next group.
GTEST_TEST(CollisionFilterGroupCompile, ClearFlushesData) {
  CollisionFilterGroupManager<double> manager;
  manager.DefineCollisionFilterGroup("group1");
  manager.DefineCollisionFilterGroup("group2");
  // Redundantly adds group 1.
  manager.AddCollisionFilterIgnoreTarget("group1", "group2");

  RigidBody<double> body1, body2;
  manager.AddCollisionFilterGroupMember("group1", body1);
  manager.AddCollisionFilterGroupMember("group1", body2);
  manager.CompileGroups();

  manager.Clear();
  // Confirms zero membership and ignore bitmasks for both bodies.
  EXPECT_EQ(manager.get_group_mask(body1), kNoneMask);
  EXPECT_EQ(manager.get_ignore_mask(body1), kNoneMask);
  EXPECT_EQ(manager.get_group_mask(body2), kNoneMask);
  EXPECT_EQ(manager.get_ignore_mask(body2), kNoneMask);
  // Confirms that the groups have been deleted.
  EXPECT_EQ(manager.GetGroupId("group1"),
            CollisionFilterGroupManager<double>::kInvalidGroupId);
  EXPECT_EQ(manager.GetGroupId("group2"),
            CollisionFilterGroupManager<double>::kInvalidGroupId);

  // Confirms that the available group counter has continued counting.
  manager.DefineCollisionFilterGroup("group3");
  EXPECT_EQ(manager.GetGroupId("group3"), 3);
}

//---------------------------------------------------------------------------

// This tests the functionality of the DrakeCollision::Element::CanCollideWith
// method.  Assuming the bit masks have been set properly, this confirms they
// are interpreted properly.
GTEST_TEST(CollisionFilterGroupElement, ElementCanCollideWithTest) {
  DrakeCollision::Element e1;
  DrakeCollision::Element e2;

  // Case 1: By default, elements belong to no group and ignore nothing.
  EXPECT_EQ(e1.get_collision_filter_group(), kDefaultGroup);
  EXPECT_EQ(e1.get_collision_filter_ignores(), kNoneMask);

  // Case 2: Two elements, belonging to the same group (which does *not*
  // ignore itself) are considered a viable collision pair.
  e1.set_collision_filter(1, 0);
  e2.set_collision_filter(1, 0);
  EXPECT_TRUE(e1.CanCollideWith(&e2));
  EXPECT_TRUE(e2.CanCollideWith(&e1));

  // Case 3: Two elements, belonging to different groups which don't ignore each
  // other are considered a viable collision pair.
  e1.set_collision_filter(1, 0);
  e2.set_collision_filter(2, 0);
  EXPECT_TRUE(e1.CanCollideWith(&e2));
  EXPECT_TRUE(e2.CanCollideWith(&e1));

  // Case 4: Two elements, belonging to different groups, where one group
  // ignores the other are *not* considered a viable collision pair.
  e1.set_collision_filter(1, 2);
  e2.set_collision_filter(2, 0);
  EXPECT_FALSE(e1.CanCollideWith(&e2));
  EXPECT_FALSE(e2.CanCollideWith(&e1));

  // Case 5: Two elements, belonging to different groups, which ignore each
  // other, are *not* considered a viable collision pair.
  e1.set_collision_filter(1, 2);
  e2.set_collision_filter(2, 1);
  EXPECT_FALSE(e1.CanCollideWith(&e2));
  EXPECT_FALSE(e2.CanCollideWith(&e1));
}

//---------------------------------------------------------------------------

// Tests RBT
//  - Adding a body with registered collision elements throws an exception.
// TODO(SeanCurtis-TRI): Still to determine if I allow post-hoc editing. In
// order to do so, I need to support modifications of the underlying model
// (a la RBT::updateCollisionTransform.)

// This test confirms that when a body is being added to an non-existent
// group through the RigidBodyTree interface, that a meaningful exception is
// thrown.
GTEST_TEST(CollisionFilterGroupRBT, AddBodyToUndefinedGroup) {
  RigidBodyTree<double> tree;
  RigidBody<double>* body_ref;
  unique_ptr<RigidBody<double>> body(body_ref = new RigidBody<double>());
  body->set_name("body");
  body->set_model_instance_id(27);
  tree.add_rigid_body(move(body));
  std::string group_name = "no-such-group";
  try {
    tree.AddCollisionFilterGroupMember(group_name, body_ref->get_name(),
                                       body_ref->get_model_instance_id());
    GTEST_FAIL();
  } catch (std::runtime_error& e) {
    std::string expected_msg =
        "Attempting to add a link to an undefined collision filter group: "
        "Adding " +
        body_ref->get_name() + " to " + group_name + ".";
    EXPECT_EQ(e.what(), expected_msg);
  }
}

// This test confirms that the collision filter groups are set correctly
// on the collision elements.
GTEST_TEST(CollisionFilterGroupRBT, CollisionElementSetFilters) {
  // Builds the tree.
  RigidBodyTree<double> tree;

  RigidBody<double>* body1;
  unique_ptr<RigidBody<double>> body(body1 = new RigidBody<double>());
  body->set_name("body1");
  body->set_model_instance_id(27);
  unique_ptr<DrakeJoint> unique_joint(
      new FixedJoint("joint1", Isometry3d::Identity()));
  body->setJoint(move(unique_joint));
  body->set_parent(&tree.world());
  tree.add_rigid_body(move(body));

  RigidBody<double>* body2;
  body.reset(body2 = new RigidBody<double>());
  body->set_name("body2");
  body->set_model_instance_id(27);
  unique_joint.reset(new FixedJoint("joint2", Isometry3d::Identity()));
  body->setJoint(move(unique_joint));
  body->set_parent(&tree.world());
  tree.add_rigid_body(move(body));

  // Adds collision elements.
  Element element(DrakeShapes::Sphere(1.0));
  // This is not a *filter* group.  This is a designation used to select
  // so-called "terrain points".  See RigidBodyTree::getTerrainContactPoints().
  std::string collision_group = "";
  tree.addCollisionElement(element, *body1, collision_group);
  tree.addCollisionElement(element, *body2, collision_group);

  // Sets up collision filter groups.
  std::string group_name1 = "test-group1";
  std::string group_name2 = "test-group2";
  tree.DefineCollisionFilterGroup(group_name1);
  tree.AddCollisionFilterGroupMember(group_name1, body1->get_name(),
                                     body1->get_model_instance_id());
  tree.AddCollisionFilterIgnoreTarget(group_name1, group_name2);
  tree.DefineCollisionFilterGroup(group_name2);
  tree.AddCollisionFilterGroupMember(group_name2, body2->get_name(),
                                     body2->get_model_instance_id());

  tree.compile();
  // Tests the state of the collision filters.
  // See file documentation for explanation of binary value.
  bitmask expected_group1(0b11), expected_ignore1(0b100);
  for (auto itr = body1->collision_elements_begin();
       itr != body1->collision_elements_end(); ++itr) {
    EXPECT_EQ((*itr)->get_collision_filter_group(), expected_group1);
    EXPECT_EQ((*itr)->get_collision_filter_ignores(), expected_ignore1);
  }

  bitmask expected_group2(0b101);
  for (auto itr = body2->collision_elements_begin();
       itr != body2->collision_elements_end(); ++itr) {
    EXPECT_EQ((*itr)->get_collision_filter_group(), expected_group2);
    EXPECT_EQ((*itr)->get_collision_filter_ignores(), kNoneMask);
  }
}

//---------------------------------------------------------------------------

// Parses a URDF file with an inherent geometric collision which is filtered
// out using a single collision filter group with multiple members (which
// ignores itself.)
GTEST_TEST(CollisionFilterGroupURDF, ParseMultiMemberTest) {
  RigidBodyTree<double> tree;
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/collision/test/"
          "collision_filter_group_test_multi_member.urdf",
      drake::multibody::joints::kRollPitchYaw, &tree);
  Eigen::Matrix<double, 16, 1> state;
  state << 0, 0, 0, 0, 0, 0, 0, 0,  // x_0, rpy_0, joint12, joint23
      0, 0, 0, 0, 0, 0, 0, 0;  // x_dot_0, omega_0, joint12_dot, joint23_dot

  VectorXd q = state.topRows(8);
  VectorXd v = state.bottomRows(8);
  auto kinematics_cache = tree.doKinematics(q, v);
  std::vector<PointPair> pairs =
      tree.ComputeMaximumDepthCollisionPoints(kinematics_cache, false);
  EXPECT_EQ(pairs.size(), 0u);
}

// Parses a URDF file with an inherent geometric collision which is filtered
// out using one collision filter group which ignores multiple other collision
// filter groups (each with a body in it).
GTEST_TEST(CollisionFilterGroupURDF, ParseMultiIgnoreTest) {
  RigidBodyTree<double> tree;
  AddModelInstanceFromUrdfFileToWorld(
      drake::GetDrakePath() +
          "/multibody/collision/test/"
          "collision_filter_group_test_multi_ignore.urdf",
      drake::multibody::joints::kRollPitchYaw, &tree);
  Eigen::Matrix<double, 16, 1> state;
  state << 0, 0, 0, 0, 0, 0, 0, 0,  // x_0, rpy_0, joint12, joint23
      0, 0, 0, 0, 0, 0, 0, 0;  // x_dot_0, omega_0, joint12_dot, joint23_dot

  VectorXd q = state.topRows(8);
  VectorXd v = state.bottomRows(8);
  auto kinematics_cache = tree.doKinematics(q, v);
  std::vector<PointPair> pairs =
      tree.ComputeMaximumDepthCollisionPoints(kinematics_cache, false);
  EXPECT_EQ(pairs.size(), 0u);
}
}  // namespace
}  // namespace test
}  // namespace DrakeCollision
