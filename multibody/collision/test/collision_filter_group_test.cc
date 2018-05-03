/* clang-format off to disable clang-format-includes */
#include "drake/multibody/collision/collision_filter.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/joints/drake_joints.h"
#include "drake/multibody/parsers/sdf_parser.h"
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
namespace drake {
namespace multibody {
namespace collision {
namespace test {
namespace {

using drake::FindResourceOrThrow;
using drake::parsers::sdf::AddModelInstancesFromSdfFileToWorld;
using drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

// Confirms that adding two groups with the same name causes a *meaningful*
// message to be thrown.
GTEST_TEST(CollisionFilterGroupDefinition, DuplicateGroupNames) {
  CollisionFilterGroupManager<double> manager;
  const std::string group_name = "group1";
  manager.DefineCollisionFilterGroup(group_name);
  DRAKE_EXPECT_THROWS_MESSAGE(
      manager.DefineCollisionFilterGroup(group_name), std::logic_error,
      "Attempting to create duplicate collision filter group: .+");
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
  DRAKE_EXPECT_THROWS_MESSAGE(
      manager.AddCollisionFilterIgnoreTarget(group_name, group_name),
      std::logic_error,
      "Attempting to add an ignored collision filter group to an undefined "
          "collision filter group: Ignoring " +
          group_name + " by " + group_name + ".");
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

// This tests the functionality of the
// drake::multibody::collision::Element::CanCollideWith method.  Assuming the
// bit masks have been set properly, this confirms they are interpreted
// properly.
GTEST_TEST(CollisionFilterGroupElement, ElementCanCollideWithTest) {
  drake::multibody::collision::Element e1;
  drake::multibody::collision::Element e2;

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

// Helper function to create dummy rigid bodies. The body is joined to a parent
// body. The joint can either be fixed or not. If the body is fixed, the child
// body gets welded to the parent in the compilation stage. If no parent is
// specified, the world body is the parent.
// NOTE: bodies that are fixed to the world are marked as "anchored" and
// anchored geometries will never collide with other anchored geometries.
// The new body will be owned by the given tree but a reference to that body
// will be returned.
RigidBody<double>& MakeDummyBody(const std::string& name, bool is_fixed,
                                 RigidBodyTree<double>* tree,
                                 RigidBody<double>* parent = nullptr) {
  DRAKE_DEMAND(tree != nullptr);
  auto body = make_unique<RigidBody<double>>();
  body->set_name(name);
  body->set_model_instance_id(27);
  if (!is_fixed) {
    // Garbage non-zero inertia to prevent the body from being welded to parent.
    // Otherwise, the floating joint gets welded to world.
    body->set_spatial_inertia(drake::SquareTwistMatrix<double>::Constant(1));
  }
  unique_ptr<DrakeJoint> unique_joint(new RevoluteJoint(
      name + "_joint", Isometry3d::Identity(), Vector3d::UnitX()));
  body->setJoint(move(unique_joint));
  body->set_parent(parent ? parent : &tree->world());
  return *tree->add_rigid_body(move(body));
}

// Performs collision detection on the system where the tree configuration is
// set to the zero state.
std::vector<PointPair<double>> CollideAtZero(RigidBodyTree<double>* tree_ptr) {
  RigidBodyTree<double>& tree = *tree_ptr;
  // Simply initialize everything to the "zero" position.
  const int q_size = tree.get_num_positions();
  const int v_size = tree.get_num_velocities();
  VectorXd q = VectorXd::Zero(q_size);
  VectorXd v = VectorXd::Zero(v_size);
  auto kinematics_cache = tree.doKinematics(q, v);
  return tree.ComputeMaximumDepthCollisionPoints(kinematics_cache, false);
}

// This test confirms that when a body is being added to an non-existent
// group through the RigidBodyTree interface, that a meaningful exception is
// thrown.
GTEST_TEST(CollisionFilterGroupRBT, AddBodyToUndefinedGroup) {
  RigidBodyTree<double> tree;
  RigidBody<double>& body = MakeDummyBody("body", true /*is_fixed*/, &tree);
  std::string group_name = "no-such-group";
  DRAKE_EXPECT_THROWS_MESSAGE(
      tree.AddCollisionFilterGroupMember(group_name, body.get_name(),
                                         body.get_model_instance_id()),
      std::logic_error,
      "Attempting to add a link to an undefined collision filter group.*");
}

// This test confirms that the collision filter groups are set correctly
// on the collision elements.
GTEST_TEST(CollisionFilterGroupRBT, CollisionElementSetFilters) {
  // Builds the tree.
  RigidBodyTree<double> tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));
  RigidBody<double>& body1 = MakeDummyBody("body1", is_fixed, &tree);
  RigidBody<double>& body2 = MakeDummyBody("body2", is_fixed, &tree);

  // This is not a *filter* group.  This is a designation used to select
  // so-called "terrain points".  See RigidBodyTree::getTerrainContactPoints().
  std::string collision_group = "";
  tree.addCollisionElement(element, body1, collision_group);
  tree.addCollisionElement(element, body2, collision_group);

  // Sets up collision filter groups.
  std::string group_name1 = "test-group1";
  std::string group_name2 = "test-group2";
  tree.DefineCollisionFilterGroup(group_name1);
  tree.AddCollisionFilterGroupMember(group_name1, body1.get_name(),
                                     body1.get_model_instance_id());
  tree.AddCollisionFilterIgnoreTarget(group_name1, group_name2);
  tree.DefineCollisionFilterGroup(group_name2);
  tree.AddCollisionFilterGroupMember(group_name2, body2.get_name(),
                                     body2.get_model_instance_id());

  tree.compile();

  // Tests the state of the collision filters.
  // See file documentation for explanation of binary value.
  bitmask expected_group1(0b11), expected_ignore1(0b100);
  for (auto itr = body1.collision_elements_begin();
       itr != body1.collision_elements_end(); ++itr) {
    EXPECT_EQ((*itr)->get_collision_filter_group(), expected_group1);
    EXPECT_EQ((*itr)->get_collision_filter_ignores(), expected_ignore1);
  }

  bitmask expected_group2(0b101);
  for (auto itr = body2.collision_elements_begin();
       itr != body2.collision_elements_end(); ++itr) {
    EXPECT_EQ((*itr)->get_collision_filter_group(), expected_group2);
    EXPECT_EQ((*itr)->get_collision_filter_ignores(), kNoneMask);
  }
}

// This confirms that a collision element can be added after compilation.
// The number of collisions goes from 0 -> 1 -> 2 as colliding elements are
// added.
GTEST_TEST(CollisionFilterGroupRBT, AddElementPostCompileNoFilter) {
  RigidBodyTree<double> tree;
  const bool is_fixed = true;
  RigidBody<double>& body = MakeDummyBody("body1", is_fixed, &tree);
  tree.compile();
  EXPECT_EQ(body.get_num_collision_elements(), 0);

  // Add to body with *no* collision elements.
  Element element(DrakeShapes::Sphere(1.0));
  EXPECT_NO_THROW(tree.addCollisionElement(element, body, ""));
  tree.compile();
  EXPECT_EQ(body.get_num_collision_elements(), 1);

  // Add to body with *some* collision elements.
  EXPECT_NO_THROW(tree.addCollisionElement(element, body, ""));
  tree.compile();
  EXPECT_EQ(body.get_num_collision_elements(), 2);
}

// Tests addition of collision filter group after compilation. No new bodies are
// added, but compiled bodies' filter groups are modified. The parsed file has
// no filter groups and one is added.
GTEST_TEST(CollisionFilterGroupRBT, PostCompileGroupAdded) {
  RigidBodyTree<double> tree;
  const bool do_compile = true;
  const std::string file_name = "drake/multibody/collision/test/"
      "no_filter_groups.urdf";
  AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(file_name),
                                      joints::kRollPitchYaw, do_compile, &tree);
  std::vector<PointPair<double>> pairs = CollideAtZero(&tree);
  ASSERT_EQ(pairs.size(), 1u);

  // Now put both bodies into a self-ignoring collision filter group.
  const std::string group_name{"added_group"};
  tree.DefineCollisionFilterGroup(group_name);
  tree.AddCollisionFilterIgnoreTarget(group_name, group_name);
  tree.AddCollisionFilterGroupMember(group_name, "sphereA", 0);
  tree.AddCollisionFilterGroupMember(group_name, "sphereB", 0);
  tree.compile();
  pairs = CollideAtZero(&tree);
  ASSERT_EQ(pairs.size(), 0u);
}

// Add three bodies (each with a unit sphere for collision). The spheres are
// overlapping. However, spheres A & B are in a self-filtering group. A & B
// can both collide with C. It should report two collisions (pairs (A, C) and
// (B, C). Adding an additional collision element to B should produce *three*
// contacts (A, C), (B0, C), (B1, C).
GTEST_TEST(CollisionFilterGroupRBT, AddedElementInheritsFilter) {
  RigidBodyTree<double> tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& bodyA = MakeDummyBody("bodyA", is_fixed, &tree);
  tree.addCollisionElement(element, bodyA, "");
  RigidBody<double>& bodyB = MakeDummyBody("bodyB", is_fixed, &tree);
  tree.addCollisionElement(element, bodyB, "");
  RigidBody<double>& bodyC = MakeDummyBody("bodyC", is_fixed, &tree);
  tree.addCollisionElement(element, bodyC, "");

  const std::string group_name{"self_filter_group"};
  tree.DefineCollisionFilterGroup(group_name);
  tree.AddCollisionFilterIgnoreTarget(group_name, group_name);
  tree.AddCollisionFilterGroupMember(group_name, "bodyA",
                                     bodyA.get_model_instance_id());
  tree.AddCollisionFilterGroupMember(group_name, "bodyB",
                                     bodyB.get_model_instance_id());

  tree.compile();
  std::vector<PointPair<double>> pairs = CollideAtZero(&tree);
  EXPECT_EQ(pairs.size(), 2u);
  auto expect_pair = [&bodyA, &bodyB, &bodyC](const PointPair<double>& pair) {
    const RigidBody<double>* body1 = pair.elementA->get_body();
    const RigidBody<double>* body2 = pair.elementB->get_body();
    if (body1 == &bodyA || body1 == &bodyB) {
      EXPECT_EQ(body2, &bodyC) << "Bodies A and B should only collide with C";
    } else if (body1 == &bodyC) {
      EXPECT_TRUE(body2 == &bodyA || body2 == &bodyB);
    } else {
      GTEST_FAIL() << "Collision included an unknown body: "
          << body1->get_name() << " and " << body2->get_name();
    }
  };
  expect_pair(pairs[0]);
  expect_pair(pairs[1]);

  // Now add a new collision element to B.
  tree.addCollisionElement(element, bodyB, "");
  tree.compile();
  pairs = CollideAtZero(&tree);
  ASSERT_EQ(pairs.size(), 3u);
  expect_pair(pairs[0]);
  expect_pair(pairs[1]);
  expect_pair(pairs[2]);
}

// Clique testing
//
// Post-compilation modifications can affect collision cliques numerous ways.
// The various *actions* can have a number of effects depending on what the
// compiled state of the RBT is. The basic principles are:
//
// 1. Elements on "adjacent" bodies will always have a common clique.
// 2. Multiple elements on a single body will always have a common clique.
//
//  - Action 1: Adding a single collision element to a single compiled body
//
//   - State 1: Body has no adjacent bodies and
//    - State 1a: Body has no previously compiled elements.
//    - Result 1a: Body has one compiled element with *no* collision cliques.
//    - State 1b: Body has *one* compiled element.
//    - Result 1b: Body has two compiled elements, each with a single common
//                 collision clique.
//    - State 1c: Body has *multiple* (N) compiled elements.
//    - Result 1c: Body has N+1 compiled elements, each with the single clique
//                 that the original N compiled elements had.
//
//   - State 2: Body has an adjacent body with one compiled collision element
//    - State 2a: Body has no previously compiled elements
//    - Result 2a: Body has one compiled element with a single clique; that
//                 clique is _added_ to adjacent collision element.
//    - State 2b: Body has one or more (N) previously compiled elements
//    - Result 2b: Body has N+1 compiled elements, each with the same cliques
//                 as the original N compiled elements had.
//
//   - State 3: Add collision elements to two adjacent bodies, A & B. Neither A
//              nor B are adjacent to any other bodies.
//    - State 3a: Neither body has previously compiled elements.
//    - Result 3a: Both bodies have 1 collision element, each with a single,
//                 common clique.
//    - State 3b: One body (body A) has one previously compiled element, body B
//                has none.
//    - Result 3b: Body A has two compiled elements, body B has one. All three
//                 elements have a single, common clique.
//    - State 3c: Body A has multiple (N) previously compiled elements (each
//                with a single common clique, C). Body B has no elements.
//    - Result 3c: Body A has N+1 compiled collision elements. Body B has one.
//                 The N Elements on body A have two cliques, C and C₂. The
//                 one element on B has a single clique, C₂.
//    - State 3d: Body A has one or more (N) previously compiled elements. Body
//                B has one or more (M) previously compiled elements. Elements
//                of A and B all have a single clique, C.
//    - Result 3d: Body A and B have N+1 and M+1 compiled elements,
//                 respectively. All elements have a single clique, C.
//
//  - Action 2: Attaching a new body A (with a single uncompiled collision
//              element) to a previously compiled body B (e.g., putting a
//              gripper on an arm), with a non-welding joint.
//    - State 4: Body B has compiled elements.
//    - Result 4: Body A has a single compiled element with no cliques.
//    - State 5: Body B has one or more (N) previously compiled elements each
//               with the set of cliques ℂ.
//    - Result 5: Body A has one compiled element and body B has N compiled
//                elements. The element on A has clique C. Each element on B
//                has the set of cliques ℂ ∪ {C}.
//
// The following tests evaluate these conditions.

// Tests the cases where a collision element is added to a previously-compiled
// body. It accounts for states 1a-1d listed above.
GTEST_TEST(PostCompileCliqueRBT, AddElementToIsolatedBody) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& bodyA = MakeDummyBody("bodyA", is_fixed, &tree);
  tree.compile();
  ASSERT_EQ(bodyA.get_num_collision_elements(), 0);

  // State 1a
  tree.addCollisionElement(element, bodyA, "");
  tree.compile();
  EXPECT_EQ(bodyA.get_num_collision_elements(), 1);
  Element& elementA = **bodyA.collision_elements_begin();
  EXPECT_EQ(elementA.get_num_cliques(), 0);

  // State 1b
  tree.addCollisionElement(element, bodyA, "");
  tree.compile();
  EXPECT_EQ(bodyA.get_num_collision_elements(), 2);
  Element& e1 = **bodyA.collision_elements_begin();
  Element& e2 = **(bodyA.collision_elements_begin() + 1);
  EXPECT_EQ(e1.get_num_cliques(), 1);
  EXPECT_EQ(e2.get_num_cliques(), 1);
  EXPECT_EQ(e1.collision_cliques()[0], e2.collision_cliques()[0]);

  // State 1c
  int original_clique = e1.collision_cliques()[0];
  tree.addCollisionElement(element, bodyA, "");
  tree.compile();
  EXPECT_EQ(bodyA.get_num_collision_elements(), 3);
  for (int i = 0; i < 3; ++i) {
    Element& test_element = **(bodyA.collision_elements_begin() + i);
    EXPECT_EQ(test_element.get_num_cliques(), 1);
    EXPECT_EQ(test_element.collision_cliques()[0], original_clique);
  }
}
// Tests the cases where a collision element is added to a previously-compiled
// body that has an adjacent body with a compiled element. It accounts for
// states 2a-2b listed above.
GTEST_TEST(PostCompileCliqueRBT, AddElementToNeighborBody) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& adjacent_body =
      MakeDummyBody("adjacent_body", is_fixed, &tree);
  tree.addCollisionElement(element, adjacent_body,
                           "");
  RigidBody<double>& target_body =
      MakeDummyBody("target_body", is_fixed, &tree, &adjacent_body);
  tree.compile();
  ASSERT_EQ(target_body.get_num_collision_elements(), 0);
  ASSERT_EQ(adjacent_body.get_num_collision_elements(), 1);
  Element& adjacent_element = **adjacent_body.collision_elements_begin();
  ASSERT_EQ(adjacent_element.get_num_cliques(), 0);

  // State 2a
  tree.addCollisionElement(element, target_body, "");
  tree.compile();
  EXPECT_EQ(adjacent_body.get_num_collision_elements(), 1);
  EXPECT_EQ(adjacent_element.get_num_cliques(), 1);
  ASSERT_EQ(target_body.get_num_collision_elements(), 1);
  Element& target_element = **target_body.collision_elements_begin();
  ASSERT_EQ(target_element.get_num_cliques(), 1);
  ASSERT_EQ(target_element.collision_cliques()[0],
            adjacent_element.collision_cliques()[0]);

  // State 2b
  tree.addCollisionElement(element, target_body, "");
  tree.compile();
  EXPECT_EQ(adjacent_body.get_num_collision_elements(), 1);
  EXPECT_EQ(adjacent_element.get_num_cliques(), 1);
  ASSERT_EQ(target_body.get_num_collision_elements(), 2);
  Element& target_element_2 = **(target_body.collision_elements_begin() + 1);
  ASSERT_EQ(target_element.get_num_cliques(), 1);
  ASSERT_EQ(target_element.collision_cliques()[0],
            adjacent_element.collision_cliques()[0]);
  ASSERT_EQ(target_element_2.collision_cliques()[0],
            adjacent_element.collision_cliques()[0]);
  ASSERT_EQ(target_element.collision_cliques()[0],
            target_element_2.collision_cliques()[0]);
}

// Tests the case where collision elements are added to adjacent bodies.
// This test accounts for state 3a.
GTEST_TEST(PostCompileCliqueRBT, AddElementsToEmptyNeighborBodies) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& body1 = MakeDummyBody("body1", is_fixed, &tree);
  RigidBody<double>& body2 = MakeDummyBody("body2", is_fixed, &tree, &body1);
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 0);
  ASSERT_EQ(body2.get_num_collision_elements(), 0);

  // State 3a
  tree.addCollisionElement(element, body1, "");
  tree.addCollisionElement(element, body2, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 1);
  ASSERT_EQ(body2.get_num_collision_elements(), 1);
  Element& element1 = **body1.collision_elements_begin();
  Element& element2 = **body2.collision_elements_begin();
  ASSERT_EQ(element1.get_num_cliques(), 1);
  ASSERT_EQ(element2.get_num_cliques(), 1);
  ASSERT_EQ(element1.collision_cliques()[0], element2.collision_cliques()[0]);
}

// Tests the case where collision elements are added to adjacent bodies and
// one of them already has a compiled collision element. This test accounts for
// state 3b.
GTEST_TEST(PostCompileCliqueRBT, AddElementsToSingleNeighborBodies) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& body1 = MakeDummyBody("body1", is_fixed, &tree);
  tree.addCollisionElement(element, body1, "");
  RigidBody<double>& body2 = MakeDummyBody("body2", is_fixed, &tree, &body1);
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 1);
  ASSERT_EQ(body2.get_num_collision_elements(), 0);

  // State 3b
  tree.addCollisionElement(element, body1, "");
  tree.addCollisionElement(element, body2, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 2);
  ASSERT_EQ(body2.get_num_collision_elements(), 1);
  Element& element1a = **body1.collision_elements_begin();
  Element& element1b = **(body1.collision_elements_begin() + 1);
  Element& element2 = **body2.collision_elements_begin();
  ASSERT_EQ(element1a.get_num_cliques(), 1);
  ASSERT_EQ(element1b.get_num_cliques(), 1);
  ASSERT_EQ(element2.get_num_cliques(), 1);
  ASSERT_EQ(element1a.collision_cliques()[0], element2.collision_cliques()[0]);
  ASSERT_EQ(element1a.collision_cliques()[0], element1b.collision_cliques()[0]);
  ASSERT_EQ(element1b.collision_cliques()[0], element2.collision_cliques()[0]);
}

// Tests the case where collision elements are added to adjacent bodies and
// one of them already has two compiled collision element. This test accounts
// for state 3c.
GTEST_TEST(PostCompileCliqueRBT, AddElementsToMultiNeighborBodies) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& body1 = MakeDummyBody("body1", is_fixed, &tree);
  tree.addCollisionElement(element, body1, "");
  tree.addCollisionElement(element, body1, "");
  RigidBody<double>& body2 = MakeDummyBody("body2", is_fixed, &tree, &body1);
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 2);
  Element& element1a = **body1.collision_elements_begin();
  Element& element1b = **(body1.collision_elements_begin() + 1);
  ASSERT_EQ(body2.get_num_collision_elements(), 0);
  ASSERT_EQ(element1a.get_num_cliques(), 1);
  ASSERT_EQ(element1b.get_num_cliques(), 1);
  ASSERT_EQ(element1a.collision_cliques()[0], element1b.collision_cliques()[0]);

  // State 3c
  tree.addCollisionElement(element, body1, "");
  tree.addCollisionElement(element, body2, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 3);
  ASSERT_EQ(body2.get_num_collision_elements(), 1);
  Element& element2 = **body2.collision_elements_begin();
  Element& element1c = **(body1.collision_elements_begin() + 2);
  ASSERT_EQ(element1a.get_num_cliques(), 2);
  ASSERT_EQ(element1b.get_num_cliques(), 2);
  ASSERT_EQ(element1c.get_num_cliques(), 2);
  ASSERT_EQ(element2.get_num_cliques(), 1);
  int common_clique = element2.collision_cliques()[0];
  // These tests rely on the fact that clique ids only get larger and that the
  // cliques are stored in increasing order.
  for (auto test_element : {&element1a, &element1b, &element1c}) {
    ASSERT_NE(test_element->collision_cliques().front(), common_clique);
    ASSERT_EQ(test_element->collision_cliques().back(), common_clique);
  }
}

// Tests the case where collision elements are added to adjacent bodies and
// both bodies already have compiled collision elements. This test accounts
// for state 3d.
GTEST_TEST(PostCompileCliqueRBT, AddElementsToPopulatedNeighborBodies) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& body1 = MakeDummyBody("body1", is_fixed, &tree);
  tree.addCollisionElement(element, body1, "");
  RigidBody<double>& body2 = MakeDummyBody("body2", is_fixed, &tree, &body1);
  tree.addCollisionElement(element, body2, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 1);
  Element& element1a = **body1.collision_elements_begin();
  ASSERT_EQ(body2.get_num_collision_elements(), 1);
  Element& element2a = **body2.collision_elements_begin();
  ASSERT_EQ(element1a.get_num_cliques(), 1);
  ASSERT_EQ(element2a.get_num_cliques(), 1);
  ASSERT_EQ(element1a.collision_cliques()[0], element2a.collision_cliques()[0]);
  int original_clique = element1a.collision_cliques()[0];

  // State 3d
  tree.addCollisionElement(element, body1, "");
  tree.addCollisionElement(element, body2, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 2);
  ASSERT_EQ(body2.get_num_collision_elements(), 2);
  Element& element1b = **(body1.collision_elements_begin() + 1);
  Element& element2b = **(body2.collision_elements_begin() + 1);
  ASSERT_EQ(element1a.get_num_cliques(), 1);
  ASSERT_EQ(element1b.get_num_cliques(), 1);
  ASSERT_EQ(element2a.get_num_cliques(), 1);
  ASSERT_EQ(element2b.get_num_cliques(), 1);
  // These tests rely on the fact that clique ids only get larger and that the
  // cliques are stored in increasing order.
  for (auto test_element : {&element1a, &element1b, &element2a, &element2b}) {
    EXPECT_EQ(test_element->collision_cliques().front(), original_clique);
  }
}

// Tests the case where a new body is affixed to a compiled body. The first
// attachment is to a body without a collision element. The second is to a body
// *with* a compiled element. This accounts for states 4 & 5.
GTEST_TEST(PostCompileCliqueRBT, AddNeighorToEmptyBody) {
  RigidBodyTreed tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& body1 = MakeDummyBody("body1", is_fixed, &tree);
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 0);

  // State 4.
  RigidBody<double>& body2 = MakeDummyBody("body2", is_fixed, &tree, &body1);
  tree.addCollisionElement(element, body2, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 0);
  ASSERT_EQ(body2.get_num_collision_elements(), 1);
  Element& element2 = **body2.collision_elements_begin();
  ASSERT_EQ(element2.get_num_cliques(), 0);

  // State 5.
  RigidBody<double>& body3 = MakeDummyBody("body3", is_fixed, &tree, &body2);
  tree.addCollisionElement(element, body3, "");
  tree.compile();
  ASSERT_EQ(body1.get_num_collision_elements(), 0);
  ASSERT_EQ(body2.get_num_collision_elements(), 1);
  ASSERT_EQ(body3.get_num_collision_elements(), 1);
  Element& element3 = **body3.collision_elements_begin();
  ASSERT_EQ(element2.get_num_cliques(), 1);
  ASSERT_EQ(element3.get_num_cliques(), 1);
  ASSERT_EQ(element3.collision_cliques()[0], element2.collision_cliques()[0]);
}

// Add a chain of two bodies: world -> bodyA -> bodyB and a third, independent
// bodyC. Each body has a collision sphere and are positioned to be in contact.
// However, no contact is reported between bodyA and body B because adjacent
// bodies' collision elements are placed into common cliques. So, it reports
// two collisions. Adding a new geometry to bodyB should produce a single
// new contact with bodyC.
GTEST_TEST(CollisionFilterGroupRBT, AddedElementInheritsCliques) {
  RigidBodyTree<double> tree;
  const bool is_fixed = false;
  // Collision element description -- copied into every body.
  Element element(DrakeShapes::Sphere(1.0));

  RigidBody<double>& bodyA = MakeDummyBody("bodyA", is_fixed, &tree);
  tree.addCollisionElement(element, bodyA, "");
  RigidBody<double>& bodyB = MakeDummyBody("bodyB", is_fixed, &tree, &bodyA);
  tree.addCollisionElement(element, bodyB, "");
  RigidBody<double>& bodyC = MakeDummyBody("bodyC", is_fixed, &tree);
  tree.addCollisionElement(element, bodyC, "");

  tree.compile();
  std::vector<PointPair<double>> pairs = CollideAtZero(&tree);
  EXPECT_EQ(pairs.size(), 2u);
  auto expect_pair = [&bodyA, &bodyB, &bodyC](const PointPair<double>& pair) {
    const RigidBody<double>* body1 = pair.elementA->get_body();
    const RigidBody<double>* body2 = pair.elementB->get_body();
    if (body1 == &bodyA || body1 == &bodyB) {
      EXPECT_EQ(body2, &bodyC) << "Bodies A and B should only collide with C";
    } else if (body1 == &bodyC) {
      EXPECT_TRUE(body2 == &bodyA || body2 == &bodyB);
    } else {
      GTEST_FAIL() << "Collision included an unknown body: "
                   << body1->get_name() << " and " << body2->get_name();
    }
  };
  expect_pair(pairs[0]);
  expect_pair(pairs[1]);

  // Now add a new collision element to B.
  tree.addCollisionElement(element, bodyB, "");
  tree.compile();
  pairs = CollideAtZero(&tree);
  ASSERT_EQ(pairs.size(), 3u);
  expect_pair(pairs[0]);
  expect_pair(pairs[1]);
  expect_pair(pairs[2]);
}
//---------------------------------------------------------------------------

// Utility function for loading an sdf|urdf file, performing *optional*
// pre-compile activities, configuring it to the "zero" configuration, perform
// collision detection, and assert the expected number of collisions.
//
// This supports tests where the file is parsed and compiled as well as tests
// where compilation is deferred.
//
// Note: The pre_compile function does *not* need to compile the tree.
void ExpectNCollisions(
    const std::string& file_name, const std::string& extension,
    int collision_count,
    std::function<void(RigidBodyTree<double>*)> pre_compile = {}) {
  // If there is pre-compile work to do, parse without compilation.
  bool do_compile = !pre_compile;
  RigidBodyTree<double> tree;
  if (extension == "urdf") {
    AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow(file_name + extension),
        drake::multibody::joints::kRollPitchYaw, do_compile, &tree);
  } else if (extension == "sdf") {
    AddModelInstancesFromSdfFileToWorld(
        FindResourceOrThrow(file_name + extension),
        drake::multibody::joints::kRollPitchYaw, do_compile, &tree);
  }  else {
    GTEST_FAIL() << "Unexpected model extension: " << extension;
  }

  if (pre_compile) {
    pre_compile(&tree);
    tree.compile();
  }

  std::vector<PointPair<double>> pairs = CollideAtZero(&tree);
  EXPECT_EQ(pairs.size(), collision_count)
            << "Failure for " << file_name << extension;
}

// These tests read the files `filter_groups_in_file.[sdf|urdf]`. Each file
// represents an identical tree with built-in collisions where all collisions
// have been filtered out by different filter specification idioms. The
// test confirms that no collisions are reported.
GTEST_TEST(CollisionFilterGroupDefinition, TestCollisionFilterGroupSpecUrdf) {
  const std::string file_name = "drake/multibody/collision/test/"
      "filter_groups_in_file.";
  ExpectNCollisions(file_name, "urdf", 0);
}

GTEST_TEST(CollisionFilterGroupDefinition, TestCollisionFilterGroupSpecSdf) {
  const std::string file_name = "drake/multibody/collision/test/"
      "filter_groups_in_file.";
  ExpectNCollisions(file_name, "sdf", 0);
}

// This confirms that default use of parsing a urdf/sdf leaves the tree in a
// state that prevents modification of collision filter groups for the parsed
// bodies.
GTEST_TEST(CollisionFilterGroupDefinition, PostParseFailure) {
  const std::string file_name = "drake/multibody/collision/test/"
      "no_filter_groups.";
  // Confirm collisions without filter groups.
  ExpectNCollisions(file_name, "urdf", 1);
  ExpectNCollisions(file_name, "sdf", 1);

  auto configure_groups = [](RigidBodyTree<double>* tree) {
    const std::string group_name = "self_group";
    tree->DefineCollisionFilterGroup(group_name);
    tree->AddCollisionFilterGroupMember(group_name, "sphereA", 0);
    tree->AddCollisionFilterGroupMember(group_name, "sphereB", 0);
    tree->AddCollisionFilterIgnoreTarget(group_name, group_name);
  };
  ExpectNCollisions(file_name, "urdf", 0, configure_groups);
  ExpectNCollisions(file_name, "sdf", 0, configure_groups);
}

GTEST_TEST(CollisionFilterGroupDefinition, MultiModelSdfFilters) {
  const std::string file_name = "drake/multibody/collision/test/"
      "multi_model_groups.";
  ExpectNCollisions(file_name, "sdf", 0);
}

}  // namespace
}  // namespace test
}  // namespace collision
}  // namespace multibody
}  // namespace drake
