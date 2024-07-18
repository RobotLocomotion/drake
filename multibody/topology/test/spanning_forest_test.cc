/* clang-format off to disable clang-format-includes */
#include "drake/multibody/topology/forest.h"
/* clang-format on */

#include <string>
#include <tuple>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

// Tests here are those that require definition of SpanningForest, including
// tests of LinkJointGraph interactions with its SpanningForest. See
// link_joint_graph_test.cc for tests that need only LinkJointGraph
// definitions.

namespace drake {
namespace multibody {
namespace internal {

using std::pair;

namespace {

// A default-constructed LinkJointGraph contains a predefined World
// Link, and can generate a valid SpanningForest containing just a World
// Mobod. The LinkJointGraph should be properly updated to have
// a single LinkComposite and proper modeling info.
GTEST_TEST(SpanningForest, WorldOnlyTest) {
  LinkJointGraph graph;
  const SpanningForest& forest = graph.forest();

  // Check that the internal forest is wired correctly but not valid yet.
  EXPECT_EQ(&forest.graph(), &graph);
  EXPECT_FALSE(graph.forest_is_valid());
  EXPECT_FALSE(forest.is_valid());
  EXPECT_EQ(forest.height(), 0);
  EXPECT_TRUE(forest.mobods().empty());
  EXPECT_TRUE(forest.trees().empty());
  EXPECT_TRUE(forest.loop_constraints().empty());

  // The default-constructed graph should contain only World.
  EXPECT_EQ(ssize(graph.links()), 1);
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(graph.world_link().name(), "world");
  EXPECT_EQ(graph.world_link().model_instance(), world_model_instance());
  const BodyIndex world_link_index = graph.world_link().index();
  EXPECT_EQ(world_link_index, BodyIndex(0));
  const LinkOrdinal world_link_ordinal = graph.world_link().ordinal();
  EXPECT_EQ(world_link_ordinal, LinkOrdinal(0));

  // Now build a forest representing the World-only graph.
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_TRUE(graph.forest_is_valid());
  EXPECT_TRUE(forest.is_valid());
  EXPECT_EQ(&forest.graph(), &graph);
  const SpanningForest::Mobod& world = forest.mobods(MobodIndex(0));
  EXPECT_EQ(&world, &forest.world_mobod());
  const MobodIndex world_mobod_index = forest.world_mobod().index();
  EXPECT_EQ(world_mobod_index, MobodIndex(0));
  EXPECT_EQ(graph.link_to_mobod(world_link_index), MobodIndex(0));
  EXPECT_EQ(ssize(graph.link_composites()), 1);
  EXPECT_EQ(ssize(graph.link_composites(LinkCompositeIndex(0))), 1);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[0], world_link_index);

  // Check that the World-only forest makes sense.
  EXPECT_EQ(ssize(forest.mobods()), 1);
  EXPECT_TRUE(forest.trees().empty());  // World isn't part of a tree.
  EXPECT_TRUE(forest.loop_constraints().empty());
  EXPECT_EQ(forest.height(), 1);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);
  EXPECT_EQ(forest.welded_mobods()[0][0], world_mobod_index);
  EXPECT_EQ(forest.mobod_to_link_ordinal(world_mobod_index),
            world_link_ordinal);
  EXPECT_EQ(ssize(forest.mobod_to_link_ordinals(world_mobod_index)), 1);
  EXPECT_EQ(forest.mobod_to_link_ordinals(world_mobod_index)[0],
            world_link_ordinal);
  EXPECT_EQ(forest.num_positions(), 0);
  EXPECT_EQ(forest.num_velocities(), 0);
  EXPECT_TRUE(forest.quaternion_starts().empty());

  // Exercise the Mobod API to check the World Mobod for reasonableness.
  EXPECT_TRUE(world.is_world());
  EXPECT_FALSE(world.is_base_body());
  EXPECT_TRUE(world.is_anchored());
  EXPECT_TRUE(world.is_leaf_mobod());
  EXPECT_FALSE(world.is_reversed());  // Not meaningful though.
  EXPECT_TRUE(world.is_weld());       // Defined as having no inboard dofs.
  EXPECT_FALSE(world.inboard().is_valid());
  EXPECT_TRUE(world.outboards().empty());
  EXPECT_EQ(world.link_ordinal(), world_link_ordinal);
  EXPECT_EQ(ssize(world.follower_link_ordinals()), 1);
  EXPECT_EQ(world.follower_link_ordinals()[0], world_link_ordinal);
  EXPECT_FALSE(world.joint_ordinal().is_valid());
  EXPECT_FALSE(world.tree().is_valid());
  EXPECT_EQ(world.welded_mobods_group(), WeldedMobodsIndex(0));
  EXPECT_EQ(world.level(), 0);
  EXPECT_EQ(world.q_start(), 0);
  EXPECT_EQ(world.nq(), 0);
  EXPECT_EQ(world.v_start(), 0);
  EXPECT_EQ(world.nv(), 0);
  EXPECT_EQ(world.nq_inboard(), 0);
  EXPECT_EQ(world.nv_inboard(), 0);
  EXPECT_EQ(world.nq_outboard(), 0);
  EXPECT_EQ(world.nv_outboard(), 0);
  EXPECT_FALSE(world.has_quaternion());
  EXPECT_EQ(world.num_subtree_mobods(), 1);
  EXPECT_EQ(world.subtree_velocities(), (std::pair{0, 0}));
  EXPECT_EQ(world.outboard_velocities(), (std::pair{0, 0}));

  // Check that if we clear the graph, the forest returns to its invalid
  // condition as above.
  graph.Clear();
  EXPECT_EQ(&forest.graph(), &graph);  // Still wired right.
  EXPECT_FALSE(graph.forest_is_valid());
  EXPECT_FALSE(forest.is_valid());
  EXPECT_EQ(forest.height(), 0);
  EXPECT_TRUE(forest.mobods().empty());
  EXPECT_TRUE(forest.trees().empty());
  EXPECT_TRUE(forest.loop_constraints().empty());
}

GTEST_TEST(SpanningForest, TreeAndLoopConstraintAPIs) {
  LinkJointGraph graph;
  const SpanningForest& forest = graph.forest();
  EXPECT_TRUE(graph.BuildForest());

  // This stub exists solely to enable these API tests until the implementing
  // code is merged. Here's the forest we're expecting:
  //            -> mobod1 => mobod2
  //     World                 ^
  //            -> mobod3 .....|  loop constraint
  // There are two 1-dof "->" joints and one 0-dof "=>" weld.
  // TODO(sherm1) Make the same forest legitimately.
  const_cast<SpanningForest&>(forest).AddStubTreeAndLoopConstraint();

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(ssize(forest.mobods()), 4);
  EXPECT_EQ(ssize(forest.loop_constraints()), 1);

  // Not much to check for the loop constraint.
  const auto& loop_constraint = forest.loop_constraints(LoopConstraintIndex(0));
  EXPECT_EQ(loop_constraint.index(), LoopConstraintIndex(0));
  EXPECT_EQ(loop_constraint.primary_mobod(), MobodIndex(3));
  EXPECT_EQ(loop_constraint.shadow_mobod(), MobodIndex(2));

  const SpanningForest::Tree& tree0 = forest.trees(TreeIndex(0));
  EXPECT_EQ(tree0.index(), TreeIndex(0));
  EXPECT_EQ(tree0.height(), 2);
  EXPECT_EQ(tree0.base_mobod(), MobodIndex(1));
  EXPECT_EQ(tree0.last_mobod(), MobodIndex(2));
  EXPECT_EQ(tree0.begin()->index(), MobodIndex(1));
  EXPECT_EQ(tree0.end() - tree0.begin(), 2);
  EXPECT_EQ(tree0.front().index(), MobodIndex(1));
  EXPECT_EQ(tree0.back().index(), MobodIndex(2));
  EXPECT_EQ(tree0.num_mobods(), 2);
  EXPECT_EQ(tree0.q_start(), 0);
  EXPECT_EQ(tree0.v_start(), 0);
  EXPECT_EQ(tree0.nq(), 1);
  EXPECT_EQ(tree0.nv(), 1);

  const SpanningForest::Tree& tree1 = forest.trees(TreeIndex(1));
  EXPECT_EQ(tree1.index(), TreeIndex(1));
  EXPECT_EQ(tree1.height(), 1);
  EXPECT_EQ(tree1.base_mobod(), MobodIndex(3));
  EXPECT_EQ(tree1.last_mobod(), MobodIndex(3));
  EXPECT_EQ(tree1.begin()->index(), MobodIndex(3));
  EXPECT_EQ(tree1.end() - tree1.begin(), 1);
  EXPECT_EQ(tree1.front().index(), MobodIndex(3));
  EXPECT_EQ(tree1.back().index(), MobodIndex(3));
  EXPECT_EQ(tree1.num_mobods(), 1);
  EXPECT_EQ(tree1.q_start(), 1);
  EXPECT_EQ(tree1.v_start(), 1);
  EXPECT_EQ(tree1.nq(), 1);
  EXPECT_EQ(tree1.nv(), 1);
}

/* Creates a straightforward graph of two trees each with multiple branches,
plus a lone unattached free link. There are no welds or reverse joints or loops.
We intentionally jumble the link numbering to make sure we don't get the right
forest numbering by luck. We'll use this to test that basic numbering and
reporting works. We'll reuse this in several tests with different options.

Here is the original graph and its expected forest model:

              Links                                Mobods

                             15                               13
                       13  14                           11  12
           9  10         11                     5  6      10
       5    6            8   7     -->      3    4        9   14
          4                3                   2            8
          1       2       12                   1            7      15

          ..........0..........                ...........0..........

Note the depth-first ordering in the Forest. We won't provide joints to World
but the modeler's base body policy should choose Links 1, 2, and 12 and add
appropriate joints. Also note our policy of moving all singleton free
links to the highest-numbered mobods.

Note also that the left link tree maps to forest tree0, the right link tree
maps to forest tree1, and the free link is forest tree2. */
LinkJointGraph MakeMultiBranchGraph(ModelInstanceIndex left,
                                    ModelInstanceIndex right,
                                    ModelInstanceIndex free_link) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);

  // Map links to their model instances.
  std::map<int, ModelInstanceIndex> link_to_instance{
      {1, left},      {4, left},   {5, left},   {6, left},
      {9, left},      {10, left},  //
      {2, free_link},              //
      {12, right},    {3, right},  {8, right},  {7, right},
      {11, right},    {13, right}, {14, right}, {15, right}};

  // Define the graph. Links:
  for (int i = 1; i <= 15; ++i)
    graph.AddLink("link" + std::to_string(i), link_to_instance[i]);

  // Joints: (Check against left-hand diagram above.)
  const std::vector<std::pair<int, int>> joints{
      {1, 4},  {4, 5}, {4, 6}, {6, 9},  {6, 10},                        // left
      {12, 3}, {3, 8}, {3, 7}, {8, 11}, {11, 13}, {11, 14}, {14, 15}};  // right
  for (int i = 0; i < ssize(joints); ++i) {
    const auto joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), link_to_instance[joint.second],
                   "revolute", BodyIndex(joint.first), BodyIndex(joint.second));
  }

  // Remove and re-add a joint to mess up the numbering. It will now have
  // JointIndex 12, and 7 should be unused.
  EXPECT_TRUE(graph.has_joint(JointIndex(7)));
  EXPECT_TRUE(graph.HasJointNamed("joint7", link_to_instance[7]));
  graph.RemoveJoint(JointIndex(7));  // The joint between 3 & 8.
  graph.AddJoint("joint7replaced", link_to_instance[7], "revolute",
                 BodyIndex(3), BodyIndex(7));
  EXPECT_FALSE(graph.has_joint(JointIndex(7)));
  EXPECT_FALSE(graph.HasJointNamed("joint7", link_to_instance[7]));
  EXPECT_TRUE(graph.has_joint(JointIndex(12)));
  EXPECT_TRUE(graph.HasJointNamed("joint7replaced", link_to_instance[7]));

  // Make sure we got the graph we're expecting. Before building the forest
  // the only links and joints are the user-added ones.
  EXPECT_EQ(ssize(graph.links()), 16);  // includes World
  EXPECT_EQ(ssize(graph.joints()), 12);
  EXPECT_EQ(graph.num_user_links(), 16);
  EXPECT_EQ(graph.num_user_joints(), 12);
  return graph;
}

/* Build the above graph with default options and then dig into the result
in detail to make sure all the relevant APIs work. Also verify base link
choice policy, and free link placement at the end. */
GTEST_TEST(SpanningForest, MultipleBranchesDefaultOptions) {
  const ModelInstanceIndex left_instance(5), right_instance(6),
      free_link_instance(7);  // arbitrary
  LinkJointGraph graph =
      MakeMultiBranchGraph(left_instance, right_instance, free_link_instance);
  const SpanningForest& forest = graph.forest();

  // Some basic sanity checks for the forest object.
  EXPECT_EQ(&forest.graph(), &graph);  // check backpointer
  EXPECT_EQ(&forest.links(), &graph.links());
  EXPECT_EQ(&forest.joints(), &graph.joints());

  // Build with default options.
  EXPECT_TRUE(graph.BuildForest());

  EXPECT_EQ(forest.options(), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(left_instance), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(right_instance), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(free_link_instance),
            ForestBuildingOptions::kDefault);

  // See that the graph got augmented properly to reflect the as-built forest.
  EXPECT_EQ(graph.num_user_links(), 16);
  EXPECT_EQ(ssize(graph.links()), 16);  // no links added
  EXPECT_EQ(graph.num_user_joints(), 12);
  EXPECT_EQ(ssize(graph.joints()), 15);  // modeling adds 3 floating joints

  // The only LinkComposite is the World composite and it is alone there.
  EXPECT_EQ(ssize(graph.link_composites()), 1);  // just World
  EXPECT_EQ(ssize(graph.link_composites(LinkCompositeIndex(0))), 1);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[0],
            graph.world_link().index());

  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(ssize(forest.mobods()), 16);        // includes World
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World
  EXPECT_EQ(forest.num_positions(), 33);   // 12 revolute, 3 x 7 quat floating
  EXPECT_EQ(forest.num_velocities(), 30);  // 12 revolute, 3 x 6 floating

  // Verify that we're tracking quaternions properly.
  EXPECT_EQ(ssize(forest.quaternion_starts()), 3);  // One per floating joint.
  EXPECT_EQ(forest.quaternion_starts(), (std::vector{0, 12, 26}));
  EXPECT_FALSE(forest.mobods(MobodIndex(10)).has_quaternion());  // Arbitrary.
  EXPECT_TRUE(forest.mobods(MobodIndex(1)).has_quaternion());
  EXPECT_TRUE(forest.mobods(MobodIndex(7)).has_quaternion());
  EXPECT_TRUE(forest.mobods(MobodIndex(15)).has_quaternion());

  // Check Mobod->Link and Link->Mobod mappings.
  const std::vector<pair<int, int>> mobod_link_map{
      {0, 0}, {1, 1}, {2, 4},   {3, 5},   {4, 6},   {5, 9},   {6, 10}, {7, 12},
      {8, 3}, {9, 8}, {10, 11}, {11, 13}, {12, 14}, {13, 15}, {14, 7}, {15, 2}};
  for (auto mobod_link : mobod_link_map) {
    EXPECT_EQ(graph.link_to_mobod(BodyIndex(mobod_link.second)),
              MobodIndex(mobod_link.first));
    EXPECT_EQ(forest.mobod_to_link_ordinal(MobodIndex(mobod_link.first)),
              LinkOrdinal(mobod_link.second));
    // Each Mobod has only a single Link that follows it.
    EXPECT_EQ(
        ssize(forest.mobod_to_link_ordinals(MobodIndex(mobod_link.first))), 1);
    EXPECT_EQ(forest.mobod_to_link_ordinals(MobodIndex(mobod_link.first))[0],
              LinkOrdinal(mobod_link.second));
  }
  EXPECT_EQ(forest.height(), 7);

  // Check that the three Trees in the forest make sense.
  // Reminder: left->tree0, right->tree1, free link->tree2.

  auto tree_check = [&forest](int index, int height, int base_mobod,
                              int last_mobod, int num_mobods, int q_start,
                              int nq, int v_start, int nv) {
    const SpanningForest::Tree& tree = forest.trees(TreeIndex(index));
    EXPECT_EQ(tree.index(), TreeIndex(index));
    EXPECT_EQ(tree.height(), height);
    EXPECT_EQ(tree.base_mobod(), MobodIndex(base_mobod));
    EXPECT_EQ(tree.last_mobod(), MobodIndex(last_mobod));
    EXPECT_EQ(tree.num_mobods(), num_mobods);
    EXPECT_EQ(tree.q_start(), q_start);
    EXPECT_EQ(tree.nq(), nq);
    EXPECT_EQ(tree.v_start(), v_start);
    EXPECT_EQ(tree.nv(), nv);
    EXPECT_EQ(&tree.front(), &forest.mobods(MobodIndex(base_mobod)));
    EXPECT_EQ(&tree.back(), &forest.mobods(MobodIndex(last_mobod)));
    EXPECT_EQ(tree.begin(), &tree.front());
    EXPECT_EQ(tree.end(), &tree.back() + 1);
  };

  // clang-format off
  //         index  height  base_ last_ num_mobods  qstart nq  vstart nv
  tree_check(0,        4,     1,    6,      6,         0,  12,    0,  11);
  tree_check(1,        6,     7,   14,      8,        12,  14,   11,  13);
  tree_check(2,        1,    15,   15,      1,        26,   7,   24,   6);
  // clang-format on

  // Sample some q's and v's to see if they can find their tree and mobod.
  EXPECT_EQ(forest.q_to_tree(9), TreeIndex(0));
  EXPECT_EQ(forest.q_to_tree(19), TreeIndex(1));
  EXPECT_EQ(forest.q_to_tree(30), TreeIndex(2));
  EXPECT_EQ(forest.v_to_tree(9), TreeIndex(0));
  EXPECT_EQ(forest.v_to_tree(19), TreeIndex(1));
  EXPECT_EQ(forest.v_to_tree(29), TreeIndex(2));
  EXPECT_EQ(forest.q_to_mobod(9), MobodIndex(4));
  EXPECT_EQ(forest.q_to_mobod(20), MobodIndex(9));
  EXPECT_EQ(forest.q_to_mobod(30), MobodIndex(15));
  EXPECT_EQ(forest.v_to_mobod(9), MobodIndex(5));
  EXPECT_EQ(forest.v_to_mobod(20), MobodIndex(11));
  EXPECT_EQ(forest.v_to_mobod(29), MobodIndex(15));

  // mobod.subtree_velocities() reports which dofs are in the subtree
  // rooted at a given Mobod as (start, ndofs). Check some of them.
  auto find_subv = [&forest](int mobod_index) -> pair<int, int> {
    return forest.mobods(MobodIndex(mobod_index)).subtree_velocities();
  };
  EXPECT_EQ(find_subv(0), pair(0, 30));   // World
  EXPECT_EQ(find_subv(1), pair(0, 11));   // base tree0
  EXPECT_EQ(find_subv(7), pair(11, 13));  // base tree1
  EXPECT_EQ(find_subv(15), pair(24, 6));  // base tree2
  EXPECT_EQ(find_subv(3), pair(7, 1));    // terminal
  EXPECT_EQ(find_subv(4), pair(8, 3));    // nonterminal
  EXPECT_EQ(find_subv(8), pair(17, 7));   // tree1 nonterminal
  EXPECT_EQ(find_subv(10), pair(19, 4));
  EXPECT_EQ(find_subv(14), pair(23, 1));  // tree1 terminal

  // mobod.outboard_velocities() is the same but excludes inboard dofs of
  // the selected Mobod.
  auto find_outv = [&forest](int mobod_index) -> pair<int, int> {
    return forest.mobods(MobodIndex(mobod_index)).outboard_velocities();
  };
  EXPECT_EQ(find_outv(0), pair(0, 30));   // Same Mobods as above
  EXPECT_EQ(find_outv(1), pair(6, 5));    // base tree0
  EXPECT_EQ(find_outv(7), pair(17, 7));   // base tree1
  EXPECT_EQ(find_outv(15), pair(30, 0));  // base tree2
  EXPECT_EQ(find_outv(3), pair(8, 0));    // terminal
  EXPECT_EQ(find_outv(4), pair(9, 2));    // nonterminal
  EXPECT_EQ(find_outv(8), pair(18, 6));   // tree1 nonterminal
  EXPECT_EQ(find_outv(10), pair(20, 3));
  EXPECT_EQ(find_outv(14), pair(24, 0));  // tree1 terminal
}

/* Starting with the same graph as the previous test, change the tree1
options to use an RpyFloating base and tree2 to use a fixed (Weld) base. */
GTEST_TEST(SpanningForest, MultipleBranchesBaseJointOptions) {
  const ModelInstanceIndex left_instance(5), right_instance(6),
      free_link_instance(7);  // arbitrary
  LinkJointGraph graph =
      MakeMultiBranchGraph(left_instance, right_instance, free_link_instance);
  const SpanningForest& forest = graph.forest();

  graph.SetForestBuildingOptions(left_instance,
                                 ForestBuildingOptions::kUseRpyFloatingJoints);
  graph.SetForestBuildingOptions(right_instance,
                                 ForestBuildingOptions::kUseFixedBase);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_EQ(forest.options(), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(left_instance),
            ForestBuildingOptions::kUseRpyFloatingJoints);
  EXPECT_EQ(forest.options(right_instance),
            ForestBuildingOptions::kUseFixedBase);
  EXPECT_EQ(forest.options(free_link_instance),
            ForestBuildingOptions::kDefault);

  // Check graph augmentation.
  EXPECT_EQ(graph.num_user_links(), 16);
  EXPECT_EQ(ssize(graph.links()), 16);  // no links added
  EXPECT_EQ(graph.num_user_joints(), 12);
  EXPECT_EQ(ssize(graph.joints()), 15);  // modeling adds 3 joints to World

  // We should have 12 user-added revolute joints, plus one RpyFloating, one
  // Weld, and one QuaternionFloating joint added.
  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(ssize(forest.mobods()), 16);        // includes World
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World
  EXPECT_EQ(forest.num_positions(), 25);        // 12 + 6 + 0 + 7
  EXPECT_EQ(forest.num_velocities(), 24);       // 12 + 6 + 0 + 6

  // Reminder: left->tree0, right->tree1, free link->tree2.
  const SpanningForest::Tree& tree0 = forest.trees(TreeIndex(0));
  const SpanningForest::Tree& tree1 = forest.trees(TreeIndex(1));
  const SpanningForest::Tree& tree2 = forest.trees(TreeIndex(2));

  // The base Mobod of tree2 is now welded to World so is anchored. (A tree's
  // "front" is the base Mobod for that tree.) The corresponding Links should
  // also know their anchored status.
  EXPECT_FALSE(tree0.front().is_anchored());
  EXPECT_TRUE(tree1.front().is_anchored());
  EXPECT_FALSE(tree2.front().is_anchored());

  EXPECT_FALSE(graph.links(tree0.front().link_ordinal()).is_anchored());
  EXPECT_TRUE(graph.links(tree1.front().link_ordinal()).is_anchored());
  EXPECT_FALSE(graph.links(tree2.front().link_ordinal()).is_anchored());

  // There is only the World composite, but now tree1's base link is included.
  EXPECT_EQ(ssize(graph.link_composites()), 1);  // just World
  EXPECT_EQ(ssize(graph.link_composites(LinkCompositeIndex(0))), 2);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[0],
            graph.world_link().index());
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[1],
            graph.links(tree1.front().link_ordinal()).index());

  // Similarly, there is only one WeldedMobods group, containing just World
  // and tree1's base
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);
  EXPECT_EQ(ssize(forest.welded_mobods(WeldedMobodsIndex(0))), 2);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0))[0],
            forest.world_mobod().index());
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0))[1], tree1.base_mobod());
}

/* Verify that our base body choice policy works as intended. The policy
applies to any disconnected subgraph with at least one joint:
  1 pick the link that appears as a child least often
  2 to break a tie, pick the one that appears as a parent _most_ often
  3 if there is still a tie, pick the lowest-numbered link.

We'll also verify that parent->child direction is preserved as inboard->outboard
when possible, but reversed otherwise.

To test we use these subgraphs, with no connections to World. The link that
should be chosen for the base body is marked with an asterisk:
  -> gives joint's parent->child direction

           {Links}
      {1} <- {2*} -> {3}              rule 1
      {4} -> {5} <- {6*} -> {7}       rule 2 ({4}, {6} tied by rule 1)
      {8*} -> {9} <- {10}             rule 3 ({8}, {10} tied by rules 1 & 2)

After depth-first numbering, the corresponding trees in the forest should look
like this. Mobod numbers are shown, with the link in {}.

                   6{4}          10{10}
     2{1}   3{3}   5{5}   7{7}    9{9}
        1{2}          4{6}        8{8}
     ..............0{0}...............

Joints {4,5} and {10,9} have to be reversed to make the trees.
*/
GTEST_TEST(SpanningForest, BaseBodyChoicePolicy) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  for (int i = 1; i <= 10; ++i)
    graph.AddLink("link" + std::to_string(i), default_model_instance());
  const std::vector<std::pair<int, int>> joints{{2, 1}, {2, 3}, {4, 5}, {6, 5},
                                                {6, 7}, {8, 9}, {10, 9}};
  for (int i = 0; i < ssize(joints); ++i) {
    const auto joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), default_model_instance(),
                   "revolute", BodyIndex(joint.first), BodyIndex(joint.second));
  }

  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_EQ(graph.num_user_joints(), 7);
  EXPECT_EQ(ssize(graph.joints()), 10);  // 3 ephemeral base joints

  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(forest.trees(TreeIndex(0)).base_mobod(), MobodIndex(1));
  EXPECT_EQ(forest.trees(TreeIndex(1)).base_mobod(), MobodIndex(4));
  EXPECT_EQ(forest.trees(TreeIndex(2)).base_mobod(), MobodIndex(8));

  // Check that only joints {4,5} and {10,9} were reversed. (The last three
  // entries are the added base joints.)
  const std::vector<bool> expect_reversed{false, false, true,  false, false,
                                          false, true,  false, false, false};
  for (auto joint : graph.joints()) {
    const auto& mobod = forest.mobods(joint.mobod_index());
    EXPECT_EQ(mobod.is_reversed(), expect_reversed[joint.index()]);
  }
}

/* Verify that we can build a good forest for a serial chain, some static and
floating links, and some simple composites, obeying forest building options.

Links can become static either by being members of a static model instance,
or by having been specified with the kStatic link flag; we test both of those
here. Forest building should add a weld joint to World for every static link
unless there is already an explicit weld; we'll check that.

LinkComposites are always computed and consist of subgraphs of links that are
mutually welded together (directly or indirectly). Depending on forest building
options, we may use a single Mobod for a LinkComposite, or we may use a Mobod
for each of those welded-together links. In the latter case we also compute
"welded Mobod" groups consisting of Mobods that are mutually interconnected by
weld mobilizers (directly or indirectly). When we're modeling each
LinkComposite with just one Mobod, there won't be any welded-together Mobods.
By convention, there will still be one welded Mobod group, consisting just
of the World Mobod.

We'll also check that coordinates are assigned properly and that pre-calculated
forest counts are correct.

The LinkJointGraph
------------------
  -> user supplied rotational joint
  => user supplied weld joint
  (arrow indicates given parent->child ordering)

  world->link1->link2->link3->link4->link5
      static6         (no joint, static model instance)
      =>static7       (weld provided, static model instance)
      static8         (no joint, static link)
      free9           (no joint)
      link10=>base11* (free floating but welded together)

    * link10 would be the preferred base link but link 11 "base11" is marked
      "must be base link" so we have to use a reversed mobilizer there

SpanningForest 1 (don't combine LinkComposites)
-----------------------------------------------
  ≡> added weld joint       [mobods]
  6> added floating joint   {links}

  When building the forest in the mode where every Link gets its own Mobod
  (even if Links are welded together) the SpanningForest should have 6 trees of
  Mobods (World is not in a tree):

  tree      world [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6]    =>static7
     2 [7]    ≡>static6        (added weld)
     3 [8]    ≡>static8        (added weld)
     4 [9-10] 6>base11=>link10 (added 6dof, reversed the user's weld)
     5 [11]   6>free9          (added 6dof, free bodies are always last)

  LinkComposites:  {0 7 6 8} {11 10}
  Welded Mobods groups: [0 6 7 8] [9 10]

  The particular ordering results from (a) user-supplied Joints get processed
  before added ones, and (b) static model instance Links get welded prior
  to individually-specified static Links in non-static model instances.
  However, note that we do not promise any particular ordering other than
  (1) World is always present and is first, and (2) the active link comes first
  in any LinkComposite.

TODO(sherm1) Retest with "combine composites" option on (currently stubbed).
*/
GTEST_TEST(SpanningForest, SerialChainAndMore) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  EXPECT_EQ(ssize(graph.joint_traits()), 4);  // Built-ins plus "revolute".
  EXPECT_TRUE(graph.IsJointTypeRegistered("revolute"));
  EXPECT_FALSE(graph.IsJointTypeRegistered("prismatic"));

  // We'll add the chain and other moving elements to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Put static bodies in this model instance.
  const ModelInstanceIndex static_model_instance(100);

  BodyIndex parent = graph.AddLink("link1", model_instance);
  graph.AddJoint("pin1", model_instance, "revolute", world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    BodyIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, "revolute",
                   parent, child);
    parent = child;
  }

  graph.AddLink("static6", static_model_instance);
  const BodyIndex static7_index =
      graph.AddLink("static7", static_model_instance);
  const BodyIndex static8_index =
      graph.AddLink("static8", model_instance, LinkFlags::kStatic);
  // Manually adding a weld to World is allowable for a static Link.
  const JointIndex static7_joint_index =
      graph.AddJoint("static7_weld", model_instance, "weld",
                     graph.world_link().index(), static7_index);
  // Now add a free link and a free-floating pair.
  graph.AddLink("free9", model_instance);

  const BodyIndex link10_index = graph.AddLink("link10", model_instance);
  const BodyIndex base11_index =
      graph.AddLink("base11", model_instance, LinkFlags::kMustBeBaseBody);
  graph.AddJoint("weld", model_instance, "weld", link10_index, base11_index);

  // SpanningForest 1 (not combining welded Links onto one Mobod)
  // ------------------------------------------------------------
  graph.ResetForestBuildingOptions();  // Unnecessary; just being tidy.
  graph.SetForestBuildingOptions(static_model_instance,
                                 ForestBuildingOptions::kStatic);
  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_TRUE(graph.forest_is_valid());

  // Verify that ChangeJointType() rejects an attempt to change a static link's
  // weld to something articulated. Also check that when it fails, it doesn't
  // invalidate the currently-valid forest.

  // static7 is in static_model_instance.
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.ChangeJointType(static7_joint_index, "revolute"),
      "ChangeJointType.*can't change type.*static7_weld.*instance 5.*"
      "weld to revolute.*because.*connects static .*static7.*World.*");
  EXPECT_TRUE(graph.forest_is_valid());

  // static8 is explicitly flagged static but doesn't have a user-defined
  // joint. Verify that ChangeJointType() refuses to operate on an ephemeral
  // (added) joint.
  const JointIndex static8_joint_index =
      graph.link_by_index(static8_index).inboard_joint_index();
  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.ChangeJointType(static8_joint_index, "revolute"),
      "ChangeJointType.*can't change the type.*ephemeral.*static8.* only "
      "user-defined.*");
  EXPECT_TRUE(graph.forest_is_valid());

  // Should have added four ephemeral joints to the graph.
  EXPECT_EQ(graph.num_user_joints(), 7);
  EXPECT_EQ(ssize(graph.joints()), 11);
  const JointIndex base11_joint_index(9);  // See above picture.
  const JointIndex free9_joint_index(10);
  EXPECT_EQ(graph.joint_by_index(base11_joint_index).traits_index(),
            LinkJointGraph::quaternion_floating_joint_traits_index());
  EXPECT_EQ(graph.joint_by_index(free9_joint_index).traits_index(),
            LinkJointGraph::quaternion_floating_joint_traits_index());

  const std::vector<BodyIndex> link_composites0{BodyIndex(0), BodyIndex(7),
                                                BodyIndex(6), BodyIndex(8)};
  const std::vector<BodyIndex> link_composites1{BodyIndex(11), BodyIndex(10)};
  const std::vector<std::vector<BodyIndex>> expected_link_composites{
      link_composites0, link_composites1};
  EXPECT_EQ(graph.link_composites(), expected_link_composites);

  EXPECT_EQ(ssize(forest.mobods()), 12);
  EXPECT_EQ(ssize(forest.trees()), 6);
  EXPECT_EQ(forest.num_positions(), 19);
  EXPECT_EQ(forest.num_velocities(), 17);

  // Check inboard & outboard coordinate counts.

  // Counts for World.
  EXPECT_EQ(forest.world_mobod().nq(), 0);
  EXPECT_EQ(forest.world_mobod().nv(), 0);
  EXPECT_EQ(forest.world_mobod().nq_inboard(), 0);
  EXPECT_EQ(forest.world_mobod().nv_inboard(), 0);
  EXPECT_EQ(forest.world_mobod().nq_outboard(), forest.num_positions());
  EXPECT_EQ(forest.world_mobod().nv_outboard(), forest.num_velocities());
  EXPECT_EQ(forest.world_mobod().num_subtree_mobods(), ssize(forest.mobods()));

  // Counts for generic middle Mobod.
  const SpanningForest::Mobod& mobod_for_link3 =
      forest.mobods(graph.link_by_index(BodyIndex(3)).mobod_index());
  EXPECT_EQ(graph.links(mobod_for_link3.link_ordinal()).index(), BodyIndex(3));
  EXPECT_EQ(mobod_for_link3.q_start(), 2);
  EXPECT_EQ(mobod_for_link3.v_start(), 2);
  EXPECT_EQ(mobod_for_link3.nq(), 1);
  EXPECT_EQ(mobod_for_link3.nv(), 1);
  EXPECT_EQ(mobod_for_link3.nq_inboard(), 3);
  EXPECT_EQ(mobod_for_link3.nv_inboard(), 3);
  EXPECT_EQ(mobod_for_link3.nq_outboard(), 2);
  EXPECT_EQ(mobod_for_link3.nv_outboard(), 2);
  EXPECT_EQ(mobod_for_link3.num_subtree_mobods(), 3);

  // Counts for a Mobod with nq != nv.
  const SpanningForest::Mobod& mobod_for_base11 =
      forest.mobods(graph.link_by_index(BodyIndex(11)).mobod_index());
  EXPECT_EQ(mobod_for_base11.q_start(), 5);
  EXPECT_EQ(mobod_for_base11.v_start(), 5);
  EXPECT_EQ(mobod_for_base11.nq(), 7);
  EXPECT_EQ(mobod_for_base11.nv(), 6);
  EXPECT_EQ(mobod_for_base11.nq_inboard(), 7);
  EXPECT_EQ(mobod_for_base11.nv_inboard(), 6);
  EXPECT_EQ(mobod_for_base11.nq_outboard(), 0);
  EXPECT_EQ(mobod_for_base11.nv_outboard(), 0);
  EXPECT_EQ(mobod_for_base11.num_subtree_mobods(), 2);

  const std::vector<MobodIndex> welded_mobods0{MobodIndex(0), MobodIndex(6),
                                               MobodIndex(7), MobodIndex(8)};
  const std::vector<MobodIndex> welded_mobods1{MobodIndex(9), MobodIndex(10)};
  const std::vector expected_welded_mobods{welded_mobods0, welded_mobods1};
  EXPECT_EQ(forest.welded_mobods(), expected_welded_mobods);

  auto find_outv = [&forest](int mobod_index) -> pair<int, int> {
    return forest.mobods(MobodIndex(mobod_index)).outboard_velocities();
  };
  EXPECT_EQ(find_outv(0), pair(0, 17));                      // World
  EXPECT_TRUE(forest.mobods(MobodIndex(1)).is_base_body());  // Base bodies
  EXPECT_TRUE(forest.mobods(MobodIndex(7)).is_base_body());
  EXPECT_TRUE(forest.mobods(MobodIndex(9)).is_base_body());
  EXPECT_TRUE(forest.mobods(MobodIndex(11)).is_base_body());
  EXPECT_EQ(find_outv(1), pair(1, 4));
  EXPECT_EQ(find_outv(6), pair(5, 0));
  EXPECT_EQ(find_outv(9), pair(11, 0));
  EXPECT_EQ(find_outv(11), pair(17, 0));
  EXPECT_FALSE(forest.mobods(MobodIndex(3)).is_base_body());  // Generic case
  EXPECT_EQ(find_outv(3), pair(3, 2));
}

/* Topological loops formed entirely by welds can be handled specially when
we're combining LinkComposites onto single Mobods. We build a Forest containing
a number of kinematic loops and subgraphs of welded bodies.

TODO(sherm1) Combining composites is stubbed out but the first part of this
 test is still relevant since the composites are still computed, though not
 yet combined. More cases will follow.

The input is given as three unconnected graphs. Joints are shown with
parent->child direction. Double bars are welds, single bars are moving joints.
Links {0-13} are shown in braces, joint numbers 0-13 are plain.

    Link/Joint graph as input

                                               ===> weld
         12    11     9                        ---> revolute or prismatic
    {0}<==={5}===>{7}--->{2}                   {1} link # in braces
  World     ^      |10    |                    10  joint # plain
          13‖      v      |8
           {12}   {11}<---+

        3        0      7      4
    {3}--->{13}<==={1}--->{10}===>{6}
            2‖      ^      5‖      ‖
             v      ‖1      v      ‖6
            {4}=====+      {8}<====+

    {9}

When we build the forest, we have to provide every link with a path to World.
We'll first process the upper graph which already starts at World. Then we
have to pick a base body for the next graph. Link {3} should be chosen since
it appears only as a parent; it gets floating joint 14. Link {9} will also be
a base body; it gets floating joint 15.

There are three loops in this graph: {7-2-11}, {13-1-4}, and {10-6-8}. The
forest-building algorithm will encounter them in that order. The last two loops
are formed entirely of welds. When modeling in the mode where every Joint gets a
Mobod all of these must be broken by adding shadow links. In each case the loop
joint will be between two bodies at the same level in their tree, so the choice
of which one to split should be made so that the parent->child orientation in
the graph is preserved in the inboard->outboard order of the tree, requiring
that the child Link is the one split. (It is easier to see the levels in the
forest diagram below.) As a result, Link {11} will be split with shadow link
{14*} on Joint 8, Link {1} gets shadow {15*} on weld Joint 1, and Link {8} gets
shadow {16*} on weld Joint 6.

Therefore we expect the following Composites, with the "World" Composite first:
  {0, 5, 7, 12}, {13, 1, 4, 15*}, {10, 6, 8, 16*}
Note that the _active_ Link (the one on a moving joint) is always listed first
in a LinkComposite (World comes first in the World composite). The remaining
non-composite links are {3}, {9}, {2}, {11}, and {14*}.

Forest building should start with Link {5} since that is the only direct
connection to World in the input ({3} and {9} get connected later). If we're
giving every Link its own mobilizer (rather than making composites from
welded-together ones) we expect this forest of 3 trees and 17 Mobods:

      level 6                 12{16*} ....              ... loop constraint
                                         .              {1} link # in braces
      level 5                  11{6}  13{8}             10  mobod # plain

      level 4  4{14*} ...        10{10}    . 15{15*}
                        .                ..
      level 3   3{2} 5{11}         9{1}..   14{4}

      level 2      2{7}   6{12}        8{13}

  base mobods          1{5}            7{3}            16{9}
                          \             |               /
        World              ............0{0}.............

Note that the Mobod numbers shown here (and in all the tests) are _after_
renumbering into depth first order, not the order in which the Mobods were
assigned.

Some of the Links are welded together. We call those LinkComposites even
though each Link has its own Mobod. Those are:
{0 5 7 12} {13 1 4 15} {10 6 8 16}
The corresponding Mobods are in WeldedMobod groups:
[0 1 2 6] [8 9 14 15] [10 11 13 12]

TODO(sherm1) Retest with "combine composites" option on (currently stubbed). */
GTEST_TEST(SpanningForest, WeldedSubgraphs) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  graph.RegisterJointType("prismatic", 1, 1);
  EXPECT_EQ(ssize(graph.joint_traits()), 5);  // Predefined+revolute+prismatic.

  const ModelInstanceIndex model_instance(5);  // Arbitrary.

  // Define the forest.
  for (int i = 1; i <= 13; ++i)
    graph.AddLink("link" + std::to_string(i), model_instance);

  // clang-format off
  // Add joints:            type   parent  child
  std::vector<std::tuple<std::string, int, int>> joints{
    {"weld", 1, 13}, {"weld", 4, 1}, {"weld", 13, 4},  // loop 1 4 13
    {"revolute", 3, 13},
    {"weld", 10, 6}, {"weld", 10, 8}, {"weld", 6, 8},  // loop 6 8 10
    {"prismatic", 1, 10},
    {"prismatic", 2, 11}, {"prismatic", 7, 2}, {"prismatic", 7, 11},  // loop
                                                                      // 2 7 11
    {"weld", 5, 7}, {"weld", 5, 0}, {"weld", 12, 5}
  };
  // clang-format on
  for (int i = 0; i < ssize(joints); ++i) {
    const auto& joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), model_instance,
                   std::get<0>(joint), BodyIndex(std::get<1>(joint)),
                   BodyIndex(std::get<2>(joint)));
  }

  EXPECT_EQ(ssize(graph.links()), 14);  // Includes World.
  EXPECT_EQ(ssize(graph.joints()), 14);

  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();

  EXPECT_EQ(graph.num_user_links(), 14);  // Same as before building.
  EXPECT_EQ(graph.num_user_joints(), 14);
  EXPECT_EQ(ssize(graph.links()), 17);   // +3 shadows.
  EXPECT_EQ(ssize(graph.joints()), 16);  // +2 floating joints to world.
  EXPECT_EQ(ssize(graph.loop_constraints()), 3);

  // Check that the shadows are connected up properly. See the test comment
  // above for why we expect these particular links to get split.
  for (BodyIndex link(14); link <= 16; ++link)
    EXPECT_TRUE(graph.link_by_index(link).is_shadow());
  EXPECT_EQ(graph.link_by_index(BodyIndex(14)).primary_link(), 11);
  EXPECT_EQ(graph.link_by_index(BodyIndex(11)).num_shadows(), 1);
  EXPECT_EQ(graph.link_by_index(BodyIndex(15)).primary_link(), 1);
  EXPECT_EQ(graph.link_by_index(BodyIndex(1)).num_shadows(), 1);
  EXPECT_EQ(graph.link_by_index(BodyIndex(16)).primary_link(), 8);
  EXPECT_EQ(graph.link_by_index(BodyIndex(8)).num_shadows(), 1);

  // Check that we built the LinkComposites properly (see above).
  EXPECT_EQ(ssize(graph.link_composites()), 3);
  const std::vector<std::vector<int>> expected_links{
      {0, 5, 7, 12}, {13, 1, 4, 15}, {10, 6, 8, 16}};
  for (LinkCompositeIndex c(0); c < 3; ++c) {
    for (int link = 0; link < ssize(expected_links[c]); ++link)
      EXPECT_EQ(graph.link_composites(c)[link], expected_links[c][link]);
  }

  // Now let's verify that we got the expected SpanningForest. To understand,
  // refer to the 6-level forest diagram above.
  EXPECT_EQ(ssize(forest.mobods()), 17);
  EXPECT_EQ(ssize(forest.loop_constraints()), 3);

  // Expected level for each mobod in forest (index by MobodIndex).
  std::array<int, 17> expected_level{0, 1, 2, 3, 4, 3, 2, 1, 2,
                                     3, 4, 5, 6, 5, 3, 4, 1};
  for (auto& mobod : forest.mobods())
    EXPECT_EQ(mobod.level(), expected_level[mobod.index()]);

  // ith entry gives the modeled Link or Joint for Mobod i (see picture above).
  const std::array mobod2link{0, 5,  7, 2,  14, 11, 12, 3, 13,
                              1, 10, 6, 16, 8,  4,  15, 9};
  const std::array mobod2joint{-1, 12, 11, 9, 8, 10, 13, 14, 3,
                               0,  7,  4,  6, 5, 2,  1,  15};
  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    EXPECT_EQ(graph.links(mobod.link_ordinal()).index(),
              BodyIndex(mobod2link[mobod.index()]));
    if (mobod.is_world()) continue;  // No joint for World mobod.
    EXPECT_EQ(graph.joints(mobod.joint_ordinal()).index(),
              JointIndex(mobod2joint[mobod.index()]));
  }

  // Should get the same information from the graph.
  for (BodyIndex link{0}; link < ssize(graph.links()); ++link) {
    EXPECT_EQ(mobod2link[graph.link_to_mobod(link)], link);
  }

  // Check that we built the WeldedMobods groups properly (see above).
  EXPECT_EQ(ssize(forest.welded_mobods()), 3);
  const std::vector<std::vector<int>> expected_mobods{
      {0, 1, 2, 6}, {8, 9, 14, 15}, {10, 11, 13, 12}};
  for (WeldedMobodsIndex w(0); w < 3; ++w) {
    for (int mobod = 0; mobod < ssize(expected_mobods[w]); ++mobod)
      EXPECT_EQ(forest.welded_mobods(w)[mobod], expected_mobods[w][mobod]);
  }
}

/* Ten links, 8 in a tree and 2 free ones. Internal link 8 is massless (should
be no problem). Terminal links 2 and 4 are massless which will prevent dynamics
unless they are welded to a massful link. We'll test with articulation and
with a weld. Also we'll make a loop between massless 2 and massful 7 which
should make it ok since both branches will end with half link 7.

         Links     *=massless

      1   2*
        3      4*  7
        8*       9
            10          5   6

     ............0............ */
GTEST_TEST(SpanningForest, SimpleTrees) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);

  // We'll add Links and Joints to this arbitrary model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the graph.
  const std::set<int> massless{2, 4, 8};
  for (int i = 1; i <= 10; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance,
                  massless.contains(i) ? LinkFlags::kTreatAsMassless
                                       : LinkFlags::kDefault);
  }
  const std::vector<std::pair<int, int>> joints{
      {3, 1}, {3, 2}, {8, 3}, {10, 8}, {10, 9}, {9, 4}, {9, 7}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint" + std::to_string(i), model_instance, "revolute",
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 11);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  // We should report that the resulting forest is unsuited for dynamics due
  // to a terminal massless body. Specifically, it should complain about link
  // 4 rather than link 2 since 4 is at a lower level and should be seen first.
  EXPECT_FALSE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_FALSE(forest.dynamics_ok());
  EXPECT_THAT(
      forest.why_no_dynamics(),
      testing::MatchesRegex("Link link4 on revolute joint joint5.*terminal.*"
                            "singular.*cannot be used for dynamics.*"));

  // Change link 4's joint type to "weld". That should shift the complaint to
  // link 2.
  graph.ChangeJointType(JointIndex(5), "weld");
  EXPECT_FALSE(graph.BuildForest());
  EXPECT_FALSE(forest.dynamics_ok());
  EXPECT_THAT(
      forest.why_no_dynamics(),
      testing::MatchesRegex("Link link2 on revolute joint joint1.*terminal.*"
                            "singular.*cannot be used for dynamics.*"));

  // Finally if we connect link 2 to a massful link forming a loop, we should
  // get a dynamics-ready forest by splitting the massful link.
  graph.AddJoint("loop_2_to_7", model_instance, "revolute", BodyIndex(2),
                 BodyIndex(7));
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_TRUE(forest.dynamics_ok());
  EXPECT_TRUE(forest.why_no_dynamics().empty());
  EXPECT_EQ(graph.num_user_links(), 11);
  EXPECT_EQ(graph.num_user_joints(), 8);
  EXPECT_EQ(ssize(graph.links()), 12);   // Now has a shadow link.
  EXPECT_EQ(ssize(graph.joints()), 11);  // Added three floating joints.
}

/* Massless bodies should alter tree-building strategy.

LinkJointGraph:
  World -0-> {1} -1-> {2} -2-> {3}*
                                |
                                3
                                v
  World -4-> {5} -5-> {6} -6-> {4}*

With all massful Links, loop should be broken at Joint 3 (between links
{3} and {4} since that minimizes the maximum chain length. In that case we
have trees {1234s} and {564} where {4s} is the shadow of {4}.

If we make just {3}* massless we'll get the same Forest since we can see
that there is an outboard massful body {4}. When we split {4} both halves are
massful, so we'll still get {1234s} and {564}.

If we make _both_ {3}* and {4}* massless, modeling should start with {12} and
{56} but then next extend the first tree to {12346s} because we can't stop at
{3} or {4}. Joint 6 is the loop joint but the mobilizer has to be reversed so
that we end with a massful shadow link {6s} rather than the massless {4}. */
GTEST_TEST(SpanningForest, MasslessLinksChangeLoopBreaking) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);

  // We'll add Links and Joints to this model instance.
  const ModelInstanceIndex model_instance(5);

  // Define the forest.
  for (int i = 1; i <= 6; ++i) {
    graph.AddLink("link" + std::to_string(i), model_instance);
  }
  const std::vector<std::pair<int, int>> joints{{0, 1}, {1, 2}, {2, 3}, {3, 4},
                                                {0, 5}, {5, 6}, {6, 4}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint" + std::to_string(i), model_instance, "revolute",
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 7);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();

  EXPECT_EQ(ssize(graph.links()), 8);  // Added a shadow.
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(BodyIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(BodyIndex(7)).primary_link(), BodyIndex(4));

  EXPECT_EQ(ssize(graph.links()), 8);  // Added a shadow.
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(BodyIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(BodyIndex(7)).primary_link(), BodyIndex(4));

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 4);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 3);
  EXPECT_EQ(graph.links(forest.mobods(MobodIndex(4)).link_ordinal()).index(),
            BodyIndex(7));

  // Changing just 3 to massless results in the same forest.
  graph.ChangeLinkFlags(BodyIndex(3), LinkFlags::kTreatAsMassless);
  EXPECT_TRUE(graph.BuildForest());

  EXPECT_EQ(ssize(graph.links()), 8);
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(BodyIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(BodyIndex(7)).primary_link(), BodyIndex(4));

  EXPECT_EQ(ssize(graph.links()), 8);
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(BodyIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(BodyIndex(7)).primary_link(), BodyIndex(4));

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 4);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 3);
  EXPECT_EQ(graph.links(forest.mobods(MobodIndex(4)).link_ordinal()).index(),
            BodyIndex(7));

  // Changing both 3 and 4 to massless breaks the loop at 6 instead of 4.
  graph.ChangeLinkFlags(BodyIndex(4), LinkFlags::kTreatAsMassless);
  EXPECT_TRUE(graph.BuildForest());

  EXPECT_EQ(ssize(graph.links()), 8);
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(BodyIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(BodyIndex(7)).primary_link(), BodyIndex(6));

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 5);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 2);
  EXPECT_EQ(graph.links(forest.mobods(MobodIndex(5)).link_ordinal()).index(),
            BodyIndex(7));
}

/* Here is a tricky case that should be handled correctly and without warnings.
We have a short loop consisting of two massless base Links and a single massful
Link. The massful Link should be split into two half-massful bodies which are
sufficient to prevent both massless Links from being terminal.

   LinkJointGraph                         SpanningForest
     {1} = link 1                           Loop = loop constraint
     joint #s are plain                     Mobod #s are plain

          {3}            massful        2{3} - Loop ->  4{3s}

      🡕 2     🡔 3                        🡑               🡑

  {1}           {2}      massless       1{1}            3{2}

   🡑 0           🡑 1                     🡑 T0            🡑 T1     T = tree
                           ---->
 ........{0}........                     ........ 0 ........
        World                                   World

On the left we show the Link and Joint numbers as input, on the right we show
the Tree numbers and mobilized body numbers in proper depth-first order, with
the corresponding links. Arrows show the parent->child and inboard->outboard
directions. We expect to process Link 1 before Link 2 so we expect Tree 0 to
contain links {1} and {3} before we discover the loop trying to extend Tree 1
from link {2} via joint 3. In that case link {3} is at level 2 in the forest
while link {2} is at level 1 so we must split {3} to get the two trees to be
the same height. Link {3} is split so there is a shadow 3s, which has
ephemeral link number {4} which is mobilized by Mobod 4 in Tree 1. */
GTEST_TEST(SpanningForest, MasslessBodiesShareSplitLink) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  graph.RegisterJointType("prismatic", 1, 1);
  const ModelInstanceIndex model_instance(19);

  graph.AddLink("massless_1", model_instance, LinkFlags::kTreatAsMassless);
  graph.AddLink("massless_2", model_instance, LinkFlags::kTreatAsMassless);
  graph.AddLink("link_3", model_instance);

  graph.AddJoint("prismatic_0", model_instance, "prismatic", world_index(),
                 BodyIndex(1));
  graph.AddJoint("prismatic_1", model_instance, "prismatic", world_index(),
                 BodyIndex(2));
  graph.AddJoint("revolute_2", model_instance, "revolute", BodyIndex(1),
                 BodyIndex(3));
  graph.AddJoint("revolute_3", model_instance, "revolute", BodyIndex(2),
                 BodyIndex(3));

  EXPECT_EQ(ssize(graph.links()), 4);  // Before modeling (includes World).
  EXPECT_EQ(graph.num_user_links(), 4);

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();

  EXPECT_EQ(ssize(graph.links()), 5);  // After modeling.
  EXPECT_EQ(graph.num_user_links(), 4);
  EXPECT_EQ(ssize(graph.loop_constraints()), 1);

  const auto& shadow_link = graph.link_by_index(BodyIndex(4));
  EXPECT_TRUE(shadow_link.is_shadow());
  EXPECT_EQ(shadow_link.primary_link(), BodyIndex(3));
  EXPECT_EQ(shadow_link.mobod_index(), MobodIndex(4));
  EXPECT_EQ(shadow_link.inboard_joint_index(), JointIndex(3));
  EXPECT_EQ(ssize(shadow_link.joints()), 1);
  EXPECT_TRUE(shadow_link.joints_as_parent().empty());
  EXPECT_EQ(shadow_link.joints_as_child()[0], JointIndex(3));
  EXPECT_EQ(shadow_link.joints()[0], JointIndex(3));

  EXPECT_EQ(graph.link_by_index(BodyIndex(3)).num_shadows(), 1);
  EXPECT_EQ(graph.link_by_index(BodyIndex(2)).num_shadows(), 0);
  EXPECT_EQ(graph.link_by_index(BodyIndex(4)).num_shadows(), 0);

  EXPECT_EQ(ssize(forest.mobods()), 5);
  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees(TreeIndex(0)).num_mobods(), 2);
  EXPECT_EQ(forest.trees(TreeIndex(1)).num_mobods(), 2);
}

/* Here we have a floating double loop requiring two shadows of the same Link:

     {2} ------> {5}           Link numbers are in {}
      ^     3     | 6          Joint numbers are plain
    0 |           v            Arrows show parent->child direction
     {1}-->{3}-->{6}           All Links are massful
    1 |  2     5  ^            All Joints are articulated (no welds)
      v           | 7
     {4} ------> {7}
            4

Because Link {1} is never used as a child, it will be the preferred base Link
and get attached to World by a free Joint. The heuristic that tries to
minimize branch length should grow the three branches from {1} like this:
  {12}    {14}     {13}
  {125}   {147}    {136}
  {1256s} {1476ss} {136}
  where 6s and 6ss are shadows 1 and 2 of Link 6.

The expected as-modeled graph and spanning forest model are:

            {2}--->{5}--->{8}                  [2]-->[3]-->[4]  branch 1
           0 ^  3      6   . loop 0             ^           .
  World      |             .            World   |           .
   {0}----->{1}--->{3}--->{6}            [0]-->[1]-->[8]-->[9]  branch 3
        8    |  2      5   . loop 1             |           .
           1 v             .                    v           .
            {4}--->{7}--->{9}                  [5]-->[6]-->[7]  branch 2
                4      7

            Links & Joints                     Mobilized bodies

Notes:
  - Joint numbering determines branch ordering in the tree so the
    middle branch gets modeled last.
  - Model Joint 8 is the added floating joint to World.
  - Link {8} is {6s} (shadow 1 of Link {6}); {9} is {6ss} (shadow 2).
  - Loop constraints (shown as . .) are always ordered so that the constraint's
    "parent" is the primary link and "child" is the shadow link. Thus the order
    here should be 6->8 and 6->9.
  - Mobilized bodies (Mobods) are numbered depth-first.
*/
GTEST_TEST(SpanningForest, DoubleLoop) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  const ModelInstanceIndex model_instance(19);

  for (int i = 1; i <= 7; ++i)
    graph.AddLink("link" + std::to_string(i), model_instance);

  const std::vector<std::pair<int, int>> joints{{1, 2}, {1, 4}, {1, 3}, {2, 5},
                                                {4, 7}, {3, 6}, {5, 6}, {7, 6}};
  for (int i = 0; i < 8; ++i) {
    graph.AddJoint("joint" + std::to_string(i), model_instance, "revolute",
                   BodyIndex(joints[i].first), BodyIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 8);  // Before modeling (includes World).
  EXPECT_EQ(ssize(graph.joints()), 8);
  EXPECT_EQ(ssize(graph.loop_constraints()), 0);

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();

  EXPECT_EQ(ssize(graph.links()), 10);  // After modeling.
  EXPECT_EQ(ssize(graph.joints()), 9);
  EXPECT_EQ(ssize(graph.loop_constraints()), 2);
  EXPECT_EQ(graph.num_user_links(), 8);
  EXPECT_EQ(graph.num_user_joints(), 8);

  const LinkJointGraph::Link& primary_link = graph.link_by_index(BodyIndex(6));
  const LinkJointGraph::Link& shadow_link_1 = graph.link_by_index(BodyIndex(8));
  const LinkJointGraph::Link& shadow_link_2 = graph.link_by_index(BodyIndex(9));

  EXPECT_EQ(primary_link.name(), "link6");
  EXPECT_EQ(shadow_link_1.name(), "link6$1");
  EXPECT_EQ(shadow_link_2.name(), "link6$2");

  EXPECT_EQ(primary_link.num_shadows(), 2);
  EXPECT_TRUE(shadow_link_1.is_shadow());
  EXPECT_TRUE(shadow_link_2.is_shadow());

  EXPECT_EQ(graph.link_by_index(BodyIndex(5)).num_shadows(), 0);
  EXPECT_EQ(graph.link_by_index(BodyIndex(7)).num_shadows(), 0);

  EXPECT_EQ(ssize(forest.mobods()), 10);
  EXPECT_EQ(ssize(forest.trees()), 1);
  const SpanningForest::Tree& tree = forest.trees(TreeIndex(0));
  EXPECT_EQ(tree.num_mobods(), 9);
  EXPECT_EQ(tree.height(), 4);
  EXPECT_EQ(tree.base_mobod(), MobodIndex(1));
  EXPECT_EQ(tree.last_mobod(), MobodIndex(9));

  // ith entry gives the modeled Link or Joint for Mobod i (see picture above).
  const std::array mobod2link{0, 1, 2, 5, 8, 4, 7, 9, 3, 6};
  const std::array mobod2joint{-1, 8, 0, 3, 6, 1, 4, 7, 2, 5};
  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    EXPECT_EQ(graph.links(mobod.link_ordinal()).index(),
              BodyIndex(mobod2link[mobod.index()]));
    if (mobod.is_world()) continue;  // No joint for World mobod.
    EXPECT_EQ(graph.joints(mobod.joint_ordinal()).index(),
              JointIndex(mobod2joint[mobod.index()]));
  }

  const LinkJointGraph::LoopConstraint& loop0 =
      graph.loop_constraints(LoopConstraintIndex(0));
  const LinkJointGraph::LoopConstraint& loop1 =
      graph.loop_constraints(LoopConstraintIndex(1));
  EXPECT_EQ(loop0.index(), 0);
  EXPECT_EQ(loop1.index(), 1);
  EXPECT_EQ(loop0.model_instance(), model_instance);
  EXPECT_EQ(loop1.model_instance(), model_instance);
  EXPECT_EQ(loop0.primary_link(), BodyIndex(6));
  EXPECT_EQ(loop0.shadow_link(), BodyIndex(8));
  EXPECT_EQ(loop1.primary_link(), BodyIndex(6));
  EXPECT_EQ(loop1.shadow_link(), BodyIndex(9));

  // Added loop constraints should be named the same as their shadow Link.
  EXPECT_EQ(loop0.name(), shadow_link_1.name());
  EXPECT_EQ(loop1.name(), shadow_link_2.name());
}

/* For both link_composites and welded_mobods: the World composite must
come first (even if nothing is welded to World). This graph's first branch has
a composite that could be seen prior to the weld to World. We'll attempt
to trick it into following that path by using a massless body, requiring it
to extend the first branch to Link {2} before moving on to the next branch.
But we want to see the {0,3} composite before the {1,2} composite.

          +---> {1*} ===> {2}
      {0} | 0         1                {Links} & Joints
    World |                            ===> is a weld joint
          +===> {3}                    * Link 1 is massless
          | 2
          |
          +---> {4}
            3
*/
GTEST_TEST(SpanningForest, WorldCompositeComesFirst) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  const ModelInstanceIndex model_instance(5);  // arbitrary

  graph.AddLink("massless_link_1", model_instance, LinkFlags::kTreatAsMassless);
  graph.AddLink("link2", model_instance);
  graph.AddLink("link3", model_instance);
  graph.AddLink("link4", model_instance);

  const auto& world = graph.link_by_index(BodyIndex(0));
  const auto& massless_link = graph.link_by_index(BodyIndex(1));
  const auto& link2 = graph.link_by_index(BodyIndex(2));
  const auto& link3 = graph.link_by_index(BodyIndex(3));
  const auto& link4 = graph.link_by_index(BodyIndex(4));

  graph.AddJoint("joint0", model_instance, "revolute", world.index(),
                 massless_link.index());
  graph.AddJoint("joint1", model_instance, "weld", massless_link.index(),
                 link2.index());
  graph.AddJoint("joint2", model_instance, "weld", world.index(),
                 link3.index());
  graph.AddJoint("joint4", model_instance, "revolute", world.index(),
                 link4.index());

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();

  EXPECT_EQ(ssize(graph.links()), 5);
  EXPECT_EQ(ssize(forest.mobods()), 5);  // Because we're not combining.

  // "Anchored" means "fixed to World" (by welds).
  EXPECT_TRUE(world.is_anchored());
  EXPECT_FALSE(massless_link.is_anchored());
  EXPECT_FALSE(link2.is_anchored());
  EXPECT_TRUE(link3.is_anchored());
  EXPECT_FALSE(link4.is_anchored());

  EXPECT_EQ(ssize(graph.link_composites()), 2);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0)),
            (std::vector<BodyIndex>{BodyIndex(0), BodyIndex(3)}));
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(1)),
            (std::vector<BodyIndex>{BodyIndex(1), BodyIndex(2)}));

  EXPECT_EQ(ssize(forest.welded_mobods()), 2);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0)),
            (std::vector<MobodIndex>{MobodIndex(0), MobodIndex(3)}));
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(1)),
            (std::vector<MobodIndex>{MobodIndex(1), MobodIndex(2)}));
}

/* We always preserve the user's parent->child order for a joint, even if we
have to use a reversed mobilizer to do so. This requires some special handling
when we introduce a shadow body -- the shadow _mobod_ is always outboard (in
fact, terminal), but it might be the shadow of a _link_ that was the parent for
the joint being modeled. This test verifies that all the relevant bookkeeping is
done properly, both for the shadow body and the ephemeral Link we create for it
in the LinkJointGraph.

While we're here we'll verify that the shadow link naming policy works
properly when some crazy user has named their links to look like shadows.

 {} Link  [] Mobod  Ji Joint i
 {3s} shadow of link 3
 ---> revolute joint
 No weld joints, no massless or floating links.

          LinkJointGraph                      SpanningTree

           J0        J2
          +---->{1}------,                 +----->[1]{1}----->[2]{3}
          |              V                 |                   .
      {0} |             {3}            [0] |                   . Loop 0
    World |              |                 |                   .
          +---->{2}<-----'                 +----->[3]{2}----->[4]{3s}
            J1       J3
                                           Mobod 4 should be reversed since
                                           joint 3 goes {3}->{2}

After we verify the above, we'll change {3} to massless which will force
us to split link {2} instead. In that case we won't need a reverse mobilizer:

     Spanning Tree with {3*} massless

     +----->[1]{1}----->[2]{3*}       Note depth-first numbering of Mobods.
     |                    |
 [0] |                    |
     |                    v
     +----->[4]{2} . . . [3]{2s}
                   Loop 0
*/
GTEST_TEST(SpanningForest, ShadowLinkPreservesJointOrder) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  graph.AddLink("link3$1", default_model_instance());  // Awkward name!
  graph.AddLink("link2", default_model_instance());
  graph.AddLink("link3", default_model_instance());
  const std::vector<std::pair<int, int>> joints{{0, 1}, {0, 2}, {1, 3}, {3, 2}};
  for (int i = 0; i < ssize(joints); ++i) {
    const auto joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), default_model_instance(),
                   "revolute", BodyIndex(joint.first), BodyIndex(joint.second));
  }

  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_EQ(graph.num_user_joints(), 4);
  EXPECT_EQ(ssize(graph.joints()), 4);
  EXPECT_EQ(graph.num_user_links(), 4);
  EXPECT_EQ(ssize(graph.links()), 5);  // Added a shadow.

  // See right-hand graph above. We're expecting to split link 3 since that
  // will produce equal-length branches.
  const LinkJointGraph::Link& primary_link = graph.link_by_index(BodyIndex(3));
  EXPECT_FALSE(primary_link.is_shadow());
  EXPECT_EQ(primary_link.num_shadows(), 1);
  EXPECT_EQ(primary_link.mobod_index(), MobodIndex(2));
  EXPECT_EQ(primary_link.inboard_joint_index(), JointIndex(2));
  EXPECT_EQ(primary_link.primary_link(), primary_link.index());
  EXPECT_EQ(graph.link_to_mobod(primary_link.index()), MobodIndex(2));

  // The original connectivity should be preserved.
  EXPECT_EQ(primary_link.joints(), (std::vector{JointIndex(2), JointIndex(3)}));
  EXPECT_EQ(primary_link.joints_as_child(), (std::vector{JointIndex(2)}));
  EXPECT_EQ(primary_link.joints_as_parent(), (std::vector{JointIndex(3)}));

  const LinkJointGraph::Link& shadow_link = graph.link_by_index(BodyIndex(4));
  EXPECT_TRUE(shadow_link.is_shadow());
  // Shadow 1 of link 3 should be "link3$1", but that name conflicts with a
  // nutty user name so we have to disambiguate with underscores.
  EXPECT_EQ(shadow_link.name(), "_link3$1");
  EXPECT_EQ(shadow_link.num_shadows(), 0);
  EXPECT_EQ(shadow_link.mobod_index(), MobodIndex(4));
  EXPECT_EQ(shadow_link.inboard_joint_index(), JointIndex(3));
  EXPECT_EQ(shadow_link.primary_link(), primary_link.index());
  EXPECT_EQ(graph.link_to_mobod(shadow_link.index()), MobodIndex(4));

  // The shadow link also reports its (ephemeral) connectivity. Although it
  // is outboard of its mobilizer, it is the parent for the joint since that
  // was the original orientation of joint 3.
  EXPECT_EQ(shadow_link.joints(), (std::vector{JointIndex(3)}));
  EXPECT_TRUE(shadow_link.joints_as_child().empty());
  EXPECT_EQ(shadow_link.joints_as_parent(), (std::vector{JointIndex(3)}));

  EXPECT_EQ(ssize(forest.mobods()), 5);
  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees(TreeIndex(0)).base_mobod(), MobodIndex(1));
  EXPECT_EQ(forest.trees(TreeIndex(0)).num_mobods(), 2);
  EXPECT_EQ(forest.trees(TreeIndex(1)).base_mobod(), MobodIndex(3));
  EXPECT_EQ(forest.trees(TreeIndex(1)).num_mobods(), 2);

  // Check that only mobilizer 4 was reversed.
  const std::vector<bool> expect_reversed{false, false, false, false, true};
  for (auto joint : graph.joints()) {
    const auto& mobod = forest.mobods(joint.mobod_index());
    EXPECT_EQ(mobod.is_reversed(), expect_reversed[mobod.index()]);
  }

  // Now make link3 massless, rebuild, and check a few things.
  graph.ChangeLinkFlags(BodyIndex(3), LinkFlags::kTreatAsMassless);
  graph.BuildForest();
  const LinkJointGraph::Link& new_primary_link =
      graph.link_by_index(BodyIndex(2));
  const LinkJointGraph::Link& new_shadow_link =
      graph.link_by_index(BodyIndex(4));

  EXPECT_EQ(ssize(forest.mobods()), 5);
  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees(TreeIndex(0)).base_mobod(), MobodIndex(1));
  EXPECT_EQ(forest.trees(TreeIndex(0)).num_mobods(), 3);
  EXPECT_EQ(forest.trees(TreeIndex(1)).base_mobod(), MobodIndex(4));
  EXPECT_EQ(forest.trees(TreeIndex(1)).num_mobods(), 1);

  EXPECT_EQ(new_primary_link.mobod_index(), MobodIndex(4));
  EXPECT_EQ(new_primary_link.num_shadows(), 1);
  EXPECT_FALSE(new_primary_link.is_shadow());
  EXPECT_EQ(new_primary_link.primary_link(), new_primary_link.index());

  EXPECT_EQ(new_shadow_link.mobod_index(), MobodIndex(3));
  EXPECT_EQ(new_shadow_link.num_shadows(), 0);
  EXPECT_TRUE(new_shadow_link.is_shadow());
  EXPECT_EQ(new_shadow_link.primary_link(), new_primary_link.index());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
