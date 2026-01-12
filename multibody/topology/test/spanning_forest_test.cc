/* clang-format off to disable clang-format-includes */
#include "drake/multibody/topology/forest.h"
/* clang-format on */

#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
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
// a single WeldedLinksAssembly and proper modeling info.
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
  const LinkIndex world_link_index = graph.world_link().index();
  EXPECT_EQ(world_link_index, LinkIndex(0));
  const LinkOrdinal world_link_ordinal = graph.world_link().ordinal();
  EXPECT_EQ(world_link_ordinal, LinkOrdinal(0));
  EXPECT_FALSE(graph.world_link().is_massless());

  // Now build a forest representing the World-only graph.
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_TRUE(graph.forest_is_valid());
  EXPECT_TRUE(forest.is_valid());
  EXPECT_NO_THROW(forest.SanityCheckForest());
  EXPECT_EQ(&forest.graph(), &graph);
  const SpanningForest::Mobod& world = forest.mobods(MobodIndex(0));
  EXPECT_EQ(&world, &forest.world_mobod());
  const MobodIndex world_mobod_index = forest.world_mobod().index();
  EXPECT_EQ(world_mobod_index, MobodIndex(0));
  EXPECT_EQ(graph.link_to_mobod(world_link_index), MobodIndex(0));
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 1);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            std::vector{world_link_index});
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).is_massless());

  EXPECT_FALSE(forest.link_to_tree_index(LinkOrdinal(0)).is_valid());
  EXPECT_FALSE(forest.link_to_tree_index(LinkIndex(0)).is_valid());

  // Check that the World-only forest makes sense.
  EXPECT_EQ(ssize(forest.mobods()), 1);
  EXPECT_TRUE(forest.mobods(world_mobod_index).has_massful_follower_link());
  EXPECT_TRUE(forest.trees().empty());  // World isn't part of a tree.
  EXPECT_TRUE(forest.loop_constraints().empty());
  EXPECT_EQ(forest.height(), 1);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);
  EXPECT_EQ(forest.welded_mobods()[0][0], world_mobod_index);
  EXPECT_EQ(forest.mobod_to_link_ordinal(world_mobod_index),
            world_link_ordinal);
  EXPECT_EQ(forest.mobod_to_link_ordinals(world_mobod_index),
            std::vector{world_link_ordinal});
  EXPECT_EQ(forest.num_positions(), 0);
  EXPECT_EQ(forest.num_velocities(), 0);
  EXPECT_TRUE(forest.quaternion_starts().empty());
  EXPECT_EQ(forest.FindPathFromWorld(world_mobod_index),
            std::vector{world_mobod_index});  // Just World.
  EXPECT_EQ(forest.FindSubtreeLinks(world_mobod_index),
            std::vector{world_link_index});
  EXPECT_EQ(
      forest.FindFirstCommonAncestor(world_mobod_index, world_mobod_index),
      world_mobod_index);

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
  EXPECT_EQ(world.follower_link_ordinals(), std::vector{world_link_ordinal});
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
  EXPECT_EQ(world.subtree_velocities(), (pair{0, 0}));
  EXPECT_EQ(world.outboard_velocities(), (pair{0, 0}));

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

/* Basic proof-of-life tests of Tree and LoopConstraint APIs. */
GTEST_TEST(SpanningForest, TreeAndLoopConstraintAPIs) {
  LinkJointGraph graph;
  const SpanningForest& forest = graph.forest();
  graph.RegisterJointType("revolute", 1, 1);

  // Create this graph:
  //          -> link1 --+
  //    World            |
  //          -> link2 <-+

  graph.AddLink("link1", default_model_instance());
  graph.AddLink("link2", default_model_instance());
  graph.AddJoint("joint0", default_model_instance(), "revolute", LinkIndex(0),
                 LinkIndex(1));
  graph.AddJoint("joint1", default_model_instance(), "revolute", LinkIndex(0),
                 LinkIndex(2));
  graph.AddJoint("joint2", default_model_instance(), "revolute", LinkIndex(1),
                 LinkIndex(2));

  EXPECT_TRUE(graph.BuildForest());

  // Here's the forest we're expecting:
  //            -> mobod1 -> mobod2                             tree0
  //     World                 ^
  //            -> mobod3 =====+  loop weld constraint          tree1
  //
  // We had to cut link2. Mobod2 is for the shadow link.

  EXPECT_EQ(forest.num_trees(), 2);
  EXPECT_EQ(forest.num_mobods(), 4);
  EXPECT_EQ(ssize(forest.loop_constraints()), 1);

  EXPECT_FALSE(forest.link_to_tree_index(LinkIndex(0)).is_valid());
  EXPECT_EQ(forest.link_to_tree_index(LinkIndex(1)), TreeIndex(0));
  // Link2's primary follows mobod3, which is in tree1.
  EXPECT_EQ(forest.link_to_tree_index(LinkIndex(2)), TreeIndex(1));

  // The index and ordinal values are the same here.
  EXPECT_FALSE(forest.link_to_tree_index(LinkOrdinal(0)).is_valid());
  EXPECT_EQ(forest.link_to_tree_index(LinkOrdinal(1)), TreeIndex(0));
  EXPECT_EQ(forest.link_to_tree_index(LinkOrdinal(2)), TreeIndex(1));

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
  EXPECT_EQ(tree0.nq(), 2);
  EXPECT_EQ(tree0.nv(), 2);

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
  EXPECT_EQ(tree1.q_start(), 2);
  EXPECT_EQ(tree1.v_start(), 2);
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
  const std::vector<pair<int, int>> joints{
      {1, 4},  {4, 5}, {4, 6}, {6, 9},  {6, 10},                        // left
      {12, 3}, {3, 8}, {3, 7}, {8, 11}, {11, 13}, {11, 14}, {14, 15}};  // right
  for (int i = 0; i < ssize(joints); ++i) {
    const auto joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), link_to_instance[joint.second],
                   "revolute", LinkIndex(joint.first), LinkIndex(joint.second));
  }

  // Remove and re-add a joint to mess up the numbering. It will now have
  // JointIndex 12, and 7 should be unused. (Ephemeral joints will be added
  // by BuildForest() starting with JointIndex 13.)
  EXPECT_TRUE(graph.has_joint(JointIndex(7)));
  EXPECT_TRUE(graph.HasJointNamed("joint7", link_to_instance[7]));
  graph.RemoveJoint(JointIndex(7));  // The joint between 3 & 8.
  graph.AddJoint("joint7replaced", link_to_instance[7], "revolute",
                 LinkIndex(3), LinkIndex(7));
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
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(forest.options(), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(left_instance), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(right_instance), ForestBuildingOptions::kDefault);
  EXPECT_EQ(forest.options(free_link_instance),
            ForestBuildingOptions::kDefault);

  // See that the graph got augmented properly to reflect the as-built forest.
  EXPECT_EQ(graph.num_user_links(), 16);
  EXPECT_EQ(graph.num_links(), 16);  // no links added
  EXPECT_EQ(graph.num_user_joints(), 12);
  EXPECT_EQ(graph.num_joints(), 15);  // modeling adds 3 floating joints

  EXPECT_EQ(forest.num_links(), graph.num_links());
  EXPECT_EQ(forest.num_joints(), graph.num_joints());

  // The only WeldedLinksAssembly is the World assembly and it is alone there.
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 1);  // just World
  EXPECT_EQ(
      ssize(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links()),
      1);
  EXPECT_EQ(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links()[0],
      graph.world_link().index());

  EXPECT_EQ(forest.num_trees(), 3);
  EXPECT_EQ(forest.num_mobods(), 16);           // includes World
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
    EXPECT_EQ(graph.link_to_mobod(LinkIndex(mobod_link.second)),
              MobodIndex(mobod_link.first));
    EXPECT_EQ(forest.mobod_to_link_ordinal(MobodIndex(mobod_link.first)),
              LinkOrdinal(mobod_link.second));
    // Each Mobod has only a single Link that follows it.
    EXPECT_EQ(forest.mobod_to_link_ordinals(MobodIndex(mobod_link.first)),
              std::vector{LinkOrdinal(mobod_link.second)});
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
  EXPECT_EQ(forest.q_to_tree_index(9), TreeIndex(0));
  EXPECT_EQ(forest.q_to_tree_index(19), TreeIndex(1));
  EXPECT_EQ(forest.q_to_tree_index(30), TreeIndex(2));
  EXPECT_EQ(forest.v_to_tree_index(9), TreeIndex(0));
  EXPECT_EQ(forest.v_to_tree_index(19), TreeIndex(1));
  EXPECT_EQ(forest.v_to_tree_index(29), TreeIndex(2));
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

  const std::vector<MobodIndex> expected_path_from_14{
      {MobodIndex(0)}, {MobodIndex{7}}, {MobodIndex(8)}, {MobodIndex(14)}};
  EXPECT_EQ(forest.FindPathFromWorld(MobodIndex(14)), expected_path_from_14);

  // Mobods on different trees have only World as a common ancestor.
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(5), MobodIndex(14)),
            MobodIndex(0));

  // Check that long/short and short/long branch ordering both work since
  // they are handled by different code.
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(13), MobodIndex(14)),
            MobodIndex(8));  // See right hand drawing above.
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(14), MobodIndex(13)),
            MobodIndex(8));

  // Check special case: if either body is World the answer is World.
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(0), MobodIndex(13)),
            MobodIndex(0));
  EXPECT_EQ(forest.FindFirstCommonAncestor(MobodIndex(14), MobodIndex(0)),
            MobodIndex(0));
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

  // There is only the World assembly, but now tree1's base link and its
  // ephemeral weld joint are included.
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 1);  // just World
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            (std::vector{graph.world_link().index(),
                         graph.links(tree1.front().link_ordinal()).index()}));
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            (std::vector{JointIndex(14)}));  // Ephemeral joints start at 13.
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).is_massless());

  // Similarly, there is only one WeldedMobods group, containing just World
  // and tree1's base.
  EXPECT_EQ(forest.welded_mobods(),
            (std::vector<std::vector<MobodIndex>>{
                {forest.world_mobod().index(), tree1.base_mobod()}}));
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
  const std::vector<pair<int, int>> joints{{2, 1}, {2, 3}, {4, 5}, {6, 5},
                                           {6, 7}, {8, 9}, {10, 9}};
  for (int i = 0; i < ssize(joints); ++i) {
    const auto joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), default_model_instance(),
                   "revolute", LinkIndex(joint.first), LinkIndex(joint.second));
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
floating links, and some simple WeldedLinksAssemblies, obeying forest building
options.

Links can become static either by being members of a static model instance,
or by having been specified with the kStatic link flag; we test both of those
here. Forest building should add a weld joint to World for every static link
unless there is already an explicit weld; we'll check that.

WeldedLinksAssemblies are always computed and consist of subgraphs of links that
are mutually welded together (directly or indirectly). Depending on forest
building options, we may use a single Mobod for a WeldedLinksAssembly (an
"optimized assembly"), or we may use a Mobod for each of those welded-together
links (an "unoptimized assembly"). In the latter case we also
compute "welded Mobod" groups consisting of Mobods that are mutually
interconnected by weld mobilizers (directly or indirectly). When we're
optimizing (modeling each WeldedLinksAssembly with just one Mobod), there won't
be any welded-together Mobods. By convention, there will still be one welded
Mobod group, consisting just of the World Mobod.

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

SerialChain 1 (don't optimize WeldedLinkAssemblies)
---------------------------------------------------
  ≡> added weld joint       [mobods]
  6> added floating joint   {links}
  #n joint number n

  When building the forest in the mode where every Link gets its own Mobod
  (even if Links are welded together) the SpanningForest should have 6 trees of
  Mobods (World is not in a tree):

  tree      world [0]
              #0       #1       #2       #3       #4
     0 [1-5]  -> link1 -> link2 -> link3 -> link4 -> link5
     1 [6]    =>static7        #5
     2 [7]    ≡>static6        (added weld #7)
     3 [8]    ≡>static8        (added weld #8)
     4 [9-10] 6>base11=>link10 (added 6dof #9, reversed the user's weld #6)
     5 [11]   6>free9          (added 6dof #10, free bodies are always last)

  WeldedLinkAssemblies:
    {0 7 6 8} #5 #7 #8
    {11 10} #6
  Welded Mobods groups: [0 6 7 8] [9 10]

  The particular ordering results from (a) user-supplied Joints get processed
  before added ones, and (b) static model instance Links get welded prior
  to individually-specified static Links in non-static model instances.
  However, note that we do not promise any particular ordering other than
  (1) World is always present and is first, and (2) the active link comes first
  in any WeldedLinkAssembly.

We will also vary this graph in several ways and retest. The details are
described in the code below.
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

  LinkIndex parent = graph.AddLink("link1", model_instance);
  graph.AddJoint("pin1", model_instance, "revolute", world_index(), parent);
  for (int i = 2; i <= 5; ++i) {
    LinkIndex child = graph.AddLink("link" + std::to_string(i), model_instance);
    graph.AddJoint("pin" + std::to_string(i), model_instance, "revolute",
                   parent, child);
    parent = child;
  }

  graph.AddLink("static6", static_model_instance);
  const LinkIndex static7_index =
      graph.AddLink("static7", static_model_instance);
  const LinkIndex static8_index =
      graph.AddLink("static8", model_instance, LinkFlags::kStatic);
  // Manually adding a weld to World is allowable for a static Link.
  const JointIndex static7_joint_index =
      graph.AddJoint("static7_weld", model_instance, "weld",
                     graph.world_link().index(), static7_index);
  // Now add a free link and a free-floating pair.
  graph.AddLink("free9", model_instance);

  const LinkIndex link10_index = graph.AddLink("link10", model_instance);
  const LinkIndex base11_index =
      graph.AddLink("base11", model_instance, LinkFlags::kMustBeBaseBody);
  const JointIndex joint_10_11_index = graph.AddJoint(
      "weld", model_instance, "weld", link10_index, base11_index);

  // SerialChain 1 (not merging welded Links onto one Mobod)
  // -------------------------------------------------------
  graph.ResetForestBuildingOptions();  // Unnecessary; just being tidy.
  graph.SetForestBuildingOptions(static_model_instance,
                                 ForestBuildingOptions::kStatic);
  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_NO_THROW(forest.SanityCheckForest());
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

  const std::vector<LinkIndex> assembly0{LinkIndex(0), LinkIndex(7),
                                         LinkIndex(6), LinkIndex(8)};
  const std::vector<JointIndex> joints0{JointIndex(5), JointIndex(7),
                                        JointIndex(8)};
  const std::vector<LinkIndex> assembly1{LinkIndex(11), LinkIndex(10)};
  const std::vector<JointIndex> joints1{JointIndex(6)};

  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            assembly0);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            joints0);
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).is_massless());
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links(),
            assembly1);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints(),
            joints1);
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).is_massless());

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

  EXPECT_EQ(
      graph.FindSubtreeLinks(graph.world_link().index()),
      (std::vector{LinkIndex(0), LinkIndex(1), LinkIndex(2), LinkIndex(3),
                   LinkIndex(4), LinkIndex(5), LinkIndex(7), LinkIndex(6),
                   LinkIndex(8), LinkIndex(11), LinkIndex(10), LinkIndex(9)}));

  // Counts for generic middle Mobod.
  const SpanningForest::Mobod& mobod_for_link3 =
      forest.mobods(graph.link_by_index(LinkIndex(3)).mobod_index());
  EXPECT_EQ(graph.links(mobod_for_link3.link_ordinal()).index(), LinkIndex(3));
  EXPECT_EQ(mobod_for_link3.q_start(), 2);
  EXPECT_EQ(mobod_for_link3.v_start(), 2);
  EXPECT_EQ(mobod_for_link3.nq(), 1);
  EXPECT_EQ(mobod_for_link3.nv(), 1);
  EXPECT_EQ(mobod_for_link3.nq_inboard(), 3);
  EXPECT_EQ(mobod_for_link3.nv_inboard(), 3);
  EXPECT_EQ(mobod_for_link3.nq_outboard(), 2);
  EXPECT_EQ(mobod_for_link3.nv_outboard(), 2);
  EXPECT_EQ(mobod_for_link3.num_subtree_mobods(), 3);
  EXPECT_EQ(graph.FindSubtreeLinks(LinkIndex(3)),
            (std::vector{LinkIndex(3), LinkIndex(4), LinkIndex(5)}));

  // Counts for a Mobod with nq != nv.
  const SpanningForest::Mobod& mobod_for_base11 =
      forest.mobods(graph.link_by_index(LinkIndex(11)).mobod_index());
  EXPECT_EQ(mobod_for_base11.q_start(), 5);
  EXPECT_EQ(mobod_for_base11.v_start(), 5);
  EXPECT_EQ(mobod_for_base11.nq(), 7);
  EXPECT_EQ(mobod_for_base11.nv(), 6);
  EXPECT_EQ(mobod_for_base11.nq_inboard(), 7);
  EXPECT_EQ(mobod_for_base11.nv_inboard(), 6);
  EXPECT_EQ(mobod_for_base11.nq_outboard(), 0);
  EXPECT_EQ(mobod_for_base11.nv_outboard(), 0);
  EXPECT_EQ(mobod_for_base11.num_subtree_mobods(), 2);
  EXPECT_EQ(graph.FindSubtreeLinks(LinkIndex(11)),
            (std::vector{LinkIndex(11), LinkIndex(10)}));

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

  /* SerialChain 2 (optimize WeldedLinksAssemblies)
  -------------------------------------------------
  If instead we ask to merge welded Links we should get a much smaller
  forest:

    tree      {world static7 static6 static8} [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6]    6>{base11<=link10} (added 6dof, unmodeled weld)
     2 [7]    6>free9            (added 6dof, free bodies are always last)

    WeldedLinksAssemblies: (no change)
      {0 7 6 8} #5 #7 #8
      {11 10} #6
    WeldedMobods groups:   [0]  (just the World group) */

  graph.ResetForestBuildingOptions();  // Restore default options.
  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  graph.SetForestBuildingOptions(
      static_model_instance,
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies |
          ForestBuildingOptions::kStatic);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  // The graph shouldn't change from SpanningForest 1, but the forest will.
  EXPECT_EQ(graph.num_joints() - graph.num_user_joints(), 4);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            assembly0);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            joints0);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links(),
            assembly1);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints(),
            joints1);

  EXPECT_EQ(ssize(forest.mobods()), 8);
  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // Just World.

  // Starting with a Link somewhere in an assembly, we should get all the
  // Links on that assembly followed by anything outboard.
  EXPECT_EQ(graph.FindSubtreeLinks(LinkIndex(10)),
            (std::vector{LinkIndex(11), LinkIndex(10)}));
  EXPECT_EQ(
      graph.FindSubtreeLinks(LinkIndex(6)),
      (std::vector{LinkIndex(0), LinkIndex(7), LinkIndex(6), LinkIndex(8),
                   LinkIndex(1), LinkIndex(2), LinkIndex(3), LinkIndex(4),
                   LinkIndex(5), LinkIndex(11), LinkIndex(10), LinkIndex(9)}));

  /* SerialChain 3 (optimize WeldedLinksAssemblies except for 10 & 11)
  --------------------------------------------------------------------
  We can optionally insist that a weld joint within an assembly that would
  otherwise be ignored is actually modeled with a weld mobilizer (useful if
  you need to know reaction forces within that weld). We'll rebuild but
  specifying that the joint between link10 and link11 must be modeled. That
  should produce this forest:

    tree      {world static7 static6 static8} [0]
     0 [1-5]  ->link1->link2->link3->link4->link5
     1 [6-7]  6>{base11<=link10} (added 6dof, weld is now modeled)
     2 [8]    6>free9            (added 6dof, free bodies are always last)

    WeldedLinksAssemblies: (no change)
      {0 7 6 8} #5 #7 #8
      {11 10} #6
    WeldedMobods groups:   [0] [6 7] */

  // Now force one of the joints in an assembly to be modeled (meaning it
  // should get its own Mobod). This should split that assembly into
  // two Mobods, which should be noted as a WeldedMobods group.
  graph.ChangeJointFlags(joint_10_11_index, JointFlags::kMustBeModeled);
  // Built the forest with same options as used for 2a.
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            assembly0);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            joints0);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links(),
            assembly1);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints(),
            joints1);

  EXPECT_EQ(ssize(forest.mobods()), 9);
  EXPECT_EQ(ssize(forest.trees()), 3);
  EXPECT_EQ(ssize(forest.welded_mobods()), 2);
  const std::vector<MobodIndex> now_expected{MobodIndex(6), MobodIndex(7)};
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(1)), now_expected);

  /* SerialChain 4 (optimize WeldedLinksAssemblies and use a fixed base)
  ----------------------------------------------------------------------
  Finally, we'll restore joint 10-11 to its default setting and build again but
  this time with the kUseFixedBase option for model_instance.
  That means we'll use weld joints rather than floating joints for links that
  have no path to World in the input graph. Now we expect this forest:

    tree        {world static7 static6 static8 base11 link10 free9} [0]
      0  [1-5]  ->link1->link2->link3->link4->link5

    WeldedLinksAssemblies:
      {0 7 6 8 11 10 9} #5 #6 #7 #8 #9 #10
    WeldedMobods groups:   [0]  (just World) */

  // Put the joint back the way we found it.
  graph.ChangeJointFlags(joint_10_11_index, JointFlags::kDefault);
  graph.ResetForestBuildingOptions();  // Back to defaults.
  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  // Caution: we must specify all the forest building options we want for
  // a model instance; it won't inherit any of the global ones if set.
  graph.SetForestBuildingOptions(
      model_instance, ForestBuildingOptions::kOptimizeWeldedLinksAssemblies |
                          ForestBuildingOptions::kUseFixedBase);
  graph.SetForestBuildingOptions(
      static_model_instance,
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies |
          ForestBuildingOptions::kStatic);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(forest.mobods()), 6);
  EXPECT_EQ(ssize(forest.trees()), 1);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // Just World.
  const std::vector<LinkIndex> expected_assembly_links{
      LinkIndex(0),  LinkIndex(7),  LinkIndex(6), LinkIndex(8),
      LinkIndex(11), LinkIndex(10), LinkIndex(9)};
  // Order
  const std::vector<JointIndex> expected_assembly_joints{
      JointIndex(5), JointIndex(7), JointIndex(8),
      JointIndex(9), JointIndex(6), JointIndex(10)};
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 1);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            expected_assembly_links);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            expected_assembly_joints);
}

/* Topological loops formed entirely by welds can be handled specially when
we're optimizing WeldedLinksAssemblies onto single Mobods. We build a Forest
containing a number of kinematic loops and subgraphs of welded bodies.

The input is given as three unconnected subgraphs. Joints are shown with
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
are formed entirely of welds. When modeling in the unoptimized mode where every
Joint gets a Mobod all of these must be broken by adding shadow links. In each
case the loop joint will be between two bodies at the same level in their tree,
so the choice of which one to split should be made so that the parent->child
orientation in the graph is preserved in the inboard->outboard order of the
tree, requiring that the child Link is the one split. (It is easier to see the
levels in the forest diagram below.) As a result, Link {11} will be split with
shadow link {14*} on Joint 8, Link {1} gets shadow {15*} on weld Joint 1, and
Link {8} gets shadow {16*} on weld Joint 6.

Therefore we expect the following unoptimized WeldedLinksAssemblies, with the
World Assembly first:
  links            joints
  {0, 5, 7, 12}    12 11 13
  {13, 1, 4, 15*}  0 2 1
  {10, 6, 8, 16*}  4 5 6
Note that the _active_ Link (the one on a moving joint) is always listed first
in a WeldedLinksAssembly (World comes first in the World assembly). The
remaining non-assembly links are {3}, {9}, {2}, {11}, and {14*}.

Forest building should start with Link {5} since that is the only direct
connection to World in the input ({3} and {9} get connected later). If we're
giving every Link its own mobilizer (rather than optimizing assemblies to use
a single Mobod) we expect this forest of 3 trees and 17 Mobods:

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

Some of the Links are welded together. We call those WeldedLinksAssemblies even
though each Link has its own Mobod. Those are:
{0 5 7 12} {13 1 4 15*} {10 6 8 16*}
The corresponding Mobods are in WeldedMobod groups:
[0 1 2 6] [8 9 14 15] [10 11 13 12]

Remodeling with WeldLinksAssembly optimization turned on should immediately
create assembly {0 5 7 12} on mobod 0, then see outboard links {2} and {11} as
new base bodies and grow those two trees, discovering a loop at joint 8. As
before, Link {11} gets split with a shadow link {14} for joint 8. Then it
should choose link {3} as a base link and add floating joint 14, and grow that
tree. Finally it makes free link {9} a base body. The forest should then look
like this:


      level 3                         6{10 6 8}
      level 2      2{14}              5{13 1 4}
  base mobods       1{2}    3{11}     4{3}        7{9}  (four trees)
                      \       \        |           /
        World          ...........0{0 5 7 12}......

In this case we don't need to split the all-Weld loops since they are now
just assemblies. Note that processing by level proceeds until we reach a new
Mobod, so all the joints in an optimized assembly are processed together. Thus
the joint ordering is different than the previous case.
  links       joints
  {0 5 7 12}  12 11 13
  {13 1 4}    0 1 2
  {10 6 8}    4 6 5
There are no Welded Mobods (except World alone). */
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
                   std::get<0>(joint), LinkIndex(std::get<1>(joint)),
                   LinkIndex(std::get<2>(joint)));
  }

  EXPECT_EQ(ssize(graph.links()), 14);  // Includes World.
  EXPECT_EQ(ssize(graph.joints()), 14);

  // We can calculate the subgraphs without first building a Forest. This
  // function should only consider user Links, not shadow links even if they
  // have already been created.
  // {0 5 7 12} {1 4 13} {6 8 10} {2} {3} {9} {11}  (see first drawing above)
  const std::vector<std::set<LinkIndex>> expected_before_subgraphs{
      {LinkIndex(0), LinkIndex(5), LinkIndex(7), LinkIndex(12)},
      {LinkIndex(1), LinkIndex(4), LinkIndex(13)},
      {LinkIndex(6), LinkIndex(8), LinkIndex(10)},
      {LinkIndex(2)},
      {LinkIndex(3)},
      {LinkIndex(9)},
      {LinkIndex(11)}};
  const std::vector<std::set<LinkIndex>> before_subgraphs =
      graph.CalcSubgraphsOfWeldedLinks();
  EXPECT_EQ(before_subgraphs.size(), 7);
  EXPECT_EQ(before_subgraphs, expected_before_subgraphs);

  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(graph.num_user_links(), 14);  // Same as before building.
  EXPECT_EQ(graph.num_user_joints(), 14);
  EXPECT_EQ(ssize(graph.links()), 17);   // +3 shadows.
  EXPECT_EQ(ssize(graph.joints()), 16);  // +2 floating joints to world.
  EXPECT_EQ(ssize(graph.loop_constraints()), 3);

  // Check that the shadows are connected up properly. See the test comment
  // above for why we expect these particular links to get split.
  auto check_shadows = [&graph](LinkIndex primary_index, LinkIndex shadow_index,
                                JointIndex joint_index) {
    const auto& primary = graph.link_by_index(primary_index);
    const auto& shadow = graph.link_by_index(shadow_index);
    EXPECT_EQ(primary.num_shadows(), 1);
    EXPECT_TRUE(primary.HasJoint(joint_index));  // it's still there, but ...
    EXPECT_TRUE(primary.JointHasMovedToShadowLink(joint_index));
    EXPECT_TRUE(shadow.is_shadow());
    EXPECT_EQ(shadow.primary_link(), primary_index);
    EXPECT_TRUE(shadow.HasJoint(joint_index));  // retargeted
    EXPECT_FALSE(shadow.JointHasMovedToShadowLink(joint_index));
    // In each case the shadow is the joint's child link.
    const auto& joint = graph.joint_by_index(joint_index);
    EXPECT_EQ(joint.child_link_index(), primary_index);           // originally
    EXPECT_EQ(joint.effective_child_link_index(), shadow_index);  // after split
    EXPECT_EQ(joint.other_effective_link_index(shadow_index),
              joint.effective_parent_link_index());
  };

  // Link 11 is split with 14 as its shadow. Joint 8 moves to the shadow.
  check_shadows(LinkIndex(11), LinkIndex(14), JointIndex(8));
  // Link 1 is split with 15 as its shadow. Joint 1 moves to the shadow.
  check_shadows(LinkIndex(1), LinkIndex(15), JointIndex(1));
  // Link 8 is split with 16 as its shadow. Joint 6 moves to the shadow.
  check_shadows(LinkIndex(8), LinkIndex(16), JointIndex(6));

  // Check that we built the WeldedLinksAssemblies properly (see above).
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 3);
  const std::vector<std::vector<int>> expected_links{
      {0, 5, 7, 12}, {13, 1, 4, 15}, {10, 6, 8, 16}};
  const std::vector<std::vector<int>> expected_joints{
      {12, 11, 13}, {0, 2, 1}, {4, 5, 6}};
  for (WeldedLinksAssemblyIndex a(0); a < 3; ++a) {
    EXPECT_FALSE(graph.welded_links_assemblies(a).is_massless());
    for (int link = 0; link < ssize(expected_links[a]); ++link)
      EXPECT_EQ(graph.welded_links_assemblies(a).links()[link],
                expected_links[a][link]);
    for (int joint = 0; joint < ssize(expected_joints[a]); ++joint)
      EXPECT_EQ(graph.welded_links_assemblies(a).joints()[joint],
                expected_joints[a][joint]);
  }

  // After building the Forest and adding shadow bodies, the non-Forest-using
  // subgraph method should not change its result. The Forest-using fast one
  // will include Shadow Links as well as the user's.
  EXPECT_EQ(graph.CalcSubgraphsOfWeldedLinks(),
            expected_before_subgraphs);  // no change

  const std::vector<std::set<LinkIndex>> welded_subgraphs =
      graph.GetSubgraphsOfWeldedLinks();

  // Verify number of expected subgraphs.
  EXPECT_EQ(welded_subgraphs.size(), 8);

  // The first subgraph must contain the world.
  const std::set<LinkIndex> world_subgraph = welded_subgraphs[0];
  EXPECT_EQ(world_subgraph.count(world_index()), 1);

  // Build the expected set of subgraphs (see above).
  std::set<std::set<LinkIndex>> expected_subgraphs;
  // {0, 5, 7, 12}, {1, 4, 13, 15}, {6, 8, 10, 16}, {3}, {9}, {2}, {11}, {14}
  const std::set<LinkIndex>& expected_world_subgraph =
      *expected_subgraphs
           .insert({LinkIndex(0), LinkIndex(5), LinkIndex(7), LinkIndex(12)})
           .first;
  const std::set<LinkIndex>& expected_subgraphA =
      *expected_subgraphs
           .insert({LinkIndex(1), LinkIndex(4), LinkIndex(13), LinkIndex(15)})
           .first;
  const std::set<LinkIndex>& expected_subgraphB =
      *expected_subgraphs
           .insert({LinkIndex(6), LinkIndex(8), LinkIndex(10), LinkIndex(16)})
           .first;
  expected_subgraphs.insert({LinkIndex(3)});
  expected_subgraphs.insert({LinkIndex(9)});
  expected_subgraphs.insert({LinkIndex(2)});
  expected_subgraphs.insert({LinkIndex(11)});
  expected_subgraphs.insert({LinkIndex(14)});

  // We do expect the first subgraph to correspond to the set of bodies welded
  // to the world.
  EXPECT_EQ(world_subgraph, expected_world_subgraph);

  // In order to compare the computed list of welded bodies against the expected
  // list, irrespective of the ordering in the computed list, we first convert
  // the computed subgraphs to a set.
  const std::set<std::set<LinkIndex>> welded_subgraphs_set(
      welded_subgraphs.begin(), welded_subgraphs.end());
  EXPECT_EQ(welded_subgraphs_set, expected_subgraphs);

  // Verify we can query the list of bodies welded to a particular Link.
  EXPECT_EQ(graph.GetLinksWeldedTo(LinkIndex(9)).size(), 1);
  EXPECT_EQ(graph.GetLinksWeldedTo(LinkIndex(11)).size(), 1);
  EXPECT_EQ(graph.GetLinksWeldedTo(LinkIndex(4)), expected_subgraphA);
  EXPECT_EQ(graph.GetLinksWeldedTo(LinkIndex(13)), expected_subgraphA);
  EXPECT_EQ(graph.GetLinksWeldedTo(LinkIndex(10)), expected_subgraphB);
  EXPECT_EQ(graph.GetLinksWeldedTo(LinkIndex(6)), expected_subgraphB);

  // Now let's verify that we got the expected SpanningForest. To understand,
  // refer to the 6-level forest diagram above.
  EXPECT_EQ(ssize(forest.mobods()), 17);
  EXPECT_EQ(ssize(forest.loop_constraints()), 3);

  // Note that this is a question about how these Links got modeled, not
  // about the original graph.
  EXPECT_EQ(graph.FindFirstCommonAncestor(LinkIndex(11), LinkIndex(12)),
            LinkIndex(5));
  EXPECT_EQ(graph.FindFirstCommonAncestor(LinkIndex(16), LinkIndex(15)),
            LinkIndex(13));
  EXPECT_EQ(graph.FindFirstCommonAncestor(LinkIndex(10), LinkIndex(2)),
            LinkIndex(0));

  // Repeat but this time collect the paths to the ancestor.
  std::vector<MobodIndex> path1, path2;

  // Check with path1 longer.
  EXPECT_EQ(
      forest.FindPathsToFirstCommonAncestor(
          graph.link_by_index(LinkIndex(11)).mobod_index(),
          graph.link_by_index(LinkIndex(12)).mobod_index(), &path1, &path2),
      graph.link_by_index(LinkIndex(5)).mobod_index());
  EXPECT_EQ(path1, (std::vector<MobodIndex>{MobodIndex(5), MobodIndex(2)}));
  EXPECT_EQ(path2, (std::vector<MobodIndex>{MobodIndex(6)}));

  // Check with path2 longer.
  EXPECT_EQ(
      forest.FindPathsToFirstCommonAncestor(
          graph.link_by_index(LinkIndex(15)).mobod_index(),
          graph.link_by_index(LinkIndex(16)).mobod_index(), &path1, &path2),
      graph.link_by_index(LinkIndex(13)).mobod_index());
  EXPECT_EQ(path1, (std::vector<MobodIndex>{MobodIndex(15), MobodIndex(14)}));
  EXPECT_EQ(path2, (std::vector<MobodIndex>{MobodIndex(12), MobodIndex(11),
                                            MobodIndex(10), MobodIndex(9)}));

  EXPECT_EQ(
      forest.FindPathsToFirstCommonAncestor(
          graph.link_by_index(LinkIndex(10)).mobod_index(),
          graph.link_by_index(LinkIndex(2)).mobod_index(), &path1, &path2),
      graph.link_by_index(LinkIndex(0)).mobod_index());
  EXPECT_EQ(path1, (std::vector<MobodIndex>{MobodIndex(10), MobodIndex(9),
                                            MobodIndex(8), MobodIndex(7)}));
  EXPECT_EQ(path2, (std::vector<MobodIndex>{MobodIndex(3), MobodIndex(2),
                                            MobodIndex(1)}));

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
              LinkIndex(mobod2link[mobod.index()]));
    if (mobod.is_world()) continue;  // No joint for World mobod.
    EXPECT_EQ(graph.joints(mobod.joint_ordinal()).index(),
              JointIndex(mobod2joint[mobod.index()]));
  }

  // Should get the same information from the graph.
  for (LinkIndex link{0}; link < ssize(graph.links()); ++link) {
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

  // Now optimize WeldedLinkAssemblies so they get a single Mobod.
  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(graph.links()), 15);  // Only one added shadow.
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 3);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            (std::vector<LinkIndex>{LinkIndex(0), LinkIndex(5), LinkIndex(7),
                                    LinkIndex(12)}));
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            (std::vector<JointIndex>{JointIndex(12), JointIndex(11),
                                     JointIndex(13)}));
  EXPECT_EQ(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links(),
      (std::vector<LinkIndex>{LinkIndex(13), LinkIndex(1), LinkIndex(4)}));
  EXPECT_EQ(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints(),
      (std::vector<JointIndex>{JointIndex(0), JointIndex(1), JointIndex(2)}));
  EXPECT_EQ(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(2)).links(),
      (std::vector<LinkIndex>{LinkIndex(10), LinkIndex(6), LinkIndex(8)}));
  EXPECT_EQ(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(2)).joints(),
      (std::vector<JointIndex>{JointIndex(4), JointIndex(6), JointIndex(5)}));
  for (WeldedLinksAssemblyIndex i(0); i < 3; ++i) {
    EXPECT_FALSE(graph.welded_links_assemblies(i).is_massless());
  }

  // Now let's verify that we got the expected SpanningForest. To understand,
  // refer to the shorter (max level 3) diagram above.
  EXPECT_EQ(ssize(forest.mobods()), 8);
  std::array<int, 8> expected_level_merged{0, 1, 2, 1, 1, 2, 3, 1};
  for (auto& mobod : forest.mobods()) {
    EXPECT_EQ(mobod.level(), expected_level_merged[mobod.index()]);
  }

  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World
  EXPECT_EQ(ssize(forest.welded_mobods(WeldedMobodsIndex(0))), 1);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0))[0], MobodIndex(0));
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
    graph.AddLink(
        "link" + std::to_string(i), model_instance,
        massless.contains(i) ? LinkFlags::kMassless : LinkFlags::kDefault);
  }
  const std::vector<pair<int, int>> joints{{3, 1},  {3, 2}, {8, 3}, {10, 8},
                                           {10, 9}, {9, 4}, {9, 7}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint" + std::to_string(i), model_instance, "revolute",
                   LinkIndex(joints[i].first), LinkIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 11);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  // We should report that the resulting forest is unsuited for dynamics due
  // to a terminal massless body. Specifically, it should complain about link
  // 4 rather than link 2 since 4 is at a lower level and should be seen first.
  // (Tests Case 1 in ExtendTreesOneLevel())
  EXPECT_FALSE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_FALSE(forest.dynamics_ok());
  EXPECT_THAT(
      forest.why_no_dynamics(),
      testing::MatchesRegex("Link link4 on revolute joint joint5.*terminal.*"
                            "singular.*cannot be used for dynamics.*"));

  // Change link 4's joint type to "weld". That should shift the complaint to
  // link 2. (Tests Case 1 in ExtendTreesOneLevel())
  // Also, we should get a massful assembly {4,9}, with 9 the active
  // link (so must be listed first in the WeldedLinksAssembly).
  graph.ChangeJointType(JointIndex(5), "weld");
  EXPECT_FALSE(graph.BuildForest());
  EXPECT_FALSE(forest.dynamics_ok());
  EXPECT_THAT(
      forest.why_no_dynamics(),
      testing::MatchesRegex("Link link2 on revolute joint joint1.*terminal.*"
                            "singular.*cannot be used for dynamics.*"));
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 2);
  const std::vector<LinkIndex>& assembly94 =
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links();
  const std::vector<JointIndex>& joints94 =
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints();
  EXPECT_EQ(assembly94, (std::vector<LinkIndex>{LinkIndex(9), LinkIndex(4)}));
  EXPECT_EQ(joints94, std::vector<JointIndex>{JointIndex(5)});
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).is_massless());

  // Finally if we connect link 2 to a massful link forming a loop, we should
  // get a dynamics-ready forest by splitting the massful link.
  graph.AddJoint("loop_2_to_7", model_instance, "revolute", LinkIndex(2),
                 LinkIndex(7));
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
  const std::vector<pair<int, int>> joints{{0, 1}, {1, 2}, {2, 3}, {3, 4},
                                           {0, 5}, {5, 6}, {6, 4}};
  for (int i = 0; i < 7; ++i) {
    graph.AddJoint("joint" + std::to_string(i), model_instance, "revolute",
                   LinkIndex(joints[i].first), LinkIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 7);  // this includes the world Link.
  EXPECT_EQ(ssize(graph.joints()), 7);

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(graph.links()), 8);  // Added a shadow.
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(LinkIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(LinkIndex(7)).primary_link(), LinkIndex(4));

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 4);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 3);
  EXPECT_EQ(graph.links(forest.mobods(MobodIndex(4)).link_ordinal()).index(),
            LinkIndex(7));

  // Changing just 3 to massless results in the same forest.
  // (Tests Case 2 in ExtendTreesOneLevel())
  graph.ChangeLinkFlags(LinkIndex(3), LinkFlags::kMassless);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  // Check that links not in an assembly still respond correctly.
  EXPECT_TRUE(graph.link_and_its_assembly_are_massless(LinkOrdinal(3)));

  EXPECT_EQ(ssize(graph.links()), 8);
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(LinkIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(LinkIndex(7)).primary_link(), LinkIndex(4));

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 4);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 3);
  EXPECT_EQ(graph.links(forest.mobods(MobodIndex(4)).link_ordinal()).index(),
            LinkIndex(7));

  // Changing both 3 and 4 to massless breaks the loop at 6 instead of 4.
  // (Tests Case 3 in ExtendTreesOneLevel())
  graph.ChangeLinkFlags(LinkIndex(4), LinkFlags::kMassless);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(graph.links()), 8);
  EXPECT_EQ(ssize(graph.joints()), 7);
  EXPECT_TRUE(graph.link_by_index(LinkIndex(7)).is_shadow());
  EXPECT_EQ(graph.link_by_index(LinkIndex(7)).primary_link(), LinkIndex(6));

  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees()[0].num_mobods(), 5);
  EXPECT_EQ(forest.trees()[1].num_mobods(), 2);
  EXPECT_EQ(graph.links(forest.mobods(MobodIndex(5)).link_ordinal()).index(),
            LinkIndex(7));
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

  graph.AddLink("massless_1", model_instance, LinkFlags::kMassless);
  graph.AddLink("massless_2", model_instance, LinkFlags::kMassless);
  graph.AddLink("link_3", model_instance);

  graph.AddJoint("prismatic_0", model_instance, "prismatic", world_index(),
                 LinkIndex(1));
  graph.AddJoint("prismatic_1", model_instance, "prismatic", world_index(),
                 LinkIndex(2));
  graph.AddJoint("revolute_2", model_instance, "revolute", LinkIndex(1),
                 LinkIndex(3));
  graph.AddJoint("revolute_3", model_instance, "revolute", LinkIndex(2),
                 LinkIndex(3));

  EXPECT_EQ(ssize(graph.links()), 4);  // Before modeling (includes World).
  EXPECT_EQ(graph.num_user_links(), 4);

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(graph.links()), 5);  // After modeling.
  EXPECT_EQ(graph.num_user_links(), 4);
  EXPECT_EQ(ssize(graph.loop_constraints()), 1);

  const auto& shadow_link = graph.link_by_index(LinkIndex(4));
  EXPECT_TRUE(shadow_link.is_shadow());
  EXPECT_EQ(shadow_link.primary_link(), LinkIndex(3));
  EXPECT_EQ(shadow_link.mobod_index(), MobodIndex(4));
  EXPECT_EQ(shadow_link.inboard_joint_index(), JointIndex(3));
  EXPECT_EQ(ssize(shadow_link.joints()), 1);
  EXPECT_TRUE(shadow_link.joints_as_parent().empty());
  EXPECT_EQ(shadow_link.joints_as_child()[0], JointIndex(3));
  EXPECT_EQ(shadow_link.joints()[0], JointIndex(3));

  EXPECT_EQ(graph.link_by_index(LinkIndex(3)).num_shadows(), 1);
  EXPECT_EQ(graph.link_by_index(LinkIndex(2)).num_shadows(), 0);
  EXPECT_EQ(graph.link_by_index(LinkIndex(4)).num_shadows(), 0);

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
  - Ephemeral Joint 8 is the added floating joint to World.
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

  const std::vector<pair<int, int>> joints{{1, 2}, {1, 4}, {1, 3}, {2, 5},
                                           {4, 7}, {3, 6}, {5, 6}, {7, 6}};
  for (int i = 0; i < 8; ++i) {
    graph.AddJoint("joint" + std::to_string(i), model_instance, "revolute",
                   LinkIndex(joints[i].first), LinkIndex(joints[i].second));
  }

  EXPECT_EQ(ssize(graph.links()), 8);  // Before modeling (includes World).
  EXPECT_EQ(ssize(graph.joints()), 8);
  EXPECT_EQ(ssize(graph.loop_constraints()), 0);

  EXPECT_TRUE(graph.BuildForest());  // Using default options.
  const SpanningForest& forest = graph.forest();
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(graph.links()), 10);  // After modeling.
  EXPECT_EQ(ssize(graph.joints()), 9);
  EXPECT_EQ(ssize(graph.loop_constraints()), 2);
  EXPECT_EQ(graph.num_user_links(), 8);
  EXPECT_EQ(graph.num_user_joints(), 8);

  const LinkJointGraph::Link& primary_link = graph.link_by_index(LinkIndex(6));
  const LinkJointGraph::Link& shadow_link_1 = graph.link_by_index(LinkIndex(8));
  const LinkJointGraph::Link& shadow_link_2 = graph.link_by_index(LinkIndex(9));

  EXPECT_EQ(primary_link.name(), "link6");
  EXPECT_EQ(shadow_link_1.name(), "link6$1");
  EXPECT_EQ(shadow_link_2.name(), "link6$2");

  EXPECT_EQ(primary_link.num_shadows(), 2);
  EXPECT_TRUE(shadow_link_1.is_shadow());
  EXPECT_TRUE(shadow_link_2.is_shadow());
  EXPECT_EQ(primary_link.shadow_links(),
            (std::vector{shadow_link_1.index(), shadow_link_2.index()}));
  EXPECT_EQ(primary_link.joints(),
            (std::vector{JointIndex(5), JointIndex(6), JointIndex(7)}));
  EXPECT_FALSE(primary_link.JointHasMovedToShadowLink(JointIndex(5)));
  EXPECT_TRUE(primary_link.JointHasMovedToShadowLink(JointIndex(6)));
  EXPECT_TRUE(primary_link.JointHasMovedToShadowLink(JointIndex(7)));

  EXPECT_EQ(graph.link_by_index(LinkIndex(5)).num_shadows(), 0);
  EXPECT_EQ(graph.link_by_index(LinkIndex(7)).num_shadows(), 0);

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
              LinkIndex(mobod2link[mobod.index()]));
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
  EXPECT_EQ(loop0.primary_link(), LinkIndex(6));
  EXPECT_EQ(loop0.shadow_link(), LinkIndex(8));
  EXPECT_EQ(loop1.primary_link(), LinkIndex(6));
  EXPECT_EQ(loop1.shadow_link(), LinkIndex(9));

  // Added loop constraints should be named the same as their shadow Link.
  EXPECT_EQ(loop0.name(), shadow_link_1.name());
  EXPECT_EQ(loop1.name(), shadow_link_2.name());
}

/* For both WeldedLinksAssemblies and WeldedMobods groups: the World assembly or
group must come first (even if nothing is welded to World). This graph's first
branch has an assembly that could be seen prior to the weld to World. We'll
attempt to trick it into following that path by using a massless body, requiring
it to extend the first branch to Link {2} before moving on to the next branch.
But we want to see the {0,3} assembly before the {1,2} assembly.

          +---> {1*} ===> {2}
      {0} | 0         1                {Links} & Joints
    World |                            ===> is a weld joint
          +===> {3}                    * Link 1 is massless
          | 2
          |
          +---> {4}
            3
*/
GTEST_TEST(SpanningForest, WorldAssemblyComesFirst) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  const ModelInstanceIndex model_instance(5);  // arbitrary

  graph.AddLink("massless_link_1", model_instance, LinkFlags::kMassless);
  graph.AddLink("link2", model_instance);
  graph.AddLink("link3", model_instance);
  graph.AddLink("link4", model_instance);

  const auto& world = graph.link_by_index(LinkIndex(0));
  const auto& massless_link = graph.link_by_index(LinkIndex(1));
  const auto& link2 = graph.link_by_index(LinkIndex(2));
  const auto& link3 = graph.link_by_index(LinkIndex(3));
  const auto& link4 = graph.link_by_index(LinkIndex(4));

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
  EXPECT_NO_THROW(forest.SanityCheckForest());

  EXPECT_EQ(ssize(graph.links()), 5);
  EXPECT_EQ(ssize(forest.mobods()), 5);  // Because we're not merging.

  // "Anchored" means "fixed to World" (by welds).
  EXPECT_TRUE(world.is_anchored());
  EXPECT_FALSE(massless_link.is_anchored());
  EXPECT_FALSE(link2.is_anchored());
  EXPECT_TRUE(link3.is_anchored());
  EXPECT_FALSE(link4.is_anchored());

  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 2);
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
            (std::vector<LinkIndex>{LinkIndex(0), LinkIndex(3)}));
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            std::vector<JointIndex>{JointIndex(2)});
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).is_massless());
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links(),
            (std::vector<LinkIndex>{LinkIndex(1), LinkIndex(2)}));
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints(),
            std::vector<JointIndex>{JointIndex(1)});
  EXPECT_FALSE(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).is_massless());

  EXPECT_EQ(ssize(forest.welded_mobods()), 2);
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(0)),
            (std::vector<MobodIndex>{MobodIndex(0), MobodIndex(3)}));
  EXPECT_EQ(forest.welded_mobods(WeldedMobodsIndex(1)),
            (std::vector<MobodIndex>{MobodIndex(1), MobodIndex(2)}));

  // Remodel making single Mobods for WeldedLinksAssemblies (optimizing).
  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());
  EXPECT_EQ(ssize(forest.mobods()), 3);  // Because we're merging.
  EXPECT_EQ(forest.mobods(MobodIndex(0)).follower_link_ordinals(),
            (std::vector<LinkOrdinal>{LinkOrdinal(0), LinkOrdinal(3)}));
  EXPECT_EQ(forest.mobods(MobodIndex(1)).follower_link_ordinals(),
            (std::vector<LinkOrdinal>{LinkOrdinal(1), LinkOrdinal(2)}));
  EXPECT_EQ(forest.mobods(MobodIndex(2)).follower_link_ordinals(),
            (std::vector<LinkOrdinal>{LinkOrdinal(4)}));

  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 2);  // no change expected
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);           // just World now
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
  const std::vector<pair<int, int>> joints{{0, 1}, {0, 2}, {1, 3}, {3, 2}};
  for (int i = 0; i < ssize(joints); ++i) {
    const auto joint = joints[i];
    graph.AddJoint("joint" + std::to_string(i), default_model_instance(),
                   "revolute", LinkIndex(joint.first), LinkIndex(joint.second));
  }

  EXPECT_TRUE(graph.BuildForest());

  const SpanningForest& forest = graph.forest();
  EXPECT_EQ(graph.num_user_joints(), 4);
  EXPECT_EQ(ssize(graph.joints()), 4);
  EXPECT_EQ(graph.num_user_links(), 4);
  EXPECT_EQ(ssize(graph.links()), 5);  // Added a shadow.

  // See right-hand graph above. We're expecting to split link 3 since that
  // will produce equal-length branches.
  const LinkJointGraph::Link& primary_link = graph.link_by_index(LinkIndex(3));
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

  const LinkJointGraph::Link& shadow_link = graph.link_by_index(LinkIndex(4));
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
  graph.ChangeLinkFlags(LinkIndex(3), LinkFlags::kMassless);
  EXPECT_TRUE(graph.BuildForest());
  const LinkJointGraph::Link& new_primary_link =
      graph.link_by_index(LinkIndex(2));
  const LinkJointGraph::Link& new_shadow_link =
      graph.link_by_index(LinkIndex(4));

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

/* Here are some rare conditions we want to process correctly.

A loop of massless links should be flagged as "no dynamics".

           +---->{1*}------,
           |               V
      {4*} |              {2*}
           |               |
           +---->{3*}<-----'

We should continue extending the forest along the branch {4}->{1} until we
get back to {4} and realize we're forced to connect massless links {4} and {3}.

But if we change link {4} to massful, that same procedure should rescue dynamics
since we can end both branches with half of link {4}.

We'll also try replacing massful link 4 with World and verify that still
works the same way. We can split off an arbitrary-mass chunk of World to
terminate the massless branch. */
GTEST_TEST(SpanningForest, MasslessLoopAreDetected) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  for (int i = 1; i <= 4; ++i) {
    graph.AddLink("link" + std::to_string(i), default_model_instance(),
                  LinkFlags::kMassless);
  }

  const std::vector<pair<int, int>> joints{{4, 1}, {4, 3}, {1, 2}, {2, 3}};
  for (int i = 0; i < ssize(joints); ++i) {
    graph.AddJoint("joint" + std::to_string(i), default_model_instance(),
                   "revolute", LinkIndex(joints[i].first),
                   LinkIndex(joints[i].second));
  }

  EXPECT_FALSE(graph.BuildForest());
  EXPECT_THAT(
      graph.forest().why_no_dynamics(),
      testing::MatchesRegex("Loop breaks.*joint1.*between two massless links.*"
                            "link4.*link3.*cannot be used for dynamics.*"));

  graph.ChangeLinkFlags(LinkIndex(1), LinkFlags::kDefault);  // Massful now.
  EXPECT_TRUE(graph.BuildForest());

  /* Make a new graph where World replaces body 4.
         +---->{1*}------,
         |               V
   World |              {2*}
    {0}  |               |
         +---->{3*}<-----'
  */
  LinkJointGraph world_graph;
  world_graph.RegisterJointType("revolute", 1, 1);
  for (int i = 1; i <= 3; ++i) {
    world_graph.AddLink("link" + std::to_string(i), default_model_instance(),
                        LinkFlags::kMassless);
  }

  const std::vector<pair<int, int>> world_graph_joints{
      {0, 1}, {0, 3}, {1, 2}, {2, 3}};
  for (int i = 0; i < ssize(world_graph_joints); ++i) {
    world_graph.AddJoint("joint" + std::to_string(i), default_model_instance(),
                         "revolute", LinkIndex(world_graph_joints[i].first),
                         LinkIndex(world_graph_joints[i].second));
  }

  /* Check that we split World as expected. */
  EXPECT_TRUE(world_graph.BuildForest());
  EXPECT_EQ(world_graph.num_user_links(), 4);
  EXPECT_EQ(ssize(world_graph.links()), 5);
  EXPECT_EQ(ssize(world_graph.forest().mobods()), 5);
  EXPECT_EQ(world_graph.world_link().num_shadows(), 1);
  EXPECT_TRUE(world_graph.link_by_index(LinkIndex(4)).is_shadow());
  EXPECT_EQ(world_graph.link_by_index(LinkIndex(4)).primary_link(),
            LinkIndex(0));
}

/* WeldedLinksAssemblies should be treated the same as single bodies while
building the trees a level at a time. We'll create a loop out of two
trees, one composed of two-body assemblies and the other single bodies.
Our loop-splitting algorithm should result in two trees of equal length in
mobilized bodies though unequal in links.

              0               1             2
      +--> {1}==>{2} --> {3*}==>{4*} --> {5}==>{6}
  {0} | 3             4              5          | 10   {Links} & Joints
      |                                         v      * = massless
      +--->   {7}  --->  {8}  --->  {9}  ---> {10}
        6           7          8          9


      +---> [1] ---> [2*] --> [3] ---> [4] {10}
      |    {1,2}    {3,4}*   {5,6}      #
  [0] |                                 # Weld            [Mobods]
      |     {7}      {8}      {9}       V                 * = massless
      +---> [5] ---> [6] ---> [7] ---> [8] {10s}

Not that the presence of massless links {3} and {4} should have almost no
consequence since they are followed by a massful body. However, the {3,4}
assembly should be marked "massless".

This test case also opportunistically uses this graph to test that copy, move,
and assign work correctly. */
GTEST_TEST(SpanningForest, LoopWithAssemblies) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  const ModelInstanceIndex model_instance(19);

  const std::set<int> massless{3, 4};
  for (int i = 1; i <= 10; ++i) {
    graph.AddLink(
        "link" + std::to_string(i), model_instance,
        massless.contains(i) ? LinkFlags::kMassless : LinkFlags::kDefault);
  }

  const std::vector<pair<int, int>> weld_joints{{1, 2}, {3, 4}, {5, 6}};
  const std::vector<pair<int, int>> revolute_joints{
      {0, 1}, {2, 3}, {4, 5}, {0, 7}, {7, 8}, {8, 9}, {9, 10}, {6, 10}};
  for (int i = 0; i < ssize(weld_joints); ++i) {
    graph.AddJoint("weld_joint_" + std::to_string(i), model_instance, "weld",
                   LinkIndex(weld_joints[i].first),
                   LinkIndex(weld_joints[i].second));
  }
  for (int i = 0; i < ssize(revolute_joints); ++i) {
    const int j = ssize(weld_joints) + i;  // joint number
    graph.AddJoint("revolute_joint_" + std::to_string(j), model_instance,
                   "revolute", LinkIndex(revolute_joints[i].first),
                   LinkIndex(revolute_joints[i].second));
  }

  // Before modeling
  EXPECT_EQ(ssize(graph.links()), 11);  // counting World
  EXPECT_EQ(ssize(graph.joints()), 11);
  EXPECT_EQ(ssize(graph.loop_constraints()), 0);

  EXPECT_TRUE(graph.BuildForest());

  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  EXPECT_TRUE(graph.BuildForest());
  const SpanningForest& forest = graph.forest();
  EXPECT_NO_THROW(forest.SanityCheckForest());

  // After modeling
  EXPECT_EQ(ssize(graph.links()), 12);            // split one, added shadow
  EXPECT_EQ(ssize(graph.joints()), 11);           // no change
  EXPECT_EQ(ssize(graph.loop_constraints()), 1);  // welded shadow to primary
  EXPECT_EQ(ssize(graph.welded_links_assemblies()), 4);  // World + 3
  std::array<bool, 4> expect_massless{false, false, true, false};
  for (WeldedLinksAssemblyIndex i{0}; i < 4; ++i) {
    EXPECT_EQ(graph.welded_links_assemblies(i).is_massless(),
              expect_massless[i]);
  }

  EXPECT_EQ(ssize(forest.mobods()), 9);
  EXPECT_EQ(ssize(forest.loop_constraints()), 1);
  EXPECT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(ssize(forest.welded_mobods()), 1);  // just World

  EXPECT_EQ(forest.mobods(MobodIndex(1)).follower_link_ordinals(),
            (std::vector<LinkOrdinal>{LinkOrdinal(1), LinkOrdinal(2)}));
  EXPECT_EQ(forest.mobods(MobodIndex(2)).follower_link_ordinals(),
            (std::vector<LinkOrdinal>{LinkOrdinal(3), LinkOrdinal(4)}));
  EXPECT_EQ(forest.mobods(MobodIndex(3)).follower_link_ordinals(),
            (std::vector<LinkOrdinal>{LinkOrdinal(5), LinkOrdinal(6)}));

  const SpanningForest::Tree tree0 = forest.trees(TreeIndex(0)),
                             tree1 = forest.trees(TreeIndex(1));
  EXPECT_EQ(tree0.num_mobods(), 4);
  EXPECT_EQ(tree0.nq(), 4);
  EXPECT_EQ(tree1.num_mobods(), 4);
  EXPECT_EQ(tree1.nq(), 4);

  // Sanity checks for graph copy, move, and assignment. These are mostly
  // compiler-generated so we just need to test any field and that the
  // bespoke backpointer adjustments get done correctly.

  LinkJointGraph graph_copy(graph);
  EXPECT_EQ(ssize(graph_copy.links()), 12);
  EXPECT_TRUE(graph_copy.forest_is_valid());
  const SpanningForest& copy_model = graph_copy.forest();
  EXPECT_NO_THROW(copy_model.SanityCheckForest());
  EXPECT_NE(&copy_model, &forest);
  EXPECT_EQ(&copy_model.graph(), &graph_copy);  // backpointer

  LinkJointGraph graph_assign;
  graph_assign = graph;
  EXPECT_EQ(ssize(graph_assign.links()), 12);
  EXPECT_TRUE(graph_assign.forest_is_valid());
  EXPECT_NE(&graph_assign.forest(), &forest);
  EXPECT_NO_THROW(graph_assign.forest().SanityCheckForest());
  EXPECT_EQ(&graph_assign.forest().graph(), &graph_assign);

  LinkJointGraph graph_move(std::move(graph));
  EXPECT_EQ(ssize(graph_move.links()), 12);
  EXPECT_EQ(ssize(graph.links()), 1);  // Just world now.
  EXPECT_EQ(&graph_move.forest(), &forest);
  EXPECT_NO_THROW(graph_move.forest().SanityCheckForest());
  EXPECT_EQ(&graph_move.forest().graph(), &graph_move);
  // graph is now default-constructed so still has a forest
  EXPECT_NE(&graph.forest(), &forest);
  EXPECT_FALSE(graph.forest_is_valid());
  EXPECT_EQ(&graph.forest().graph(), &graph);
  EXPECT_NO_THROW(graph.forest().SanityCheckForest());  // Empty but OK.

  LinkJointGraph graph_move_assign;
  graph_move_assign = std::move(graph_copy);
  EXPECT_EQ(ssize(graph_move_assign.links()), 12);
  EXPECT_TRUE(graph_move_assign.forest_is_valid());
  EXPECT_EQ(&graph_move_assign.forest(), &copy_model);
  EXPECT_NO_THROW(graph_move_assign.forest().SanityCheckForest());
  EXPECT_EQ(&graph_move_assign.forest().graph(), &graph_move_assign);
  // graph_copy is now default-constructed. Should have world and a
  // new (empty) forest.
  EXPECT_EQ(ssize(graph_copy.links()), 1);
  EXPECT_NE(&graph_copy.forest(), &copy_model);
  EXPECT_FALSE(graph_copy.forest_is_valid());
  EXPECT_EQ(&graph_copy.forest().graph(), &graph_copy);
  EXPECT_NO_THROW(graph_copy.forest().SanityCheckForest());  // Empty but OK.
}

/* Make sure massless, optimized assemblies are working correctly. They are
supposed to behave the same way as individual massless bodies:
  - They should use only a single Mobod, and be treated as a single level
    along a branch for branch-length minimization purposes.
  - If there is anything massful attached to the massless assembly, we
    assume that mass always moves with the assembly so we don't need to
    give up branch-length minimization. (Test 2 below)
  - We should not break a loop in a way that leaves a branch with a
    terminal massless assembly. (Test 3 below)
*/
GTEST_TEST(SpanningForest, MasslessOptimizedAssemblies) {
  LinkJointGraph graph;
  const SpanningForest& forest = graph.forest();
  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  graph.RegisterJointType("revolute", 1, 1);
  const ModelInstanceIndex model_instance(19);

  /* (Test 1) Massless assembly welded to World ends up on the World
  WeldedLinksAssembly and doesn't count as a level in its tree. All its links
  should be at level 0, and branch-length balancing should ignore the assembly.

  Input graph:
    {1} link, *=massless, joint numbers are plain
    --> revolute  ==> weld

           0        1        2
         +---> {1} ---> {2} ---> {3}     {4:5:6} is a massless assembly
     {0} |                        | 5            but welded to World
         | 6      7      3     4  v
         +==>{4*}==>{5*}-->{7}-->{8}     Should cut {3} to leave two
               ∥ 8                       branches of length 3:
               v                          {1 2 3} and {7 8 3s}
              {6*}

  Expected Forest:
    [1] mobod

           + --> [1]{1} --> [2]{2} --> [3]{3}
       [0] |                            # loop weld
  {0 4 5 6}|                            #
           + --> [4]{7} --> [5]{8} --> [6]{3s}
  */

  const std::set<int> massless{4, 5, 6};
  for (int i = 1; i <= 8; ++i) {
    graph.AddLink(
        "link" + std::to_string(i), model_instance,
        massless.contains(i) ? LinkFlags::kMassless : LinkFlags::kDefault);
  }

  const std::vector<pair<int, int>> revolute_joints{{0, 1}, {1, 2}, {2, 3},
                                                    {5, 7}, {7, 8}, {3, 8}};
  const std::vector<pair<int, int>> weld_joints{{0, 4}, {4, 5}, {4, 6}};

  for (int i = 0; i < ssize(revolute_joints); ++i) {
    graph.AddJoint("joint_" + std::to_string(i), model_instance, "revolute",
                   LinkIndex(revolute_joints[i].first),
                   LinkIndex(revolute_joints[i].second));
  }
  for (int i = 0; i < ssize(weld_joints); ++i) {
    const int j = ssize(revolute_joints) + i;  // joint number
    graph.AddJoint("joint_" + std::to_string(j), model_instance, "weld",
                   LinkIndex(weld_joints[i].first),
                   LinkIndex(weld_joints[i].second));
  }

  // Before modeling
  EXPECT_EQ(ssize(graph.links()), 9);  // counting World
  EXPECT_EQ(ssize(graph.joints()), 9);
  EXPECT_EQ(ssize(graph.loop_constraints()), 0);

  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  // After modeling
  EXPECT_EQ(ssize(graph.links()), 10);  // added shadow 3s {9}
  EXPECT_EQ(ssize(graph.joints()), 9);
  EXPECT_EQ(ssize(graph.loop_constraints()), 1);  // glue {3} back together
  EXPECT_EQ(ssize(forest.mobods()), 7);

  // The links are massless but part of the World assembly, which is not.
  for (LinkOrdinal link_ordinal(4); link_ordinal <= 6; ++link_ordinal)
    EXPECT_FALSE(graph.link_and_its_assembly_are_massless(link_ordinal));

  // Check that links not in an assembly still respond correctly.
  EXPECT_FALSE(graph.link_and_its_assembly_are_massless(LinkOrdinal(2)));

  // Check for equal-height trees.
  ASSERT_EQ(ssize(forest.trees()), 2);
  EXPECT_EQ(forest.trees(TreeIndex(0)).height(), 3);
  EXPECT_EQ(forest.trees(TreeIndex(1)).height(), 3);

  const auto& shadow_link = graph.link_by_index(LinkIndex(9));
  EXPECT_TRUE(shadow_link.is_shadow());
  EXPECT_EQ(shadow_link.name(), "link3$1");
  EXPECT_EQ(shadow_link.index(), LinkIndex(9));
  EXPECT_EQ(shadow_link.primary_link(), LinkIndex(3));
  EXPECT_EQ(shadow_link.mobod_index(), MobodIndex(6));
  EXPECT_EQ(shadow_link.joints(), std::vector{JointIndex(5)});

  ASSERT_EQ(ssize(graph.welded_links_assemblies()),
            1);  // just the world assembly
  EXPECT_EQ(
      graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).links(),
      (std::vector{LinkIndex(0), LinkIndex(4), LinkIndex(5), LinkIndex(6)}));
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(0)).joints(),
            (std::vector{JointIndex(6), JointIndex(7), JointIndex(8)}));

  /* (Test 2) Change the type of joint 6 (connects {4} to World) from weld
  to revolute. That should move massless assembly {4:5:6} onto its own Mobod.
  Since it is immediately followed by massful link {7}, it won't prevent
  the Forest from being suited for dynamics. This should cause the loop to be
  split at {8} now, resulting in Tree 0 having a height of 4 and Tree 1 a
  height of 3. */
  graph.ChangeJointType(JointIndex(6), "revolute");
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());

  // The links are massless and so is their assembly.
  for (LinkOrdinal link_ordinal(4); link_ordinal <= 6; ++link_ordinal)
    EXPECT_TRUE(graph.link_and_its_assembly_are_massless(link_ordinal));

  EXPECT_EQ(forest.trees(TreeIndex(0)).height(), 4);
  EXPECT_EQ(forest.trees(TreeIndex(1)).height(), 3);
  const auto& new_shadow_link = graph.link_by_index(LinkIndex(9));
  EXPECT_TRUE(new_shadow_link.is_shadow());
  EXPECT_EQ(new_shadow_link.name(), "link8$1");

  for (LinkIndex i(4); i <= 6; ++i)
    EXPECT_EQ(graph.link_by_index(i).mobod_index(), 5);  // Merged to one Mobod.

  ASSERT_EQ(ssize(graph.welded_links_assemblies()), 2);  // world and {4:5:6}
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).links(),
            (std::vector{LinkIndex(4), LinkIndex(5), LinkIndex(6)}));
  EXPECT_EQ(graph.welded_links_assemblies(WeldedLinksAssemblyIndex(1)).joints(),
            (std::vector{JointIndex(7), JointIndex(8)}));

  /* (Test 3) Change links 7 and 8 to be massless so that we have to continue
  extending the branch after the massless assembly to hunt down something
  massful with which to end the branch in Tree 1. This should affect when we see
  the loop so Tree 0 will have height 3 and Tree 1 height 4, with link 3
  split. */
  graph.ChangeLinkFlags(LinkIndex(7), LinkFlags::kMassless);
  graph.ChangeLinkFlags(LinkIndex(8), LinkFlags::kMassless);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_NO_THROW(forest.SanityCheckForest());
  const auto& newer_shadow_link = graph.link_by_index(LinkIndex(9));
  EXPECT_TRUE(newer_shadow_link.is_shadow());
  EXPECT_EQ(newer_shadow_link.name(), "link3$1");

  EXPECT_EQ(forest.trees(TreeIndex(0)).height(), 3);
  EXPECT_EQ(forest.trees(TreeIndex(1)).height(), 4);
}

/* A joint connects a parent link to a child link. In general, the joint,
parent, and child can each be in different model instances. Each of those
model instances can specify whether we should optimize link assemblies to use
a single Mobod or whether parent and child must each have their own Mobod.
Our policy is that when a joint is a weld, we look only at that joint's
model instance to determine whether that joint will optimize parent and child
onto a single Mobod.

A few nuances:
 - When an ephemeral joint is added, it is assigned the same model instance
   as its child link. So in that case the choice is effectively determined by
   the child's model instance. This affects static links since those are
   attached to World with a weld and will end up in the World
   WeldedLinksAssembly.
 - There is a joint flag that should force the joint to be modeled (rather
   than merged) regardless of its model instance's setting.
 - If the joint's model instance has no specified forest building options,
   it inherits them from the global forest building options. (That's tested
   elsewhere.)

We'll use this graph:

         I4  I1  I5  I2              Ix model instance index x
   World --> {1} ==> {2}
                                     I3 is a static model instance
             {3} I3                     so link {3} will be welded to World

Each link is in the model instance whose index is the same as the link
number. The joints are in I4 and I5 as shown. We'll fiddle with the forest
building options and check the behavior.
*/
GTEST_TEST(SpanningForest, CheckMergingPolicy) {
  LinkJointGraph graph;
  const SpanningForest& forest = graph.forest();
  graph.RegisterJointType("revolute", 1, 1);
  for (int i = 1; i <= 3; ++i)
    graph.AddLink("link" + std::to_string(i), ModelInstanceIndex(i));
  graph.AddJoint("revolute_0", ModelInstanceIndex(4), "revolute", LinkIndex(0),
                 LinkIndex(1));
  graph.AddJoint("weld_1", ModelInstanceIndex(5), "weld", LinkIndex(1),
                 LinkIndex(2));

  graph.SetForestBuildingOptions(ModelInstanceIndex(3),
                                 ForestBuildingOptions::kStatic);

  // Baseline -- no merging. Every link gets a Mobod.
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_EQ(ssize(forest.mobods()), 4);

  // Only I5 should determine whether we merge {1} and {2}. Set the merge
  // flag on everything else and verify it makes no difference.
  graph.SetForestBuildingOptions(
      ModelInstanceIndex(1),
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  graph.SetForestBuildingOptions(
      ModelInstanceIndex(2),
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  graph.SetForestBuildingOptions(
      ModelInstanceIndex(4),
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_EQ(ssize(forest.mobods()), 4);

  // If I5 says merge, we should have one fewer Mobod.
  graph.SetForestBuildingOptions(
      ModelInstanceIndex(5),
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_EQ(ssize(forest.mobods()), 3);

  // We're still getting a Mobod for the static link {3}. Let's merge that
  // with World now.
  graph.SetForestBuildingOptions(
      ModelInstanceIndex(3),
      ForestBuildingOptions::kOptimizeWeldedLinksAssemblies |
          ForestBuildingOptions::kStatic);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_EQ(ssize(forest.mobods()), 2);

  // Finally, let's insist that the weld joint gets modeled. That should
  // override the setting in its model instance and give us a separate
  // Mobod for links 1 and 2.
  graph.ChangeJointFlags(JointIndex(1), JointFlags::kMustBeModeled);
  EXPECT_TRUE(graph.BuildForest());
  EXPECT_EQ(ssize(forest.mobods()), 3);
}

/* Check just some basic functioning of the Graphviz utilities for visualizing
the graph and forest. We'll test that we get the expected dot strings for
a very simple graph, but won't attempt to conjure up complete dot
strings for every visualization quirk. Those will have to be validated by a
human looking at the results. We'll also verify that the expected .png files
are created, but won't look at their contents. */
GTEST_TEST(SpanningForest, VisualizationWithGraphviz) {
  const std::string tmpdir = temp_directory();
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  const SpanningForest& forest = graph.forest();

  // Input graph:
  //   {0} {1} -> {2}
  // Will get an ephemeral floating joint between {0} and {1}.
  const LinkIndex link1 = graph.AddLink("link1", default_model_instance());
  const LinkIndex link2 = graph.AddLink("link2", default_model_instance());
  graph.AddJoint("joint0", default_model_instance(), "revolute", link1, link2);
  EXPECT_TRUE(graph.BuildForest());

  // This is what we get if only the user elements are to be shown.
  const char expected_graph[] = R"""(digraph LinkJointGraph {
rankdir=BT;
labelloc=t;
label="GraphvizTest
LinkJointGraph";
legend [shape=none]
[label="* = massless
L/J(i) link/joint(ordinal)
name:index"]
subgraph cluster0 {
label="WeldedLinksAssembly(0)";
link0 [label="L(0) world:0"];
}
link1 [label="L(1) link1:1"];
link2 [label="L(2) link2:2"];
link1 -> link2 [arrowhead=normal] [style=solid] [fontsize=10] [label="J(0) joint0:0
revolute"] [color=green];
}
)""";

  // This is what we should get if ephemeral elements are shown (the
  // default behavior).
  const char expected_augmented_graph[] = R"""(digraph LinkJointGraph {
rankdir=BT;
labelloc=t;
label="GraphvizTest
LinkJointGraph+";
legend [shape=none]
[label="* = massless
L/J(i) link/joint(ordinal)
name:index
red = ephemeral"]
subgraph cluster0 {
label="WeldedLinksAssembly(0)";
link0 [label="L(0) world:0"];
}
link1 [label="L(1) link1:1"];
link2 [label="L(2) link2:2"];
link1 -> link2 [arrowhead=normal] [style=solid] [fontsize=10] [label="J(0) joint0:0
revolute"] [color=green];
link0 -> link1 [arrowhead=empty] [style=solid] [fontsize=10] [label="J(1) link1:1
quaternion_floating"] [color=red];
}
)""";

  const char expected_forest[] = R"""(digraph SpanningForest {
rankdir=BT;
labelloc=t;
label="GraphvizTest
SpanningForest";
legend [shape=none]
[label="* = massless
red = shadow
purple = reversed"]
mobod0 [color=black] [label="mobod(0)
L(0) "];
mobod1 [color=black] [label="mobod(1)
L(1) "];
mobod0 -> mobod1 [arrowhead=normal] [fontsize=10] [style=solid][label="mobilizer(1)
J(1) quaternion_floating
q0 v0"] [color=blue];
mobod2 [color=black] [label="mobod(2)
L(2) "];
mobod1 -> mobod2 [arrowhead=normal] [fontsize=10] [style=solid][label="mobilizer(2)
J(0) revolute
q7 v6"] [color=blue];
}
)""";

  EXPECT_EQ(graph.GenerateGraphvizString("GraphvizTest", false),
            expected_graph);
  EXPECT_EQ(graph.GenerateGraphvizString("GraphvizTest"),
            expected_augmented_graph);
  EXPECT_EQ(forest.GenerateGraphvizString("GraphvizTest"), expected_forest);

  // Returns true if file {tmpdir}/GraphvizTest_{suffix}.{extension} exists.
  auto exists = [&tmpdir](const char* suffix, const char* extension) {
    std::filesystem::path name =
        std::filesystem::path(tmpdir).append("GraphvizTest_");
    name.concat(suffix);
    name.replace_extension(extension);
    return std::filesystem::exists(name);
  };

  // Make sure we don't already have the .png files.
  for (const char* suffix : {"graph", "graph+", "forest"})
    EXPECT_FALSE(exists(suffix, "png"));

  DRAKE_EXPECT_THROWS_MESSAGE(
      graph.MakeGraphvizFiles("////bad_dir", "bad_file"),
      "MakeGraphvizFiles.*can't create.*bad_dir/bad_file.*");

  // We can't test running dot if it's not there. This is the documented
  // search path for MakeGraphvizFiles().
  if (!std::filesystem::exists("/usr/bin/dot") &&
      !std::filesystem::exists("/usr/local/bin/dot") &&
      !std::filesystem::exists("/opt/homebrew/bin/dot") &&
      !std::filesystem::exists("/bin/dot")) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        graph.MakeGraphvizFiles(tmpdir, "Anything"),
        "MakeGraphvizFiles.*dot.*required but missing.*");
  } else {
    const std::filesystem::path directory =
        graph.MakeGraphvizFiles(tmpdir, "GraphvizTest");

    EXPECT_EQ(directory.string(), std::filesystem::absolute(tmpdir).string());

    // Now check that (a) we have .pngs, and (b) we didn't leave any .dot files.
    for (const char* suffix : {"graph", "graph+", "forest"}) {
      EXPECT_TRUE(exists(suffix, "png"));
      EXPECT_FALSE(exists(suffix, "dot"));
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
