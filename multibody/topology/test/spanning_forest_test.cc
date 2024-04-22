/* clang-format off to disable clang-format-includes */
#include "drake/multibody/topology/forest.h"
/* clang-format on */

#include <string>

#include <fmt/format.h>
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

  // Now build a forest representing the World-only graph.
  graph.BuildForest();
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
  EXPECT_EQ(forest.mobod_to_link(world_mobod_index), world_link_index);
  EXPECT_EQ(ssize(forest.mobod_to_links(world_mobod_index)), 1);
  EXPECT_EQ(forest.mobod_to_links(world_mobod_index)[0], world_link_index);
  EXPECT_EQ(forest.num_positions(), 0);
  EXPECT_EQ(forest.num_velocities(), 0);

  // Exercise the Mobod API to check the World Mobod for reasonableness.
  EXPECT_TRUE(world.is_world());
  EXPECT_FALSE(world.is_base_body());
  EXPECT_TRUE(world.is_anchored());
  EXPECT_TRUE(world.is_leaf_mobod());
  EXPECT_FALSE(world.is_reversed());  // Not meaningful though.
  EXPECT_TRUE(world.is_weld());       // Defined as having no inboard dofs.
  EXPECT_FALSE(world.inboard().is_valid());
  EXPECT_TRUE(world.outboards().empty());
  EXPECT_EQ(world.link(), world_link_index);
  EXPECT_EQ(ssize(world.follower_links()), 1);
  EXPECT_EQ(world.follower_links()[0], world_link_index);
  EXPECT_FALSE(world.joint().is_valid());
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
  graph.BuildForest();

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
    graph.AddJoint("joint_" + std::to_string(i), link_to_instance[joint.second],
                   "revolute", BodyIndex(joint.first), BodyIndex(joint.second));
  }

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
  graph.BuildForest();
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

  // Check Mobod->Link and Link->Mobod mappings.
  const std::vector<pair<int, int>> mobod_link_map{
      {0, 0}, {1, 1}, {2, 4},   {3, 5},   {4, 6},   {5, 9},   {6, 10}, {7, 12},
      {8, 3}, {9, 8}, {10, 11}, {11, 13}, {12, 14}, {13, 15}, {14, 7}, {15, 2}};
  for (auto mobod_link : mobod_link_map) {
    EXPECT_EQ(graph.link_to_mobod(BodyIndex(mobod_link.second)),
              MobodIndex(mobod_link.first));
    EXPECT_EQ(forest.mobod_to_link(MobodIndex(mobod_link.first)),
              BodyIndex(mobod_link.second));
    // Each Mobod has only a single Link that follows it.
    EXPECT_EQ(ssize(forest.mobod_to_links(MobodIndex(mobod_link.first))), 1);
    EXPECT_EQ(forest.mobod_to_links(MobodIndex(mobod_link.first))[0],
              BodyIndex(mobod_link.second));
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
  graph.BuildForest();
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

  EXPECT_FALSE(graph.links(tree0.front().link()).is_anchored());
  EXPECT_TRUE(graph.links(tree1.front().link()).is_anchored());
  EXPECT_FALSE(graph.links(tree2.front().link()).is_anchored());

  // There is only the World composite, but now tree1's base link is included.
  EXPECT_EQ(ssize(graph.link_composites()), 1);  // just World
  EXPECT_EQ(ssize(graph.link_composites(LinkCompositeIndex(0))), 2);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[0],
            graph.world_link().index());
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[1],
            tree1.front().link());

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
    graph.AddJoint("joint_" + std::to_string(i), default_model_instance(),
                   "revolute", BodyIndex(joint.first), BodyIndex(joint.second));
  }

  graph.BuildForest();
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

Links can become static either by being tagged with a static model instance,
or by having been specified with the Static link flag; we test both of those
here. Forest building should add a weld joint to World for every static link
unless there is already an explicit weld; we'll check that here.

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

  TODO(sherm1) Rebuild with CombineLinkComposites options and check the result.
*/
GTEST_TEST(SpanningForest, SerialChainAndMore) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);
  EXPECT_EQ(ssize(graph.joint_types()), 4);  // Built-ins plus "revolute".
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
  graph.AddLink("static8", model_instance, LinkFlags::kStatic);
  // Manually adding a weld to World is allowable for a static Link.
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
  graph.BuildForest();
  const SpanningForest& forest = graph.forest();

  // Should have added four ephemeral joints to the graph.
  EXPECT_EQ(graph.num_user_joints(), 7);
  EXPECT_EQ(ssize(graph.joints()), 11);
  const JointIndex base11_joint_index(9);  // See above picture.
  const JointIndex free9_joint_index(10);
  EXPECT_EQ(graph.joints(base11_joint_index).type_index(),
            LinkJointGraph::quaternion_floating_joint_type_index());
  EXPECT_EQ(graph.joints(free9_joint_index).type_index(),
            LinkJointGraph::quaternion_floating_joint_type_index());

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
      forest.mobods(graph.links(BodyIndex(3)).mobod_index());
  EXPECT_EQ(mobod_for_link3.link(), BodyIndex(3));
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
      forest.mobods(graph.links(BodyIndex(11)).mobod_index());
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

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
