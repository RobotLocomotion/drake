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

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
