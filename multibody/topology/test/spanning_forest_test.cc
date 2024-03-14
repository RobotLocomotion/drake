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
// a single link composite and proper modeling info.
GTEST_TEST(SpanningForest, WorldOnlyTest) {
  LinkJointGraph graph;

  // World is predefined.
  EXPECT_EQ(ssize(graph.links()), 1);
  EXPECT_TRUE(graph.joints().empty());
  EXPECT_EQ(graph.world_link().name(), "world");
  EXPECT_EQ(graph.world_link().model_instance(), world_model_instance());
  const BodyIndex world_link_index = graph.world_link().index();
  EXPECT_EQ(world_link_index, BodyIndex(0));

  graph.BuildForest();
  EXPECT_TRUE(graph.forest_is_valid());
  const SpanningForest& forest = graph.forest();
  EXPECT_EQ(&forest.graph(), &graph);
  EXPECT_TRUE(forest.is_valid());
  EXPECT_EQ(graph.link_to_mobod(world_link_index), MobodIndex(0));
  EXPECT_EQ(ssize(graph.link_composites()), 1);
  EXPECT_EQ(ssize(graph.link_composites(LinkCompositeIndex(0))), 1);
  EXPECT_EQ(graph.link_composites(LinkCompositeIndex(0))[0], world_link_index);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
