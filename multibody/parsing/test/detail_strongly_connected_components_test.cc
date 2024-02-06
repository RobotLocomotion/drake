#include "drake/multibody/parsing/detail_strongly_connected_components.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::UnorderedElementsAre;

GTEST_TEST(StronglyConnectedComponentsTest, Self) {
  // Self-edges are boring, but don't cause problems.
  DirectedGraph<int> graph;
  graph[0] = {0};
  auto sccs = FindStronglyConnectedComponents(graph);
  EXPECT_EQ(sccs.size(), 1);
}

GTEST_TEST(StronglyConnectedComponentsTest, Unrelated) {
  // Unrelated edges don't cause problems.
  DirectedGraph<int> graph;
  graph[0] = {1};
  graph[2] = {3};
  auto sccs = FindStronglyConnectedComponents(graph);
  EXPECT_EQ(sccs.size(), 4);
}

GTEST_TEST(StronglyConnectedComponentsTest, Arbitrary) {
  // Check the arbitrary graph shown in the Wikipedia animation linked below.
  // https://en.wikipedia.org/wiki/Tarjan%27s_strongly_connected_components_algorithm#/media/File:Tarjan's_Algorithm_Animation.gif
  DirectedGraph<int> graph;
  graph[1] = {2};
  graph[2] = {3};
  graph[3] = {1};

  graph[4] = {2, 3, 5};
  graph[5] = {4, 6};

  graph[6] = {3, 7};
  graph[7] = {6};

  graph[8] = {5, 8};

  auto sccs = FindStronglyConnectedComponents(graph);
  EXPECT_EQ(sccs.size(), 4);

  EXPECT_THAT(sccs[0], UnorderedElementsAre(1, 2, 3));
  EXPECT_THAT(sccs[1], UnorderedElementsAre(6, 7));
  EXPECT_THAT(sccs[2], UnorderedElementsAre(4, 5));
  EXPECT_THAT(sccs[3], UnorderedElementsAre(8));
}

GTEST_TEST(StronglyConnectedComponentsTest, Cycle) {
  // A large cycle produces 1 scc.
  const int length = 2'000;
  DirectedGraph<int> graph;
  for (int k = 0; k < length; ++k) {
    graph[k] = {(k + 1) % length};
  }
  auto sccs = FindStronglyConnectedComponents(graph);
  EXPECT_EQ(sccs.size(), 1);
}

GTEST_TEST(StronglyConnectedComponentsTest, DeepNest) {
  // A long nesting path produces N sccs.
  const int depth = 2'000;
  DirectedGraph<int> graph;
  for (int k = 0; k < depth - 1; ++k) {
    graph[k] = {k + 1};
  }
  auto sccs = FindStronglyConnectedComponents(graph);
  EXPECT_EQ(sccs.size(), depth);

  // Ordering is top-down; successors before sources.
  EXPECT_EQ(sccs[0].count(depth - 1), 1);
  EXPECT_EQ(sccs[depth - 1].count(0), 1);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
