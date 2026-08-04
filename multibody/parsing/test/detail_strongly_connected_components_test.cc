#include "drake/multibody/parsing/detail_strongly_connected_components.h"

#include <unordered_set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::UnorderedElementsAre;

template <typename T>
std::unordered_set<T> DirectedGraphNodes(const DirectedGraph<T>& graph) {
  std::unordered_set<T> unique_nodes;
  for (const auto& item : graph) {
    unique_nodes.insert(item.first);
    unique_nodes.insert(item.second.begin(), item.second.end());
  }
  return unique_nodes;
}

template <typename T>
int DirectedGraphSize(const DirectedGraph<T>& graph) {
  return DirectedGraphNodes(graph).size();
}

template <typename T>
std::unordered_set<T> ComponentsNodes(
    const StronglyConnectedComponents<T>& components) {
  std::unordered_set<T> unique_nodes;
  for (const auto& component : components) {
    unique_nodes.insert(component.begin(), component.end());
  }
  return unique_nodes;
}
template <typename T>
int SumComponentsSizes(const StronglyConnectedComponents<T>& components) {
  int result{0};
  for (const auto& component : components) {
    result += component.size();
  }
  return result;
}

template <typename T>
void CheckStronglyConnectedComponentsInvariants(
    const DirectedGraph<T>& graph,
    const StronglyConnectedComponents<T>& components) {
  // Input and output use the same set of nodes.
  auto graph_nodes = DirectedGraphNodes(graph);
  EXPECT_EQ(graph_nodes, ComponentsNodes(components));

  // The output partitions the graph.
  EXPECT_EQ(graph_nodes.size(), SumComponentsSizes(components));

  // Components are non-empty.
  for (const auto& component : components) {
    EXPECT_FALSE(component.empty());
  }
}

GTEST_TEST(StronglyConnectedComponentsTest, Empty) {
  // Empty graphs are boring, but don't cause problems.
  DirectedGraph<int> graph;
  auto sccs = FindStronglyConnectedComponents(graph);
  CheckStronglyConnectedComponentsInvariants(graph, sccs);
  EXPECT_EQ(sccs.size(), 0);
}

GTEST_TEST(StronglyConnectedComponentsTest, Self) {
  // Self-edges are boring, but don't cause problems.
  DirectedGraph<int> graph;
  graph[0] = {0};
  auto sccs = FindStronglyConnectedComponents(graph);
  CheckStronglyConnectedComponentsInvariants(graph, sccs);
  EXPECT_EQ(sccs.size(), 1);
}

GTEST_TEST(StronglyConnectedComponentsTest, Unrelated) {
  // Unrelated edges don't cause problems.
  DirectedGraph<int> graph;
  graph[0] = {1};
  graph[2] = {3};
  auto sccs = FindStronglyConnectedComponents(graph);
  CheckStronglyConnectedComponentsInvariants(graph, sccs);
  EXPECT_EQ(sccs.size(), DirectedGraphSize<int>(graph));
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
  CheckStronglyConnectedComponentsInvariants(graph, sccs);
  EXPECT_EQ(sccs.size(), 4);
  // We have cycles, so there must be fewer components than nodes in the input.
  EXPECT_LT(sccs.size(), DirectedGraphSize<int>(graph));

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
  CheckStronglyConnectedComponentsInvariants(graph, sccs);
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
  CheckStronglyConnectedComponentsInvariants(graph, sccs);
  EXPECT_EQ(sccs.size(), depth);

  // Ordering is top-down; successors before sources.
  EXPECT_TRUE(sccs[0].contains(depth - 1));
  EXPECT_TRUE(sccs[depth - 1].contains(0));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
