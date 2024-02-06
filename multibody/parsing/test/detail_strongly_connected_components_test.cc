#include "drake/multibody/parsing/detail_strongly_connected_components.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::MatchesRegex;


GTEST_TEST(StronglyConnectedComponentsTest, Self) {
  // Self-edges are boring, but don't cause problems.
  SccGraph<int> graph;
  graph[0] = {0};
  auto sccs = strongly_connected_components(graph);
  EXPECT_EQ(sccs.size(), 1);
}


GTEST_TEST(StronglyConnectedComponentsTest, Cycle) {
  // A large cycle produces 1 scc.
  const int length = 2'000;
  SccGraph<int> graph;
  for (int k = 0; k < length; ++k) {
    graph[k] = {(k + 1) % length};
  }
  auto sccs = strongly_connected_components(graph);
  EXPECT_EQ(sccs.size(), 1);
}


GTEST_TEST(StronglyConnectedComponentsTest, DeepNest) {
  // A long nesting path produces N sccs.
  const int depth = 2'000;
  SccGraph<int> graph;
  for (int k = 0; k < depth - 1; ++k) {
    graph[k] = {k + 1};
  }
  auto sccs = strongly_connected_components(graph);
  EXPECT_EQ(sccs.size(), depth);
}


}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
