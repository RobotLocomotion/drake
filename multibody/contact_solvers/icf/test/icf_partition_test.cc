#include "drake/multibody/contact_solvers/icf/icf_partition.h"

#include <algorithm>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

using contact_solvers::internal::BlockSparsityPattern;

/* Builds a BlockSparsityPattern over `num_cliques` unit-sized cliques, with the
given undirected edges. The pattern requires lower-triangular adjacency lists
(neighbors[j] holds j plus every connected i > j). */
BlockSparsityPattern MakePattern(
    int num_cliques, const std::vector<std::pair<int, int>>& edges) {
  std::vector<int> block_sizes(num_cliques, 1);
  std::vector<std::vector<int>> neighbors(num_cliques);
  for (int i = 0; i < num_cliques; ++i) neighbors[i].push_back(i);
  for (const auto& [a, b] : edges) {
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    if (lo != hi) neighbors[lo].push_back(hi);
  }
  return BlockSparsityPattern(std::move(block_sizes), std::move(neighbors));
}

/* Collects island_cliques into a vector-of-vectors for easy comparison. */
std::vector<std::vector<int>> Islands(const IcfPartition& p) {
  std::vector<std::vector<int>> result(p.num_islands());
  for (int i = 0; i < p.num_islands(); ++i) {
    const auto cliques = p.island_cliques(i);
    result[i].assign(cliques.begin(), cliques.end());
  }
  return result;
}

GTEST_TEST(IcfPartitionTest, Empty) {
  IcfPartition p;
  p.Compute(MakePattern(0, {}));
  EXPECT_EQ(p.num_cliques(), 0);
  EXPECT_EQ(p.num_islands(), 0);
}

GTEST_TEST(IcfPartitionTest, AllSingletons) {
  // Four cliques with no coupling: four singleton islands.
  IcfPartition p;
  p.Compute(MakePattern(4, {}));
  EXPECT_EQ(p.num_islands(), 4);
  for (int c = 0; c < 4; ++c) EXPECT_EQ(p.clique_to_island(c), c);
  EXPECT_EQ(Islands(p), (std::vector<std::vector<int>>{{0}, {1}, {2}, {3}}));
}

GTEST_TEST(IcfPartitionTest, FullyCoupled) {
  // 0-1-2 chained into a single island.
  IcfPartition p;
  p.Compute(MakePattern(3, {{0, 1}, {1, 2}}));
  EXPECT_EQ(p.num_islands(), 1);
  EXPECT_EQ(Islands(p), (std::vector<std::vector<int>>{{0, 1, 2}}));
}

GTEST_TEST(IcfPartitionTest, ForcePathCompression) {
  // Single island, connected so as to force path compression in `Find()`.
  IcfPartition p;
  p.Compute(MakePattern(3, {{0, 2}, {1, 2}}));
  EXPECT_EQ(Islands(p), (std::vector<std::vector<int>>{{0, 1, 2}}));

  // Arbitrary large graph, connected so as to force multiple iterations of
  // path compression.
  p.Compute(BlockSparsityPattern(
      std::vector<int>(7, 1),
      std::vector<std::vector<int>>{
          {0, 5, 6}, {1, 5}, {2, 5}, {3, 5}, {4, 5, 6}, {5}, {6}}));
  EXPECT_EQ(Islands(p), (std::vector<std::vector<int>>{{0, 1, 2, 3, 4, 5, 6}}));
}

GTEST_TEST(IcfPartitionTest, TwoDisjointPairs) {
  // {0,1} and {2,3} form two islands.
  IcfPartition p;
  p.Compute(MakePattern(4, {{0, 1}, {2, 3}}));
  EXPECT_EQ(p.num_islands(), 2);
  EXPECT_EQ(p.clique_to_island(0), 0);
  EXPECT_EQ(p.clique_to_island(1), 0);
  EXPECT_EQ(p.clique_to_island(2), 1);
  EXPECT_EQ(p.clique_to_island(3), 1);
  EXPECT_EQ(Islands(p), (std::vector<std::vector<int>>{{0, 1}, {2, 3}}));
}

GTEST_TEST(IcfPartitionTest, MixedAndDeterministicLabeling) {
  // Cliques: 0 couples to 3; 1 alone; 2 couples to 4 and 5. Islands are
  // labeled by their smallest clique index, so:
  //   island 0 = {0, 3}, island 1 = {1}, island 2 = {2, 4, 5}.
  IcfPartition p;
  p.Compute(MakePattern(6, {{3, 0}, {4, 2}, {2, 5}}));
  EXPECT_EQ(p.num_islands(), 3);
  EXPECT_EQ(Islands(p),
            (std::vector<std::vector<int>>{{0, 3}, {1}, {2, 4, 5}}));
  // Spot-check the inverse map.
  EXPECT_EQ(p.clique_to_island(3), 0);
  EXPECT_EQ(p.clique_to_island(1), 1);
  EXPECT_EQ(p.clique_to_island(5), 2);
}

GTEST_TEST(IcfPartitionTest, CliqueLocalIndex) {
  // island 0 = {0, 3}, island 1 = {1}, island 2 = {2, 4, 5}.
  IcfPartition p;
  p.Compute(MakePattern(6, {{3, 0}, {4, 2}, {2, 5}}));
  // Local index is the clique's position within its island's ascending list.
  EXPECT_EQ(p.clique_local_index(0), 0);
  EXPECT_EQ(p.clique_local_index(3), 1);
  EXPECT_EQ(p.clique_local_index(1), 0);
  EXPECT_EQ(p.clique_local_index(2), 0);
  EXPECT_EQ(p.clique_local_index(4), 1);
  EXPECT_EQ(p.clique_local_index(5), 2);
  const auto local = p.clique_local_index();
  EXPECT_EQ(std::vector<int>(local.begin(), local.end()),
            (std::vector<int>{0, 0, 0, 1, 1, 2}));
}

GTEST_TEST(IcfPartitionTest, MultiDofCliques) {
  // Island structure is independent of clique sizes.
  std::vector<int> block_sizes = {3, 1, 6, 2};
  std::vector<std::vector<int>> neighbors = {{0, 2}, {1}, {2}, {3}};
  BlockSparsityPattern pattern(std::move(block_sizes), std::move(neighbors));
  IcfPartition p;
  p.Compute(pattern);
  EXPECT_EQ(p.num_islands(), 3);
  EXPECT_EQ(Islands(p), (std::vector<std::vector<int>>{{0, 2}, {1}, {3}}));
}

GTEST_TEST(IcfPartitionTest, RecomputeReusesStorageGrowOnly) {
  IcfPartition p;
  // Warm up the buffers at the high-water mark for both clique count (6) and
  // island count (6, all singletons).
  p.Compute(MakePattern(6, {}));
  EXPECT_EQ(p.num_islands(), 6);

  // A subsequent Compute() on a problem within those bounds must not allocate,
  // even though the island structure is completely different. Build the pattern
  // outside the guard so we measure only Compute().
  const BlockSparsityPattern pattern = MakePattern(6, {{0, 1}, {1, 2}});
  {
    drake::test::LimitMalloc guard;
    p.Compute(pattern);
  }
  EXPECT_EQ(p.num_islands(), 4);  // {0,1,2}, {3}, {4}, {5}.
  EXPECT_EQ(Islands(p),
            (std::vector<std::vector<int>>{{0, 1, 2}, {3}, {4}, {5}}));
}

GTEST_TEST(IslandItemMapTest, GroupsAndSorts) {
  IslandItemMap map;
  // Five items assigned to three islands.
  const std::vector<int> island_of_item = {2, 0, 1, 0, 2};
  map.Build(3, island_of_item);
  EXPECT_EQ(map.num_islands(), 3);
  EXPECT_EQ(map.num_items(), 5);

  auto to_vec = [&](int i) {
    const auto items = map.items(i);
    return std::vector<int>(items.begin(), items.end());
  };
  EXPECT_EQ(to_vec(0), (std::vector<int>{1, 3}));
  EXPECT_EQ(to_vec(1), (std::vector<int>{2}));
  EXPECT_EQ(to_vec(2), (std::vector<int>{0, 4}));
}

GTEST_TEST(IslandItemMapTest, SkipsNegativeIslands) {
  IslandItemMap map;
  // Items 1 and 3 belong to no island (e.g., anchored bodies) and are omitted.
  const std::vector<int> island_of_item = {1, -1, 0, -1, 0};
  map.Build(2, island_of_item);
  EXPECT_EQ(map.num_items(), 3);

  auto to_vec = [&](int i) {
    const auto items = map.items(i);
    return std::vector<int>(items.begin(), items.end());
  };
  EXPECT_EQ(to_vec(0), (std::vector<int>{2, 4}));
  EXPECT_EQ(to_vec(1), (std::vector<int>{0}));
}

GTEST_TEST(IslandItemMapTest, EmptyIslands) {
  IslandItemMap map;
  // Island 1 gets no items.
  map.Build(3, std::vector<int>{0, 2, 0});
  EXPECT_TRUE(map.items(1).empty());
  EXPECT_EQ(map.items(0).size(), 2);
  EXPECT_EQ(map.items(2).size(), 1);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
