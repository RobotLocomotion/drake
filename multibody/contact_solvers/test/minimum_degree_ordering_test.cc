#include "drake/multibody/contact_solvers/minimum_degree_ordering.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

GTEST_TEST(MinimumDegreeOrderingTest, Union) {
  const std::vector<int> a = {1, 2, 5, 8, 9};
  const std::vector<int> b = {1, 3, 5, 7, 9, 11};
  const std::vector<int> result = Union(a, b);
  const std::vector<int> expected = {1, 2, 3, 5, 7, 8, 9, 11};
  EXPECT_EQ(result, expected);
}

GTEST_TEST(MinimumDegreeOrderingTest, SetDifference) {
  const std::vector<int> a = {1, 2, 5, 8, 9};
  const std::vector<int> b = {1, 3, 5, 7, 9, 11};
  const std::vector<int> result = SetDifference(a, b);
  const std::vector<int> expected = {2, 8};
  EXPECT_EQ(result, expected);
}

GTEST_TEST(MinimumDegreeOrderingTest, RemoveValueFromSortedVector) {
  const std::vector<int> input = {1, 2, 5, 8, 9};
  std::vector<int> v = input;
  RemoveValueFromSortedVector(3, &v);
  EXPECT_EQ(input, v);
  RemoveValueFromSortedVector(2, &v);
  const std::vector<int> expected = {1, 5, 8, 9};
  EXPECT_EQ(v, expected);
}

GTEST_TEST(MinimumDegreeOrderingTest, InsertValueInSortedVector) {
  const std::vector<int> input = {1, 2, 5, 8, 9};
  std::vector<int> v = input;
  InsertValueInSortedVector(2, &v);
  EXPECT_EQ(input, v);
  InsertValueInSortedVector(3, &v);
  const std::vector<int> expected = {1, 2, 3, 5, 8, 9};
  EXPECT_EQ(v, expected);
}

/* This example is taken from G5 in figure 2 from [Amestoy 1996].
 The quotient graph is set up such that

       __ 5
     /   /  \
    9   7     6
    | \   \   |
    |  \    \ /
   10---8----4

  where nodes 4 and 5 are elements and all other nodes are variables. We call
  update external degree on all variables to verify the results are as expected.

  [Amestoy 1996] Amestoy, Patrick R., Timothy A. Davis, and Iain S. Duff. "An
 approximate minimum degree ordering algorithm." SIAM Journal on Matrix Analysis
 and Applications 17.4 (1996): 886-905. */
GTEST_TEST(MinimumDegreeOrderingTest, UpdateExternalDegree) {
  Node n4{
      .degree = 0, .size = 40, .index = 4, .A = {}, .E = {}, .L = {6, 7, 8}};
  Node n5{
      .degree = 0, .size = 50, .index = 5, .A = {}, .E = {}, .L = {6, 7, 9}};
  Node n6{.degree = 0, .size = 60, .index = 6, .A = {}, .E = {4, 5}, .L = {}};
  Node n7{.degree = 0, .size = 70, .index = 7, .A = {10}, .E = {4, 5}, .L = {}};
  Node n8{.degree = 0, .size = 80, .index = 8, .A = {9, 10}, .E = {4}, .L = {}};
  Node n9{.degree = 0, .size = 90, .index = 9, .A = {8, 10}, .E = {5}, .L = {}};
  Node n10{
      .degree = 0, .size = 100, .index = 10, .A = {7, 8, 9}, .E = {}, .L = {}};
  std::vector<Node> nodes;
  /* Nodes 0-3 are already eliminated. */
  for (int i = 0; i < 4; ++i) {
    nodes.emplace_back();
  }
  nodes.push_back(n4);
  nodes.push_back(n5);
  nodes.push_back(n6);
  nodes.push_back(n7);
  nodes.push_back(n8);
  nodes.push_back(n9);
  nodes.push_back(n10);

  n6.UpdateExternalDegree(nodes);
  n7.UpdateExternalDegree(nodes);
  n8.UpdateExternalDegree(nodes);
  n9.UpdateExternalDegree(nodes);
  n10.UpdateExternalDegree(nodes);
  /* Fill-ins are 7,8,9. */
  EXPECT_EQ(n6.degree, 240);
  /* Fill-ins are 6,8,9,10. */
  EXPECT_EQ(n7.degree, 330);
  /* Fill-ins are 6,7,9,10. */
  EXPECT_EQ(n8.degree, 320);
  /* Fill-ins are 6,7,8,10. */
  EXPECT_EQ(n9.degree, 310);
  /* Fill-ins are 7,8,9. */
  EXPECT_EQ(n10.degree, 240);
}

GTEST_TEST(MinimumDegreeOrderingTest, SimplifiedNodeComparator) {
  const SimplifiedNode a = {.degree = 0, .index = 0, .priority = 1};
  const SimplifiedNode b = {.degree = 0, .index = 1, .priority = 1};
  const SimplifiedNode c = {.degree = 1, .index = 2, .priority = 1};
  EXPECT_LT(a, b);
  EXPECT_LT(a, c);
  EXPECT_LT(b, c);

  /* Priority trumps everything else. */
  const SimplifiedNode priority_c = {.degree = 1, .index = 2, .priority = 0};

  EXPECT_LT(priority_c, a);
  EXPECT_LT(priority_c, b);
  EXPECT_LT(priority_c, c);
}

GTEST_TEST(MinimumDegreeOrderingTest, ComputeMinimumDegreeOrdering) {
  /* The block sparsity pattern is
    X X | O O O | O O O O | O O O
    X X | O O O | O O O O | O O O
    --- | ----- |---------| -----
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    --- | ----- |---------| -----
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    --- | ----- |---------| -----
    O O | O O O | O O O O | X X X
    O O | O O O | O O O O | X X X
    O O | O O O | O O O O | X X X
  The expected elimination ordering is 0, 3, 2, 1 from pen and paper
  calculation. 2 is eliminated before 1 because when 0 and 3 are eliminated, the
  degree of 2 is 3 and the degree of 1 is 4. */

  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0});
  sparsity.emplace_back(std::vector<int>{1, 2});
  sparsity.emplace_back(std::vector<int>{2});
  sparsity.emplace_back(std::vector<int>{3});
  std::vector<int> block_sizes = {2, 3, 4, 3};
  BlockSparsityPattern block_pattern(block_sizes, sparsity);
  std::vector<int> result = ComputeMinimumDegreeOrdering(block_pattern);
  EXPECT_EQ(result, std::vector<int>({0, 3, 2, 1}));
}

/* Another test for minimum degree ordering. Here we take the example from
Figure 1 and 2 in [Amestoy, 1996], where the minimum degree ordering is the
natural ordering when the size of the blocks are the same for all nodes.

[Amestoy 1996] Amestoy, Patrick R., Timothy A. Davis, and
Iain S. Duff. "An approximate minimum degree ordering algorithm." SIAM Journal
on Matrix Analysis and Applications 17.4 (1996): 886-905. */
GTEST_TEST(MinimumDegreeOrderingTest, ComputeMinimumDegreeOrdering2) {
  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0, 3, 5});
  sparsity.emplace_back(std::vector<int>{1, 4, 5, 8});
  sparsity.emplace_back(std::vector<int>{2, 4, 5, 6});
  sparsity.emplace_back(std::vector<int>{3, 6, 7});
  sparsity.emplace_back(std::vector<int>{4, 6, 8});
  sparsity.emplace_back(std::vector<int>{5});
  sparsity.emplace_back(std::vector<int>{6, 7, 8, 9});
  sparsity.emplace_back(std::vector<int>{7, 8, 9});
  sparsity.emplace_back(std::vector<int>{8, 9});
  sparsity.emplace_back(std::vector<int>{9});
  std::vector<int> block_sizes(10, 2);
  BlockSparsityPattern block_pattern(block_sizes, sparsity);
  std::vector<int> result = ComputeMinimumDegreeOrdering(block_pattern);
  EXPECT_EQ(result, std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
}

/* Here we use the same initial sparsity pattern as the example from Figure 1 in
 [Amestoy, 1996], but run Minimum Degree ordering with even nodes as priority
 nodes. We compare the computed results with pen-and-paper results from
 actually performing the chordal completion of the graph. */
GTEST_TEST(MinimumDegreeOrderingTest,
           ComputeMinimumDegreeOrderingWithPriority) {
  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0, 3, 5});
  sparsity.emplace_back(std::vector<int>{1, 4, 5, 8});
  sparsity.emplace_back(std::vector<int>{2, 4, 5, 6});
  sparsity.emplace_back(std::vector<int>{3, 6, 7});
  sparsity.emplace_back(std::vector<int>{4, 6, 8});
  sparsity.emplace_back(std::vector<int>{5});
  sparsity.emplace_back(std::vector<int>{6, 7, 8, 9});
  sparsity.emplace_back(std::vector<int>{7, 8, 9});
  sparsity.emplace_back(std::vector<int>{8, 9});
  sparsity.emplace_back(std::vector<int>{9});
  std::vector<int> block_sizes(10, 2);
  BlockSparsityPattern block_pattern(block_sizes, sparsity);
  const std::vector<int> result =
      ComputeMinimumDegreeOrdering(block_pattern, {0, 2, 4, 6, 8});
  EXPECT_EQ(result, std::vector<int>({0, 2, 4, 8, 6, 1, 3, 5, 7, 9}));
}

GTEST_TEST(MinimumDegreeOrderingTest, SymbolicCholeskyFactor) {
  /*
  In this schematic (unlike the one in the previous test), an X corresponds to a
  block with the sizes specified below.

    X X O O O O
    X X X X O X
    O X X O O O
    O X O X O O
    O O O O X X
    O X O O X X

    Symbolic Cholesky factorization should produce the following lower
    triangular sparsity pattern.

    X O O O O O
    X X O O O O
    O X X O O O
    O X X X O O
    O O O O X O
    O X X X X X  */
  std::vector<std::vector<int>> A_sparsity;
  A_sparsity.emplace_back(std::vector<int>{0, 1});
  A_sparsity.emplace_back(std::vector<int>{1, 2, 3, 5});
  A_sparsity.emplace_back(std::vector<int>{2});
  A_sparsity.emplace_back(std::vector<int>{3});
  A_sparsity.emplace_back(std::vector<int>{4, 5});
  A_sparsity.emplace_back(std::vector<int>{5});
  /* Set up arbitrary block sizes. */
  std::vector<int> A_block_sizes = {8, 4, 7, 6, 4, 4};
  BlockSparsityPattern A_block_pattern(A_block_sizes, A_sparsity);

  const BlockSparsityPattern L_block_pattern =
      SymbolicCholeskyFactor(A_block_pattern);
  const std::vector<int>& L_block_sizes = L_block_pattern.block_sizes();
  EXPECT_EQ(L_block_sizes, A_block_sizes);

  const std::vector<std::vector<int>>& L_sparsity = L_block_pattern.neighbors();
  ASSERT_EQ(L_sparsity.size(), 6);
  EXPECT_EQ(L_sparsity[0], std::vector<int>({0, 1}));
  EXPECT_EQ(L_sparsity[1], std::vector<int>({1, 2, 3, 5}));
  EXPECT_EQ(L_sparsity[2], std::vector<int>({2, 3, 5}));
  EXPECT_EQ(L_sparsity[3], std::vector<int>({3, 5}));
  EXPECT_EQ(L_sparsity[4], std::vector<int>({4, 5}));
  EXPECT_EQ(L_sparsity[5], std::vector<int>({5}));
}

/* In this test, we make a graph with 8 vertices, {0, 1, ..., 7}, such that
 odd indexed vertices belong to v₁ and even indexed vertices belong to v₂.
 Within v₁ and v₂, the block sparsity pattern looks like
    X X | O O O | O O O O | O O O
    X X | O O O | O O O O | O O O
    --- | ----- |---------| -----
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    --- | ----- |---------| -----
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    O O | X X X | X X X X | O O O
    --- | ----- |---------| -----
    O O | O O O | O O O O | X X X
    O O | O O O | O O O O | X X X
    O O | O O O | O O O O | X X X
 The expected elimination ordering for this block sparsity pattern is
 [0, 3, 2, 1] from pen and paper calculation. 2 is eliminated before 1 because
 when 0 and 3 are eliminated, the degree of 2 is 3 and the degree of 1 is 4.

 The global to local index mapping looks like
  0->0, 2->1, 4->2, 6->3
  1->0, 3->1, 5->2, 7->3.
 Because vertices in v₁ appear first in the resulting ordering, the final result
 should be [1, 7, 5, 3, 0, 6, 4, 2].
 We arbitrarily add edges across v₁ and v₂ (4-5, 4-7, 0-7, 0-5, 2-1) but they do
 not affect the result. */
GTEST_TEST(BlockSparseCholeskySolverTest, ConcatenateMdOrderingWithinGroup) {
  std::vector<std::vector<int>> sparsity;
  sparsity.emplace_back(std::vector<int>{0, 5, 7});
  sparsity.emplace_back(std::vector<int>{1, 2});
  sparsity.emplace_back(std::vector<int>{2, 4});
  sparsity.emplace_back(std::vector<int>{3, 5});
  sparsity.emplace_back(std::vector<int>{4, 5, 7});
  sparsity.emplace_back(std::vector<int>{5});
  sparsity.emplace_back(std::vector<int>{6});
  sparsity.emplace_back(std::vector<int>{7});
  std::vector<int> block_sizes = {2, 2, 3, 3, 4, 4, 3, 3};
  BlockSparsityPattern block_pattern(block_sizes, sparsity);
  const std::unordered_set<int> v1 = {1, 3, 5, 7};
  const std::vector<int> result =
      CalcAndConcatenateMdOrderingWithinGroup(block_pattern, v1);
  EXPECT_EQ(result, std::vector<int>({1, 7, 5, 3, 0, 6, 4, 2}));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
