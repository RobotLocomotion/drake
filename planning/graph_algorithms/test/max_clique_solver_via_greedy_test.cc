#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"

#include <exception>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/graph_algorithms/test/common_graphs.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace {

using Eigen::Triplet;

// Test maximum clique solved via greedy. Compare against the expected size of
// the solution and ensure that the result is one of the true maximum cliques in
// the graph.
void TestMaxCliqueViaGreedy(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix,
    const int expected_size,
    const std::vector<VectorX<bool>>& possible_solutions) {
  MaxCliqueSolverViaGreedy solver{};
  VectorX<bool> max_clique_inds = solver.SolveMaxClique(adjacency_matrix);
  EXPECT_EQ(max_clique_inds.cast<int>().sum(), expected_size);
  bool solution_match_found = false;
  for (const auto& possible_solution : possible_solutions) {
    if (max_clique_inds == possible_solution) {
      solution_match_found = true;
      break;
    }
  }
  EXPECT_TRUE(solution_match_found);
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, TestConstructor) {
  // Test the default constructor.
  MaxCliqueSolverViaGreedy solver{};
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, CompleteGraph) {
  for (const auto n : {3, 8}) {
    // The entire graph forms a clique.
    std::vector<VectorX<bool>> possible_solutions{
        VectorX<bool>::Constant(n, true)};
    Eigen::SparseMatrix<bool> graph = internal::MakeCompleteGraph(n);
    TestMaxCliqueViaGreedy(graph, n, possible_solutions);
  }
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, BullGraph) {
  VectorX<bool> solution(5);
  // The largest clique is (1,2,3).
  solution << false, true, true, true, false;
  std::vector<VectorX<bool>> possible_solutions{solution};
  TestMaxCliqueViaGreedy(internal::BullGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, ButterflyGraph) {
  VectorX<bool> solution1(5);
  VectorX<bool> solution2(5);
  // The largest cliques are (0,1,2) and (2,3,4).
  solution1 << true, true, true, false, false;
  solution2 << false, false, true, true, true;

  std::vector<VectorX<bool>> possible_solutions{solution1, solution2};
  TestMaxCliqueViaGreedy(internal::ButterflyGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, ButteryflyWithSelfLoops) {
  // The max clique should not change if we allow self loops in the adjacency.
  // We test that here.
  Eigen::SparseMatrix<bool> graph_no_loops = internal::ButterflyGraph();
  std::vector<Triplet<bool>> triplets_identity;
  for (int i = 0; i < graph_no_loops.rows(); ++i) {
    triplets_identity.push_back(Triplet<bool>(i, i, 1));
  }
  Eigen::SparseMatrix<bool> identity(graph_no_loops.rows(),
                                     graph_no_loops.rows());
  identity.setFromTriplets(triplets_identity.begin(), triplets_identity.end());

  Eigen::SparseMatrix<bool> graph =
      (graph_no_loops + identity).template cast<bool>();
  VectorX<bool> solution1(5);
  VectorX<bool> solution2(5);
  // The largest cliques are (0,1,2) and (2,3,4).
  solution1 << true, true, true, false, false;
  solution2 << false, false, true, true, true;

  std::vector<VectorX<bool>> possible_solutions{solution1, solution2};
  TestMaxCliqueViaGreedy(graph, 3, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, PetersenGraph) {
  // The Petersen graph has a clique number of size 2, so all edges are possible
  // solutions.
  Eigen::SparseMatrix<bool> graph = internal::PetersenGraph();
  std::vector<VectorX<bool>> possible_solutions;
  possible_solutions.reserve(graph.nonZeros());
  for (int i = 0; i < graph.outerSize(); ++i) {
    for (Eigen::SparseMatrix<bool>::InnerIterator it(graph, i); it; ++it) {
      VectorX<bool> solution = VectorX<bool>::Constant(10, false);
      solution(it.row()) = true;
      solution(it.col()) = true;
      possible_solutions.push_back(solution);
    }
  }
  TestMaxCliqueViaGreedy(graph, 2, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, FullyConnectedPlusFullBipartiteGraph) {
  // The FullyConnectedPlusFullBipartiteGraph graph has a clique number of size
  // 3, but we expect the greedy algorithm to find 2 because the vertices in the
  // bipartite subgraph have higher degree.
  Eigen::SparseMatrix<bool> graph =
      internal::FullyConnectedPlusFullBipartiteGraph();
  VectorX<bool> solution1(9);
  VectorX<bool> solution2(9);
  VectorX<bool> solution3(9);
  VectorX<bool> solution4(9);
  VectorX<bool> solution5(9);
  VectorX<bool> solution6(9);
  VectorX<bool> solution7(9);
  VectorX<bool> solution8(9);
  VectorX<bool> solution9(9);

  // The max clique solutions are pairs of vertices on the bipartite graph.
  solution1 << false, false, false, true, false, false, true, false, false;
  solution2 << false, false, false, true, false, false, false, true, false;
  solution3 << false, false, false, true, false, false, false, false, true;

  solution4 << false, false, false, false, true, false, true, false, false;
  solution5 << false, false, false, false, true, false, false, true, false;
  solution6 << false, false, false, false, true, false, false, false, true;

  solution7 << false, false, false, false, false, true, true, false, false;
  solution8 << false, false, false, false, false, true, false, true, false;
  solution9 << false, false, false, false, false, true, false, false, true;
  std::vector<VectorX<bool>> possible_solutions{
      solution1, solution2, solution3, solution4, solution5,
      solution6, solution7, solution8, solution9};

  TestMaxCliqueViaGreedy(graph, 2, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaGreedyTest, AdjacencyNotSquare) {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0, 1, 1));
  Eigen::SparseMatrix<bool> graph(3, 2);
  graph.setFromTriplets(triplets.begin(), triplets.end());
  MaxCliqueSolverViaGreedy solver{};
  // Cast to void since we expect it to throw, but SolveMaxClique is
  // marked as nodiscard.
  EXPECT_THROW((void)solver.SolveMaxClique(graph), std::exception);
}

}  // namespace
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
