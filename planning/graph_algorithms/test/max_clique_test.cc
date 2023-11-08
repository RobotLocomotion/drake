#include "drake/planning/graph_algorithms/max_clique.h"

#include <exception>
#include <optional>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/graph_algorithms/test/common_graphs.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace {

using Eigen::Triplet;

// Test maximum clique solved via MIP. Compare against the expected size of
// the solution and ensure that the result is one of the true maximum cliques in
// the graph.
void TestMaxCliqueViaMip(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix,
    const int expected_size,
    const std::vector<VectorX<bool>>& possible_solutions) {
  MaxCliqueSolverViaMip solver{};
  MaxCliqueOptions options(&solver);

  // If mixed integer solver is available, find the max clique.
  if ((solvers::MosekSolver::is_available() &&
       solvers::MosekSolver::is_enabled()) ||
      (solvers::GurobiSolver::is_available() &&
       solvers::GurobiSolver::is_enabled())) {
    VectorX<bool> max_clique_inds = CalcMaxClique(adjacency_matrix, options);
    EXPECT_EQ(max_clique_inds.cast<int>().sum(), expected_size);
    bool solution_match_found = false;
    for (const auto& possible_solution : possible_solutions) {
      if (CompareMatrices(max_clique_inds.cast<int>(),
                          possible_solution.cast<int>())) {
        solution_match_found = true;
        break;
      }
    }
    EXPECT_TRUE(solution_match_found);
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(CalcMaxClique(adjacency_matrix, options),
                                ".*There is no solver available.*");
  }
}

GTEST_TEST(MaxCliqueSolverViaMipTest, TestConstructor) {
  // Test the default constructor.
  MaxCliqueSolverViaMip solver1{};
  EXPECT_EQ(solver1.get_initial_guess(), std::nullopt);
  EXPECT_EQ(solver1.solver_options(), solvers::SolverOptions());

  // Test the constructor with only the solver id passed.
  const Eigen::Vector2d initial_guess = Eigen::Vector2d::Zero();
  solvers::SolverOptions options;
  options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);
  MaxCliqueSolverViaMip solver2{initial_guess, options};
  EXPECT_TRUE(solver2.get_initial_guess().has_value());
  EXPECT_TRUE(
      CompareMatrices(solver2.get_initial_guess().value(), initial_guess));
  EXPECT_TRUE(solver2.get_solver_options()->get_print_to_console());
}

GTEST_TEST(MaxCliqueSolverViaMipTest, CompleteGraph) {
  for (const auto n : {3, 8}) {
    // The entire graph forms a clique.
    std::vector<VectorX<bool>> possible_solutions{
        VectorX<bool>::Constant(n, true)};
    Eigen::SparseMatrix<bool> graph = internal::MakeCompleteGraph(n);
    TestMaxCliqueViaMip(graph, n, possible_solutions);
  }
}

GTEST_TEST(MaxCliqueSolverViaMipTest, BullGraph) {
  VectorX<bool> solution(5);
  // The largest clique is (1,2,3).
  solution << false, true, true, true, false;
  std::vector<VectorX<bool>> possible_solutions{solution};
  TestMaxCliqueViaMip(internal::BullGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaMipTest, ButterflyGraph) {
  VectorX<bool> solution1(5);
  VectorX<bool> solution2(5);
  // The largest cliques are (0,1,2) and (2,3,4).
  solution1 << true, true, true, false, false;
  solution2 << false, false, true, true, true;

  std::vector<VectorX<bool>> possible_solutions{solution1, solution2};
  TestMaxCliqueViaMip(internal::ButterflyGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaMipTest, PetersenGraph) {
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
  TestMaxCliqueViaMip(graph, 2, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaMipTest, AdjacencyNotSquare) {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0, 1, 1));
  Eigen::SparseMatrix<bool> graph(3, 2);
  graph.setFromTriplets(triplets.begin(), triplets.end());
  MaxCliqueSolverViaMip solver{};
  MaxCliqueOptions options(&solver);
  EXPECT_THROW(CalcMaxClique(graph, options), std::runtime_error);
}

GTEST_TEST(MaxCliqueSolverViaMipTest, AdjacencyNotSymmetric) {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0, 1, 1));
  triplets.push_back(Triplet<bool>(0, 2, 1));
  Eigen::SparseMatrix<bool> graph(3, 3);
  graph.setFromTriplets(triplets.begin(), triplets.end());
  MaxCliqueSolverViaMip solver{};
  MaxCliqueOptions options(&solver);
  EXPECT_THROW(CalcMaxClique(graph, options), std::runtime_error);
}

}  // namespace
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
