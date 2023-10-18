#include "drake/planning/graph_algorithms/max_clique.h"

#include <gtest/gtest.h>

#include "drake/planning/graph_algorithms/test/common_graphs.h"
namespace drake {
namespace planning {
namespace graph_algorithms {
namespace {

// Test maximum clique solved via mip. Compare against the expected size of
// the solution and ensure that the result is one of the true maximum cliques in
// the graph.
void TestMaxCliqueViaMIP(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix,
    const int expected_size,
    const std::vector<VectorX<bool>>& possible_solutions) {
  MaxCliqueSolverViaMIP solver;
  MaxCliqueOptions options(&solver);
  VectorX<bool> max_clique_inds = MaxClique(adjacency_matrix, options);
  EXPECT_EQ(max_clique_inds.cast<int>().sum(), expected_size);
  bool solution_match_found = false;
  for (const auto& possible_solution : possible_solutions) {
    bool all_equal = true;
    for (int i = 0; i < max_clique_inds.rows(); ++i) {
      if (max_clique_inds(i) != possible_solution(i)) {
        all_equal = false;
        break;
      }
    }
    if (all_equal) {
      solution_match_found = true;
      break;
    }
  }
  EXPECT_TRUE(solution_match_found);
}

GTEST_TEST(MaxCliqueSolverViaMIPTest, TestConstructor) {
  // Test the default constructor.
  MaxCliqueSolverViaMIP solver1;
  EXPECT_EQ(solver1.solver_id(), solvers::MosekSolver::id());

  // Test the constructor with only the solver id passed
  MaxCliqueSolverViaMIP solver2{solvers::GurobiSolver::id()};
  EXPECT_EQ(solver2.solver_id(), solvers::GurobiSolver::id());

  // Test the constructor with the solver id and some options passed
  solvers::SolverOptions options;
  options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);
  MaxCliqueSolverViaMIP solver3{solvers::GurobiSolver::id(), options};
  EXPECT_EQ(solver3.solver_id(), solvers::GurobiSolver::id());
}

GTEST_TEST(MaxCliquMaxCliqueSolverViaMIPTesteTest, CompleteGraph) {
  for (const auto n : {3, 8}) {
    // The entire graph forms a clique
    std::vector<VectorX<bool>> possible_solutions{
        VectorX<bool>::Constant(n, true)};
    Eigen::SparseMatrix<bool> graph = Kn(n);
    TestMaxCliqueViaMIP(graph, n, possible_solutions);
  }
}

GTEST_TEST(MaxCliqueSolverViaMIPTest, BullGraph) {
  VectorX<bool> solution(5);
  // The largest stable set is (1,2,3)
  solution << false, true, true, true, false;
  std::vector<VectorX<bool>> possible_solutions{solution};
  TestMaxCliqueViaMIP(BullGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxCliqueSolverViaMIPTest, ButterflyGraph) {
  VectorX<bool> solution1(5);
  VectorX<bool> solution2(5);
  // The largest cliques are (0,1,2) and (2,3,4)
  solution1 << true, true, true, false, false;
  solution2 << false, false, true, true, true;

  std::vector<VectorX<bool>> possible_solutions{solution1, solution2};
  TestMaxCliqueViaMIP(ButterflyGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxStableSetSolverViaMIPTest, PetersenGraph) {
  // The petersen graph has a clique number of size 2, so all edges are possible
  // solutions.
  Eigen::SparseMatrix<bool> graph = PetersenGraph();
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
  TestMaxCliqueViaMIP(graph, 2, possible_solutions);
}

}  // namespace
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
