#include "drake/planning/graph_algorithms/max_stable_set.h"

#include <gtest/gtest.h>

#include "drake/planning/graph_algorithms/test/common_graphs.h"
namespace drake {
namespace planning {
namespace graph_algorithms {
namespace {

// Test maximum stable set solved via mip. Compare against the expected size of
// the solution and ensure that the result is one of the true maximum stable
// sets in the graph.
void TestMaxStableSetViaMIP(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix,
    const int expected_size,
    const std::vector<VectorX<bool>>& possible_solutions) {
  MaxStableSetSolverViaMIP solver;
  MaxStableSetOptions options(&solver);
  VectorX<bool> stable_set_inds = MaxStableSet(adjacency_matrix, options);
  EXPECT_EQ(stable_set_inds.cast<int>().sum(), expected_size);
  bool solution_match_found = false;
  for (const auto& possible_solution : possible_solutions) {
    bool all_equal = true;
    for (int i = 0; i < stable_set_inds.rows(); ++i) {
      if (stable_set_inds(i) != possible_solution(i)) {
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

GTEST_TEST(MaxStableSetSolverViaMIPTest, TestConstructor) {
  // Test the default constructor.
  MaxStableSetSolverViaMIP solver1;
  EXPECT_EQ(solver1.solver_id(), solvers::MosekSolver::id());

  // Test the constructor with only the solver id passed
  MaxStableSetSolverViaMIP solver2{solvers::GurobiSolver::id()};
  EXPECT_EQ(solver2.solver_id(), solvers::GurobiSolver::id());

  // Test the constructor with the solver id and some options passed
  solvers::SolverOptions options;
  options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);
  MaxStableSetSolverViaMIP solver3{solvers::GurobiSolver::id(), options};
  EXPECT_EQ(solver3.solver_id(), solvers::GurobiSolver::id());
}

GTEST_TEST(MaxStableSetSolverViaMIPTest, CompleteGraph) {
  for (const auto n : {3, 8}) {
    std::vector<VectorX<bool>> possible_solutions;
    // The largest stable set is any singleton in the graph
    for (int i = 0; i < n; ++i) {
      VectorX<bool> sol = VectorX<bool>::Constant(n, false);
      sol(i) = true;
      possible_solutions.push_back(sol);
    }
    Eigen::SparseMatrix<bool> graph = Kn(n);
    TestMaxStableSetViaMIP(Kn(n), 1, possible_solutions);
  }
}

GTEST_TEST(MaxStableSetSolverViaMIPTest, BullGraph) {
  VectorX<bool> solution(5);
  // The largest stable set is (0,3,4)
  solution << true, false, true, false, true;
  std::vector<VectorX<bool>> possible_solutions{solution};
  TestMaxStableSetViaMIP(BullGraph(), 3, possible_solutions);
}

GTEST_TEST(MaxStableSetSolverViaMIPTest, ButterflyGraph) {
  VectorX<bool> solution1(5);
  VectorX<bool> solution2(5);
  VectorX<bool> solution3(5);
  VectorX<bool> solution4(5);
  // The largest stable sets are (0,3), (0,4), (1,3), (1,4)
  solution1 << true, false, false, true, false;
  solution2 << true, false, false, false, true;
  solution3 << false, true, false, true, false;
  solution4 << false, true, false, false, true;

  std::vector<VectorX<bool>> possible_solutions{solution1, solution2, solution3,
                                                solution4};
  TestMaxStableSetViaMIP(ButterflyGraph(), 2, possible_solutions);
}

GTEST_TEST(MaxStableSetSolverViaMIPTest, PetersenGraph) {
  VectorX<bool> solution1 = VectorX<bool>::Constant(10, false);
  VectorX<bool> solution2 = VectorX<bool>::Constant(10, false);
  VectorX<bool> solution3 = VectorX<bool>::Constant(10, false);
  VectorX<bool> solution4 = VectorX<bool>::Constant(10, false);
  // The largest stable sets are (0,1,7,9), (0,4,6,8), (1,2,5,8), (2,3,6,7),
  // (3,4,5,7)
  std::vector<std::vector<int>> solutions_index = {
      {0, 1, 7, 9}, {0, 4, 6, 8}, {1, 2, 5, 8}, {2, 3, 6, 7}, {3, 4, 5, 7},
  };
  std::vector<VectorX<bool>> possible_solutions;
  for (const auto& sol_inds : solutions_index) {
    possible_solutions.push_back(VectorX<bool>::Constant(10, false));
    for (const auto& ind : sol_inds) {
      possible_solutions.back()(ind) = true;
    }
  }
  TestMaxStableSetViaMIP(PetersenGraph(), 4, possible_solutions);
}

}  // namespace
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
