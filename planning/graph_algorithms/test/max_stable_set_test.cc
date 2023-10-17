#include "drake/planning/graph_algorithms/max_stable_set.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/planning/graph_algorithms/test/common_graphs.h"

namespace drake {
namespace planning {
namespace graph_algorithms {

// Test maximum stable set solved via mip. Compare against the expected size of
// the solution and ensure that the result is one of the true maximum stable
// sets in the graph.
void TestStableSetViaMIP(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix,
    int expected_size, std::vector<VectorX<bool>> possible_solutions) {
  MaxStableSetSolverViaMIP solver;
  MaxStableSetOptions options(&solver);
  VectorX<bool> stable_set_inds = MaxStableSet(adjacency_matrix, options);
  EXPECT_EQ(stable_set_inds.sum(), expected_size);
  bool solution_found = false;
  for (const auto& possible_solution : possible_solutions) {
    bool all_equal = true;
    for (int i = 0; i < stable_set_inds.rows(); ++i) {
      if (stable_set_inds(i) != possible_solution(i)) {
        all_equal = false;
        break;
      }
    }
    if (all_equal) {
      solution_found = true;
      break;
    }
  }
  EXPECT_TRUE(solution_found);
}

// GTEST_TEST(MaxStableSetTest, CompleteGraph) {
//  for (const auto n : {3, 8}) {
//    std::vector<VectorX<bool>> possible_solutions{VectorX<bool>(n, false)};
//    Eigen::SparseMatrix<bool> graph = Kn(n);
//    std::cout << fmt::format("{}", fmt_eigen(graph.toDense())) << std::endl;
//    TestStableSetViaMIP(Kn(n), 0, possible_solutions);
//  }
//}

GTEST_TEST(MaxStableSetTest, BullGraph) {
  VectorX<bool> solution(5);
  solution << true, false, false, false, true;
  std::vector<VectorX<bool>> possible_solutions{solution};
  TestStableSetViaMIP(BullGraph(), 2, possible_solutions);
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake