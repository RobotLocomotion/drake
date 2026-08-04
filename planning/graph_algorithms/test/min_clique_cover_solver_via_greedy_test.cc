#include "drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h"

#include <exception>
#include <memory>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"
#include "drake/planning/graph_algorithms/test/common_graphs.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace {

// Test min clique cover. Compare against the expected size of
// the solution and ensure that the result is one of the true maximum cliques in
// the graph.
using clique_solution_type = std::vector<std::set<int>>;
// Make the clique cover a set of sets to simplify comparison.
using comparable_clique_solution_type = std::set<std::set<int>>;

// Call SolveMinCliqueCover for the given solver with the given adjacency matrix
// and partition argument and checks whether the returned solution is one of the
// possible solutions.
void TestMinCliqueCover(
    const Eigen::Ref<const Eigen::SparseMatrix<bool>>& adjacency_matrix,
    bool partition,
    const std::vector<comparable_clique_solution_type>& possible_solutions,
    MinCliqueCoverSolverViaGreedy* solver) {
  clique_solution_type min_clique_cover_vect =
      solver->SolveMinCliqueCover(adjacency_matrix, partition);
  // Convert the vector of sets into a set of sets so that comparison is easier.
  comparable_clique_solution_type min_clique_cover(
      min_clique_cover_vect.begin(), min_clique_cover_vect.end());

  // If both the possible solutions is empty and the min_clique_cover is empty,
  // then the clique cover is empty and this is a valid solution. Otherwise,
  // this value is initialized to false.
  bool solution_match_found =
      possible_solutions.empty() && min_clique_cover_vect.empty();
  for (const auto& possible_solution : possible_solutions) {
    if (min_clique_cover == possible_solution) {
      solution_match_found = true;
      break;
    }
  }
  EXPECT_TRUE(solution_match_found);
}

GTEST_TEST(MinCliqueCoverSolverViaGreedyTest,
           TestConstructorSettersAndGetters) {
  // Test the default constructor.
  MinCliqueCoverSolverViaGreedy solver{
      std::make_unique<MaxCliqueSolverViaGreedy>(), 3};

  EXPECT_EQ(solver.get_min_clique_size(), 3);

  solver.set_min_clique_size(5);
  EXPECT_EQ(solver.get_min_clique_size(), 5);
}

GTEST_TEST(MinCliqueCoverSolverViaGreedyTestTest, CompleteGraph) {
  const int n = 15;
  Eigen::SparseMatrix<bool> graph = internal::MakeCompleteGraph(n);
  comparable_clique_solution_type solution;
  std::set<int> solution_set;
  for (int i = 0; i < n; ++i) {
    solution_set.insert(i);
  }
  solution.insert(solution_set);

  // There is only one solution.
  std::vector<comparable_clique_solution_type> possible_solutions;
  possible_solutions.push_back(solution);

  MinCliqueCoverSolverViaGreedy solver{
      std::make_unique<MaxCliqueSolverViaGreedy>(), 1};

  // There is only one clique so the min clique cover is the whole graph.
  TestMinCliqueCover(graph, false, possible_solutions, &solver);
  TestMinCliqueCover(graph, true, possible_solutions, &solver);
}

GTEST_TEST(MinCliqueCoverSolverViaGreedyTestTest, BullGraph) {
  Eigen::SparseMatrix<bool> graph = internal::BullGraph();
  MinCliqueCoverSolverViaGreedy solver{
      std::make_unique<MaxCliqueSolverViaGreedy>(), 1};

  // The solution when vertices are allowed to be repeated.
  comparable_clique_solution_type solution;
  solution.insert(std::initializer_list<int>{1, 2, 3});
  solution.insert(std::initializer_list<int>{1, 0});
  solution.insert(std::initializer_list<int>{3, 4});
  std::vector<comparable_clique_solution_type> possible_solutions;
  possible_solutions.push_back(solution);
  TestMinCliqueCover(graph, false, possible_solutions, &solver);

  // The solution when vertices are not allowed to be repeated.
  solution.clear();
  solution.insert(std::initializer_list<int>{1, 2, 3});
  solution.insert(std::initializer_list<int>{0});
  solution.insert(std::initializer_list<int>{4});
  possible_solutions.clear();
  possible_solutions.push_back(solution);
  TestMinCliqueCover(graph, true, possible_solutions, &solver);

  // The BullGraph graph has only one clique of
  // size 3.
  solver.set_min_clique_size(3);
  solution.clear();
  solution.insert(std::initializer_list<int>{1, 2, 3});
  possible_solutions.clear();
  possible_solutions.push_back(solution);
  TestMinCliqueCover(graph, false, possible_solutions, &solver);
}

GTEST_TEST(MinCliqueCoverSolverViaGreedyTestTest, ButterflyGraph) {
  Eigen::SparseMatrix<bool> graph = internal::ButterflyGraph();
  MinCliqueCoverSolverViaGreedy solver{
      std::make_unique<MaxCliqueSolverViaGreedy>(), 1};

  comparable_clique_solution_type solution;
  solution.insert(std::initializer_list<int>{0, 1, 2});
  solution.insert(std::initializer_list<int>{2, 3, 4});
  std::vector<comparable_clique_solution_type> possible_solutions;
  possible_solutions.push_back(solution);
  TestMinCliqueCover(graph, false, possible_solutions, &solver);
  // If the min clique size is 3, then the solution is still the same.
  solver.set_min_clique_size(3);
  TestMinCliqueCover(graph, false, possible_solutions, &solver);

  // If the min clique size is 4, then the solution are no cliques.
  solver.set_min_clique_size(4);
  TestMinCliqueCover(graph, false,
                     std::vector<comparable_clique_solution_type>{}, &solver);

  // Now we test when we ask for a partition
  solver.set_min_clique_size(1);
  solution.clear();
  possible_solutions.clear();
  solution.insert(std::initializer_list<int>{0, 1, 2});
  solution.insert(std::initializer_list<int>{3, 4});
  possible_solutions.push_back(solution);
  solution.clear();
  solution.insert(std::initializer_list<int>{0, 3, 4});
  solution.insert(std::initializer_list<int>{1, 2});
  possible_solutions.push_back(solution);
  EXPECT_EQ(ssize(possible_solutions), 2);
  TestMinCliqueCover(graph, true, possible_solutions, &solver);

  // If we make the minimum clique size 3, then the solution is only one of
  // lobes of the butterfly.
  solver.set_min_clique_size(3);
  solution.clear();
  possible_solutions.clear();
  solution.insert(std::initializer_list<int>{0, 1, 2});
  possible_solutions.push_back(solution);
  solution.clear();
  solution.insert(std::initializer_list<int>{0, 3, 4});
  possible_solutions.push_back(solution);
  EXPECT_EQ(ssize(possible_solutions), 2);
  TestMinCliqueCover(graph, true, possible_solutions, &solver);
}

GTEST_TEST(MinCliqueCoverSolverViaGreedyTestTest, PetersenGraph) {
  Eigen::SparseMatrix<bool> graph = internal::PetersenGraph();
  MinCliqueCoverSolverViaGreedy solver{
      std::make_unique<MaxCliqueSolverViaGreedy>(), 1};

  // The largest clique is of size 2, so if we ask for a cover which allows
  // repeated vertices, we will get all the edges of the adjacency matrix.
  comparable_clique_solution_type solution;
  for (int i = 0; i < graph.cols(); ++i) {
    for (Eigen::SparseMatrix<bool>::InnerIterator it(graph, i); it; ++it) {
      if (it.index() > i && it.value()) {
        solution.insert(std::initializer_list<int>{i, it.index()});
      }
    }
  }
  std::vector<comparable_clique_solution_type> possible_solutions;
  possible_solutions.push_back(solution);

  TestMinCliqueCover(graph, false, possible_solutions, &solver);

  // If the min clique size is 3, then there are no cliques.
  solver.set_min_clique_size(3);
  TestMinCliqueCover(graph, false,
                     std::vector<comparable_clique_solution_type>{}, &solver);
}

}  // namespace
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
