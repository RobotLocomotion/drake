#include "drake/planning/graph_algorithms/graph_algorithms_internal.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/planning/graph_algorithms/test/common_graphs.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace {

GTEST_TEST(SymmetrizeTripletList, TestSymmetrization) {
  // an arbitrary list of entries
  std::vector<Eigen::Triplet<bool>> entries;
  entries.emplace_back(0, 3, 1);
  entries.emplace_back(0, 4, 1);
  entries.emplace_back(1, 3, 1);
  entries.emplace_back(1, 4, 1);

  internal::SymmetrizeTripletList(&entries);

  std::vector<Eigen::Triplet<bool>> entries_expected;
  entries_expected.emplace_back(0, 3, 1);
  entries_expected.emplace_back(0, 4, 1);
  entries_expected.emplace_back(1, 3, 1);
  entries_expected.emplace_back(1, 4, 1);
  entries_expected.emplace_back(3, 0, 1);
  entries_expected.emplace_back(4, 0, 1);
  entries_expected.emplace_back(3, 1, 1);
  entries_expected.emplace_back(4, 1, 1);

  auto check_all_elts_in_v1_are_in_v2 =
      [](const std::vector<Eigen::Triplet<bool>>& v1,
         const std::vector<Eigen::Triplet<bool>>& v2) {
        for (const auto& elt1 : v1) {
          bool found_elt = false;
          for (const auto& elt2 : v2) {
            if (elt1.row() == elt2.row() && elt1.col() == elt2.col() &&
                elt1.value() == elt2.value()) {
              found_elt = true;
              break;
            }
          }
          if (!found_elt) {
            return false;
          }
        }
        return true;
      };
  // Check if every element in entries appears in entries_expected.
  EXPECT_TRUE(check_all_elts_in_v1_are_in_v2(entries, entries_expected));
  EXPECT_TRUE(check_all_elts_in_v1_are_in_v2(entries_expected, entries));
}

GTEST_TEST(ComplementGraph, CompleteGraph) {
  for (const int n : {3, 8}) {
    Eigen::SparseMatrix<bool> graph = internal::MakeCompleteGraph(n);
    Eigen::SparseMatrix<bool> complement_graph =
        internal::ComplementAdjacencyMatrix(graph);
    Eigen::SparseMatrix<bool> complement_graph_expected(n, n);
    complement_graph.finalize();
    EXPECT_TRUE(
        CompareMatrices(complement_graph.toDense().cast<double>(),
                        complement_graph_expected.toDense().cast<double>()));
  }
}

GTEST_TEST(ComplementGraph, BullGraph) {
  Eigen::SparseMatrix<bool> graph = internal::BullGraph();
  Eigen::SparseMatrix<bool> complement_graph =
      internal::ComplementAdjacencyMatrix(graph);

  std::vector<Eigen::Triplet<bool>> expected_entries;
  expected_entries.emplace_back(0, 2, 1);
  expected_entries.emplace_back(0, 3, 1);
  expected_entries.emplace_back(0, 4, 1);

  expected_entries.emplace_back(1, 4, 1);
  expected_entries.emplace_back(2, 4, 1);

  internal::SymmetrizeTripletList(&expected_entries);
  Eigen::SparseMatrix<bool> complement_graph_expected(graph.rows(),
                                                      graph.cols());
  complement_graph_expected.setFromTriplets(expected_entries.begin(),
                                            expected_entries.end());
  EXPECT_TRUE(
      CompareMatrices(complement_graph.toDense().cast<double>(),
                      complement_graph_expected.toDense().cast<double>()));
}

GTEST_TEST(ComplementGraph, ButterflyGraph) {
  Eigen::SparseMatrix<bool> graph = internal::ButterflyGraph();
  Eigen::SparseMatrix<bool> complement_graph =
      internal::ComplementAdjacencyMatrix(graph);

  std::vector<Eigen::Triplet<bool>> expected_entries;
  expected_entries.emplace_back(0, 3, 1);
  expected_entries.emplace_back(0, 4, 1);
  expected_entries.emplace_back(1, 3, 1);
  expected_entries.emplace_back(1, 4, 1);

  internal::SymmetrizeTripletList(&expected_entries);
  Eigen::SparseMatrix<bool> complement_graph_expected(graph.rows(),
                                                      graph.cols());
  complement_graph_expected.setFromTriplets(expected_entries.begin(),
                                            expected_entries.end());
  EXPECT_TRUE(
      CompareMatrices(complement_graph.toDense().cast<double>(),
                      complement_graph_expected.toDense().cast<double>()));
}

GTEST_TEST(ComplementGraph, PetersenGraph) {
  Eigen::SparseMatrix<bool> graph = internal::PetersenGraph();
  Eigen::SparseMatrix<bool> complement_graph =
      internal::ComplementAdjacencyMatrix(graph);

  std::vector<Eigen::Triplet<bool>> expected_entries;
  expected_entries.emplace_back(0, 1, 1);
  expected_entries.emplace_back(0, 4, 1);
  expected_entries.emplace_back(0, 6, 1);
  expected_entries.emplace_back(0, 7, 1);
  expected_entries.emplace_back(0, 8, 1);
  expected_entries.emplace_back(0, 9, 1);

  expected_entries.emplace_back(1, 2, 1);
  expected_entries.emplace_back(1, 5, 1);
  expected_entries.emplace_back(1, 7, 1);
  expected_entries.emplace_back(1, 8, 1);
  expected_entries.emplace_back(1, 9, 1);

  expected_entries.emplace_back(2, 3, 1);
  expected_entries.emplace_back(2, 5, 1);
  expected_entries.emplace_back(2, 6, 1);
  expected_entries.emplace_back(2, 8, 1);
  expected_entries.emplace_back(2, 9, 1);

  expected_entries.emplace_back(3, 4, 1);
  expected_entries.emplace_back(3, 5, 1);
  expected_entries.emplace_back(3, 6, 1);
  expected_entries.emplace_back(3, 7, 1);
  expected_entries.emplace_back(3, 9, 1);

  expected_entries.emplace_back(4, 5, 1);
  expected_entries.emplace_back(4, 6, 1);
  expected_entries.emplace_back(4, 7, 1);
  expected_entries.emplace_back(4, 8, 1);

  expected_entries.emplace_back(5, 7, 1);
  expected_entries.emplace_back(5, 8, 1);

  expected_entries.emplace_back(6, 8, 1);
  expected_entries.emplace_back(6, 9, 1);

  expected_entries.emplace_back(7, 9, 1);

  internal::SymmetrizeTripletList(&expected_entries);
  Eigen::SparseMatrix<bool> complement_graph_expected(graph.rows(),
                                                      graph.cols());
  complement_graph_expected.setFromTriplets(expected_entries.begin(),
                                            expected_entries.end());
  EXPECT_TRUE(
      CompareMatrices(complement_graph.toDense().cast<double>(),
                      complement_graph_expected.toDense().cast<double>()));
}

GTEST_TEST(ComplementGraph, FullyConnectedPlusBipartiteGraph) {
  Eigen::SparseMatrix<bool> graph =
      internal::FullyConnectedPlusFullBipartiteGraph();
  Eigen::SparseMatrix<bool> complement_graph =
      internal::ComplementAdjacencyMatrix(graph);

  std::vector<Eigen::Triplet<bool>> expected_entries;
  // bipartite part
  expected_entries.emplace_back(3, 4, 1);
  expected_entries.emplace_back(3, 5, 1);
  expected_entries.emplace_back(4, 5, 1);
  expected_entries.emplace_back(6, 7, 1);
  expected_entries.emplace_back(6, 8, 1);
  expected_entries.emplace_back(7, 8, 1);

  // connection fully connected to bipartite part
  expected_entries.emplace_back(0, 3, 1);
  expected_entries.emplace_back(0, 4, 1);
  expected_entries.emplace_back(0, 5, 1);
  expected_entries.emplace_back(0, 6, 1);
  expected_entries.emplace_back(0, 7, 1);
  expected_entries.emplace_back(0, 8, 1);

  expected_entries.emplace_back(1, 3, 1);
  expected_entries.emplace_back(1, 4, 1);
  expected_entries.emplace_back(1, 5, 1);
  expected_entries.emplace_back(1, 6, 1);
  expected_entries.emplace_back(1, 7, 1);
  expected_entries.emplace_back(1, 8, 1);

  expected_entries.emplace_back(2, 3, 1);
  expected_entries.emplace_back(2, 4, 1);
  expected_entries.emplace_back(2, 5, 1);
  expected_entries.emplace_back(2, 6, 1);
  expected_entries.emplace_back(2, 7, 1);
  expected_entries.emplace_back(2, 8, 1);

  internal::SymmetrizeTripletList(&expected_entries);
  Eigen::SparseMatrix<bool> complement_graph_expected(graph.rows(),
                                                      graph.cols());
  complement_graph_expected.setFromTriplets(expected_entries.begin(),
                                            expected_entries.end());
  EXPECT_TRUE(
      CompareMatrices(complement_graph.toDense().cast<double>(),
                      complement_graph_expected.toDense().cast<double>()));
}

}  // namespace
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
