#include "drake/planning/graph_algorithms/test/common_graphs.h"

#include <vector>

#include <Eigen/Sparse>

#include "drake/planning/graph_algorithms/graph_algorithms_internal.h"

namespace drake {
namespace planning {
namespace graph_algorithms {
namespace internal {

using Eigen::Dynamic;
using Eigen::SparseMatrix;
using Eigen::Triplet;

Eigen::SparseMatrix<bool> MakeCompleteGraph(const int n) {
  return (Eigen::MatrixXd::Ones(n, n) - Eigen::MatrixXd::Identity(n, n))
      .cast<bool>()
      .sparseView();
}

Eigen::SparseMatrix<bool> BullGraph() {
  std::vector<Triplet<bool>> triplets;

  triplets.emplace_back(0, 1, true);
  triplets.emplace_back(1, 2, true);
  triplets.emplace_back(1, 3, true);
  triplets.emplace_back(2, 3, true);
  triplets.emplace_back(3, 4, true);

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(5, 5);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

Eigen::SparseMatrix<bool> ButterflyGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.emplace_back(0, 1, true);
  triplets.emplace_back(0, 2, true);
  triplets.emplace_back(1, 2, true);
  triplets.emplace_back(2, 3, true);
  triplets.emplace_back(2, 4, true);
  triplets.emplace_back(3, 4, true);

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(5, 5);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

Eigen::SparseMatrix<bool> PetersenGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.emplace_back(0, 2, true);
  triplets.emplace_back(0, 3, true);
  triplets.emplace_back(1, 3, true);
  triplets.emplace_back(1, 4, true);
  triplets.emplace_back(2, 4, true);

  triplets.emplace_back(5, 6, true);
  triplets.emplace_back(6, 7, true);
  triplets.emplace_back(7, 8, true);
  triplets.emplace_back(8, 9, true);
  triplets.emplace_back(5, 9, true);

  triplets.emplace_back(0, 5, true);
  triplets.emplace_back(1, 6, true);
  triplets.emplace_back(2, 7, true);
  triplets.emplace_back(3, 8, true);
  triplets.emplace_back(4, 9, true);

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(10, 10);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

Eigen::SparseMatrix<bool> FullyConnectedPlusFullBipartiteGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.emplace_back(0, 1, true);
  triplets.emplace_back(0, 2, true);
  triplets.emplace_back(1, 2, true);
  triplets.emplace_back(3, 6, true);
  triplets.emplace_back(3, 7, true);
  triplets.emplace_back(3, 8, true);
  triplets.emplace_back(4, 6, true);
  triplets.emplace_back(4, 7, true);
  triplets.emplace_back(4, 8, true);
  triplets.emplace_back(5, 6, true);
  triplets.emplace_back(5, 7, true);
  triplets.emplace_back(5, 8, true);

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(9, 9);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
