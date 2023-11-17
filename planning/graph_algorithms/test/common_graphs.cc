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

  triplets.push_back(Triplet<bool>(0, 1, true));
  triplets.push_back(Triplet<bool>(1, 2, true));
  triplets.push_back(Triplet<bool>(1, 3, true));
  triplets.push_back(Triplet<bool>(2, 3, true));
  triplets.push_back(Triplet<bool>(3, 4, true));

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(5, 5);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

Eigen::SparseMatrix<bool> ButterflyGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0, 1, 1));
  triplets.push_back(Triplet<bool>(0, 2, 1));
  triplets.push_back(Triplet<bool>(1, 2, 1));
  triplets.push_back(Triplet<bool>(2, 3, 1));
  triplets.push_back(Triplet<bool>(2, 4, 1));
  triplets.push_back(Triplet<bool>(3, 4, 1));

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(5, 5);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

Eigen::SparseMatrix<bool> PetersenGraph() {
  std::vector<Triplet<bool>> triplets;
  triplets.push_back(Triplet<bool>(0, 2, 1));
  triplets.push_back(Triplet<bool>(0, 3, 1));
  triplets.push_back(Triplet<bool>(1, 3, 1));
  triplets.push_back(Triplet<bool>(1, 4, 1));
  triplets.push_back(Triplet<bool>(2, 4, 1));

  triplets.push_back(Triplet<bool>(5, 6, 1));
  triplets.push_back(Triplet<bool>(6, 7, 1));
  triplets.push_back(Triplet<bool>(7, 8, 1));
  triplets.push_back(Triplet<bool>(8, 9, 1));
  triplets.push_back(Triplet<bool>(5, 9, 1));

  triplets.push_back(Triplet<bool>(0, 5, 1));
  triplets.push_back(Triplet<bool>(1, 6, 1));
  triplets.push_back(Triplet<bool>(2, 7, 1));
  triplets.push_back(Triplet<bool>(3, 8, 1));
  triplets.push_back(Triplet<bool>(4, 9, 1));

  internal::SymmetrizeTripletList(&triplets);
  Eigen::SparseMatrix<bool> ret(10, 10);
  ret.setFromTriplets(triplets.begin(), triplets.end());
  return ret;
}

}  // namespace internal
}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
