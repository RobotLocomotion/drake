#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"

#include <algorithm>
#include <list>
#include <memory>
#include <numeric>
#include <vector>

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

namespace {

// Computes the degrees of all verticies in a graph given by its adjacency
// matrix.
Eigen::VectorXi ComputeDegreeOfVertices(
    const Eigen::SparseMatrix<bool>& adjacency_matrix) {
  DRAKE_ASSERT(adjacency_matrix.rows() == adjacency_matrix.cols());
  DRAKE_ASSERT(adjacency_matrix.toDense() ==
               adjacency_matrix.transpose().toDense());

  // Initialize a vector to store the degree of each vertex
  Eigen::VectorXi degrees(adjacency_matrix.cols());
  // Compute the degree of each candidate
  int candidate_index{0};
  for (auto j = 0; j < adjacency_matrix.cols(); ++j) {
    degrees(candidate_index) = 0;
    for (SparseMatrix<bool>::InnerIterator it(adjacency_matrix, j); it; ++it) {
      degrees(candidate_index) += 1;
    }
    ++candidate_index;
  }
  return degrees;
}

// Removes all nodes from available_nodes that are not adjacent to point_to_add.
void UpdateAvailableNodes(const Eigen::SparseMatrix<bool>& mat,
                          const int point_to_add,
                          std::list<int>* available_nodes) {
  for (auto it = available_nodes->begin(); it != available_nodes->end();) {
    int element = *it;
    if (!mat.coeff(element, point_to_add) || (element == point_to_add)) {
      available_nodes->erase(it++);
    } else {
      ++it;
    }
  }
}

std::list<int> DecreasingArgsort(
    const Eigen::Ref<const Eigen::VectorXi>& values) {
  std::vector<int> index(values.size());
  std::iota(index.begin(), index.end(), 0);
  std::sort(index.begin(), index.end(), [&values](int a, int b) {
    return values(a) > values(b);
  });
  return {index.begin(), index.end()};
}

}  // namespace

VectorX<bool> MaxCliqueSolverViaGreedy::DoSolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) const {
  std::vector<int> clique_members;
  SparseMatrix<bool> curr_ad_matrix = adjacency_matrix;
  const Eigen::VectorXi degrees = ComputeDegreeOfVertices(curr_ad_matrix);
  std::list<int> available_nodes = DecreasingArgsort(degrees);

  while (available_nodes.size() > 0) {
    int point_to_add = available_nodes.front();
    available_nodes.pop_front();
    clique_members.push_back(point_to_add);
    UpdateAvailableNodes(curr_ad_matrix, point_to_add, &available_nodes);
  }

  // fill in solution
  Eigen::VectorX<bool> is_clique_member =
      Eigen::VectorX<bool>::Constant(adjacency_matrix.rows(), false);
  for (const int i : clique_members) {
    is_clique_member(i) = true;
  }
  return is_clique_member;
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
