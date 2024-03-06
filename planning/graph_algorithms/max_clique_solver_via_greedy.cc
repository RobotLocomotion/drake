#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"

#include <algorithm>
#include <numeric>
#include <set>
#include <vector>

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

namespace {

// Computes the degrees of all verticies in a graph given by its adjacency
// matrix.
Eigen::VectorXi ComputeDegreeOfVertices(
    const Eigen::SparseMatrix<bool>& adjacency_matrix,
    const std::set<int>& available_nodes) {
  DRAKE_DEMAND(adjacency_matrix.rows() == adjacency_matrix.cols());
  DRAKE_DEMAND(adjacency_matrix.isApprox(adjacency_matrix.transpose()));

  // Initialize a vector to store the degree of each vertex
  Eigen::VectorXi degrees(available_nodes.size());
  // Compute the degree of each candidate
  int candidate_index{0};
  for (auto j = available_nodes.begin(); j != available_nodes.end(); ++j) {
    degrees(candidate_index) = 0;
    for (SparseMatrix<bool>::InnerIterator it(adjacency_matrix, *j); it; ++it) {
      degrees(candidate_index) += 1;
    }
    ++candidate_index;
  }
  return degrees;
}

// Removes all nodes from available_nodes that are not adjacent to point_to_add.
void UpdateAvailableNodes(const Eigen::SparseMatrix<bool>& mat,
                          const int point_to_add,
                          std::set<int>* available_nodes) {
  for (auto it = available_nodes->begin(); it != available_nodes->end();) {
    int element = *it;
    if (!mat.coeff(element, point_to_add) || (element == point_to_add)) {
      available_nodes->erase(it++);
    } else {
      ++it;
    }
  }
}

// Sets mat(i,:) and mat(:,i) to false if i is not a neighbour of index.
void MaskNonNeighbours(const int index, Eigen::SparseMatrix<bool>* mat) {
  for (int j = 0; j < mat->outerSize(); ++j) {
    if (!(mat->coeff(index, j)) && index != j) {
      for (Eigen::SparseMatrix<bool>::InnerIterator it(*mat, j); it; ++it) {
        int row = it.row();
        int col = it.col();
        mat->coeffRef(row, col) = false;
        mat->coeffRef(col, row) = false;
      }
    }
  }
  *mat = mat->pruned();
}

std::vector<int> decreasing_argsort(
    const Eigen::Ref<const Eigen::VectorXi>& values) {
  std::vector<int> index(values.size());
  std::iota(index.begin(), index.end(), 0);
  std::sort(index.begin(), index.end(), [&values](uint8_t a, uint8_t b) {
    return values(a) > values(b);
  });
  return index;
}

}  // namespace

VectorX<bool> MaxCliqueSolverViaGreedy::DoSolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) const {
  const int n = adjacency_matrix.rows();
  std::set<int> available_nodes;
  for (int i = 0; i < n; ++i) {
    available_nodes.insert(available_nodes.end(), i);
  }

  std::vector<int> clique_members;
  SparseMatrix<bool> curr_ad_matrix = adjacency_matrix;

  while (available_nodes.size() > 0) {
    const Eigen::VectorXi degrees =
        ComputeDegreeOfVertices(curr_ad_matrix, available_nodes);
    const std::vector<int> degrees_sort_idx = decreasing_argsort(degrees);
    auto it = available_nodes.begin();
    std::advance(it, degrees_sort_idx.at(0));
    int point_to_add = *it;
    clique_members.push_back(point_to_add);

    UpdateAvailableNodes(curr_ad_matrix, point_to_add, &available_nodes);
    MaskNonNeighbours(point_to_add, &curr_ad_matrix);
  }

  // fill in solution
  Eigen::VectorX<bool> is_clique_member =
      Eigen::VectorX<bool>::Constant(n, false);
  for (const int i : clique_members) {
    is_clique_member(i) = true;
  }
  return is_clique_member;
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
