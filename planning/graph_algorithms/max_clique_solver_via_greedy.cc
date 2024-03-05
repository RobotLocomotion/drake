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
    const Eigen::SparseMatrix<bool>& adjacency_matrix) {
  const int num_vertices = adjacency_matrix.cols();

  DRAKE_DEMAND(adjacency_matrix.rows() == adjacency_matrix.cols());
  DRAKE_DEMAND(adjacency_matrix.isApprox(adjacency_matrix.transpose()));

  // Initialize a vector to store the degree of each vertex
  Eigen::VectorXi degrees(num_vertices);
  // Compute the degree of each vertex
  for (int j = 0; j < adjacency_matrix.outerSize(); ++j) {
    degrees(j) = 0;
    for (SparseMatrix<bool>::InnerIterator it(adjacency_matrix, j); it; ++it) {
      degrees(j) += 1;
    }
  }
  return degrees;
}

// Given the vertices ordered by their degree in descending order return the
// first that is in the list of available vertices.
int PickBest(const std::vector<int>& degrees_sort_idx,
             const std::set<int>& available_nodes) {
  for (int d : degrees_sort_idx) {
    auto it = available_nodes.find(d);
    if (it != available_nodes.end()) {
      return d;
    }
  }
  throw std::runtime_error("No valid element found in available_nodes.");
}

// Removes all nodes from available_nodes that are not adjacent to point_to_add.
void UpdateAvailableNodes(const Eigen::SparseMatrix<bool>* mat,
                          const int point_to_add,
                          std::set<int>* available_nodes) {
  for (auto it = available_nodes->begin(); it != available_nodes->end();) {
    int element = *it;
    if (!mat->coeff(element, point_to_add) || (element == point_to_add)) {
      available_nodes->erase(it++);
    } else {
      ++it;
    }
  }
}

// Sets mat(i,:) and mat(:,i) to false if i is not a neighbour of index
void MaskNonNeighbours(Eigen::SparseMatrix<bool>* mat, int index) {
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
    available_nodes.insert(i);
  }

  std::vector<int> clique_members;
  SparseMatrix<bool> curr_ad_matrix = adjacency_matrix;

  while (available_nodes.size() > 0) {
    Eigen::VectorXi degrees = ComputeDegreeOfVertices(curr_ad_matrix);
    // std::cout<<fmt::format("{}", fmt_eigen(degrees))<<std::endl;
    std::vector<int> degrees_sort_idx = decreasing_argsort(degrees);
    int point_to_add = PickBest(degrees_sort_idx, available_nodes);
    // std::cout<<fmt::format("point to add {}", point_to_add)<<std::endl;

    clique_members.push_back(point_to_add);
    UpdateAvailableNodes(&curr_ad_matrix, point_to_add, &available_nodes);
    // std::cout << "Available Nodes: ";
    // for (const auto& element : available_nodes) {
    //     std::cout << element << " ";
    // }
    // std::cout << std::endl;

    // std::cout<<fmt::format("Available nodes {}",
    // available_nodes.size())<<std::endl;
    MaskNonNeighbours(&curr_ad_matrix, point_to_add);
  }

  // fill in solution
  Eigen::VectorX<bool> is_clique_member =
      Eigen::VectorX<bool>::Constant(n, false);
  for (const int i : clique_members) {
    is_clique_member(i) = true;
  }
  return is_clique_member.cast<bool>();
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
