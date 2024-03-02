#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"
#include "drake/common/unused.h"

#include <algorithm>
#include <iostream>
#include <vector>
#include <numeric>

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;

namespace{

Eigen::VectorXi compute_degree_of_vertices(const Eigen::SparseMatrix<bool>& adjacency_matrix) {
    const int num_vertices = adjacency_matrix.rows();

    // Initialize a vector to store the degree of each vertex
    Eigen::VectorXi degrees(num_vertices);

    // Compute the degree of each vertex
    for (int j = 0; j < num_vertices; ++j) {
        degrees(j) = 0;
        for (SparseMatrix<bool>::InnerIterator it(adjacency_matrix, j); it; ++it) {
            degrees(j) +=1;
        }
    }
    return degrees;
}

// Sets mat(i, :) and mat(:, i) to false if mask(i) is true.
void MakeFalseRowsAndColumns(const VectorX<bool>& mask,
                             SparseMatrix<bool>* mat) {
  for (int j = 0; j < mat->outerSize(); ++j) {
    if (mask[j]) {
      for (SparseMatrix<bool>::InnerIterator it(*mat, j); it; ++it) {
        if (mask[it.index()]) {
          it.valueRef() = false;
        }
      }
    }
  }
  *mat = mat->pruned();
}

using index_t = std::vector<int>;

index_t decreasing_argsort(const Eigen::Ref<Eigen::VectorXi>& values) {
  index_t index(values.size());
  std::iota(index.begin(), index.end(), 0);
  std::sort(index.begin(), index.end(), [&values](uint8_t a, uint8_t b) {
    return values(a) > values(b);
  });
  return index;
}

}//namespace

MaxCliqueSolverViaGreedy::MaxCliqueSolverViaGreedy(){}

VectorX<bool> MaxCliqueSolverViaGreedy::DoSolveMaxClique(
    const SparseMatrix<bool>& adjacency_matrix) const {
  const int n = adjacency_matrix.rows();
  VectorX<bool> is_clique_member = VectorX<bool>::Constant(n, false); 
  Eigen::VectorXi degrees = compute_degree_of_vertices(adjacency_matrix);
  index_t decreasing_degrees_idx = decreasing_argsort(degrees);
  
  
  unused(MakeFalseRowsAndColumns);
  
  // Manually cast the return to a boolean to avoid round off errors from the
  // MIP solver.
  return is_clique_member;
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
