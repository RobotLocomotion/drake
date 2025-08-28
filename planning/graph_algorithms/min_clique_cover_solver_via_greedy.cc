#include "drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h"

#include <utility>

namespace drake {
namespace planning {
namespace graph_algorithms {
using Eigen::SparseMatrix;
namespace {
// Sets mat(i, :) and mat(:, i) to false if mask(i) is true.
// If mask(i) is true and partition is true, then sets mat(i, :) and mat(:, i).
// Otherwise, if partition is false mask(i) and mask(j) are true and then set
// mat(i,j) to false.
void MakeFalseRowsAndColumns(const VectorX<bool>& mask, bool partition,
                             SparseMatrix<bool>* mat) {
  if (partition) {
    for (int j = 0; j < mat->outerSize(); ++j) {
      for (SparseMatrix<bool>::InnerIterator it(*mat, j); it; ++it) {
        if (mask[j] || mask[it.index()]) {
          it.valueRef() = false;
        }
      }
    }
  } else {
    for (int j = 0; j < mat->outerSize(); ++j) {
      if (mask[j]) {
        for (SparseMatrix<bool>::InnerIterator it(*mat, j); it; ++it) {
          if (mask[it.index()]) {
            it.valueRef() = false;
          }
        }
      }
    }
  }
  *mat = mat->pruned();
}
}  // namespace

MinCliqueCoverSolverViaGreedy::MinCliqueCoverSolverViaGreedy(
    std::shared_ptr<MaxCliqueSolverBase> max_clique_solver, int min_clique_size)
    : MinCliqueCoverSolverBase(),
      max_clique_solver_(std::move(max_clique_solver)),
      min_clique_size_(min_clique_size) {
  DRAKE_THROW_UNLESS(max_clique_solver_ != nullptr);
}

std::vector<std::set<int>> MinCliqueCoverSolverViaGreedy::DoSolveMinCliqueCover(
    const Eigen::SparseMatrix<bool>& original_matrix, bool partition) {
  Eigen::SparseMatrix<bool> adjacency_matrix = original_matrix;
  std::vector<std::set<int>> ret;
  std::vector<bool> covered(original_matrix.cols(), false);
  while (adjacency_matrix.sum() > 0) {
    const VectorX<bool> max_clique =
        max_clique_solver_->SolveMaxClique(adjacency_matrix);
    if (max_clique.count() < min_clique_size_) {
      break;
    }
    std::set<int> new_clique;
    for (int i = 0; i < max_clique.size(); ++i) {
      if (max_clique(i)) {
        new_clique.insert(i);
        covered[i] = true;
      }
    }
    ret.emplace_back(std::move(new_clique));
    MakeFalseRowsAndColumns(max_clique, partition, &adjacency_matrix);
  }
  // Edge case where cliques of size 1 cannot be found by the max clique solver
  // since the adjacency matrix only contains false for that row and column.
  // Therefore, we need to add the singletons here at the end.
  if (min_clique_size_ == 1) {
    for (int i = 0; i < original_matrix.cols(); ++i) {
      if (!covered[i]) {
        ret.emplace_back(std::set<int>{i});
      }
    }
  }
  return ret;
}

}  // namespace graph_algorithms
}  // namespace planning
}  // namespace drake
