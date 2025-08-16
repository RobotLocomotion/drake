#include "drake/multibody/tree/block_system_jacobian_cache.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
BlockSystemJacobianCache<T>::BlockSystemJacobianCache(
    const SpanningForest& forest)
    : total_rows_(6 * forest.num_mobods()),
      total_cols_(forest.num_velocities()),
      block_system_jacobian_(forest.trees().size()) {
  // We're counting rows & columns just to verify consistency with the
  // totals we stored above. We need to account for World's 6 rows in the
  // count, but that does _not_ mean World is part of any Tree. There is
  // no block corresponding to World. However, ToFullMatrix() below produces a
  // Jacobian indexed by Mobod rather than Tree, and World is Mobod 0 so does
  // show up in that matrix for consistent numbering (as 6 rows of zero).
  int row_count = 6, col_count = 0;
  for (const SpanningForest::Tree& tree : forest.trees()) {
    const int n = 6 * tree.num_mobods();
    const int m = tree.nv();
    Eigen::MatrixX<T>& J = block_system_jacobian_[tree.index()];
    J.resize(n, m);
    J.setZero();
    row_count += n;
    col_count += m;
  }
  DRAKE_DEMAND(row_count == total_rows_ && col_count == total_cols_);
}

template <typename T>
Eigen::MatrixX<T> BlockSystemJacobianCache<T>::ToFullMatrix() const {
  Eigen::MatrixX<T> full_matrix(total_rows_, total_cols_);
  full_matrix.setZero();
  int next_row = 6, next_col = 0;  // Leave the World row zero.
  for (const Eigen::MatrixX<T>& tree_matrix : block_system_jacobian_) {
    const int n = tree_matrix.rows();
    const int m = tree_matrix.cols();
    full_matrix.block(next_row, next_col, n, m) = tree_matrix;
    next_row += n;
    next_col += m;
  }
  DRAKE_DEMAND(next_row == total_rows_ && next_col == total_cols_);
  return full_matrix;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::BlockSystemJacobianCache);
