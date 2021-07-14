#include "drake/multibody/fixed_fem/dev/permute_tangent_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
Eigen::SparseMatrix<T> PermuteTangentMatrix(
    const Eigen::SparseMatrix<T>& tangent_matrix,
    const std::vector<int>& vertex_permutation) {
  DRAKE_ASSERT(tangent_matrix.rows() == tangent_matrix.cols());
  DRAKE_ASSERT(static_cast<int>(vertex_permutation.size()) * 3 ==
               tangent_matrix.cols());
  const int nv = tangent_matrix.rows();
  Eigen::SparseMatrix<T> permuted_tangent_matrix(nv, nv);
  std::vector<Eigen::Triplet<T>> triplets;
  triplets.reserve(tangent_matrix.nonZeros());
  using InnerIterator = typename Eigen::SparseMatrix<T>::InnerIterator;
  for (int i = 0; i < tangent_matrix.outerSize(); ++i) {
    for (InnerIterator it(tangent_matrix, i); it; ++it) {
      const int row_dim = it.row() % 3;
      const int col_dim = it.col() % 3;
      const int row_vertex = it.row() / 3;
      const int col_vertex = it.col() / 3;
      const int new_row_vertex = vertex_permutation[row_vertex];
      const int new_col_vertex = vertex_permutation[col_vertex];
      const int new_row = new_row_vertex * 3 + row_dim;
      const int new_col = new_col_vertex * 3 + col_dim;
      triplets.emplace_back(new_row, new_col, it.value());
    }
  }
  /* The permuted matrix should have the exact same number of non-zero entries
   as the old matrix. */
  DRAKE_DEMAND(static_cast<int>(triplets.size()) == tangent_matrix.nonZeros());
  permuted_tangent_matrix.setFromTriplets(triplets.begin(), triplets.end());
  return permuted_tangent_matrix;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&PermuteTangentMatrix<T>))

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
