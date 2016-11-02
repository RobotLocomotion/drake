#pragma once

#include <vector>

#include <Eigen/SparseCore>

namespace drake {
namespace math {
/**
 * For a sparse matrix, return a vector of triplets, such that we can
 * reconstruct the matrix using setFromTriplet function
 * @param matrix A sparse matrix
 * @return A triplet with the row, column and value of the non-zero entries.
 * See https://eigen.tuxfamily.org/dox/group__TutorialSparse.html for more
 * information on the triplet
 */
template <typename Derived>
std::vector<Eigen::Triplet<typename Derived::Scalar>> SparseMatrixToTriplets(
    const Derived& matrix) {
  using Scalar = typename Derived::Scalar;
  std::vector<Eigen::Triplet<Scalar>> triplets;
  triplets.reserve(matrix.nonZeros());
  for (int i = 0; i < matrix.outerSize(); i++) {
    for (typename Derived::InnerIterator it(matrix, i); it; ++it) {
      triplets.push_back(
          Eigen::Triplet<Scalar>(it.row(), it.col(), it.value()));
    }
  }
  return triplets;
}

/**
 * For a sparse matrix, return the row indices, the column indices, and value of
 * the non-zero entries.
 * For example, the matrix
 * <!--
 * mat = [1 0 2;
 *       [0 3 4]
 * has row = [0 1 0 1]
 *     col = [0 1 2 2]
 *     val = [1 3 2 4]
 * -->
 * \f[
 * mat = \begin{bmatrix} 1 & 0 & 2\\
 *                       0 & 3 & 4\end{bmatrix}
 * \f]
 * has
 * \f[
 * row = \begin{bmatrix} 0 & 1 & 0 & 1\end{bmatrix}\\
 * col = \begin{bmatrix} 0 & 1 & 2 & 2\end{bmatrix}\\
 * val = \begin{bmatrix} 1 & 3 & 2 & 4\end{bmatrix}
 * \f]
 * @param[in] matrix the input sparse matrix
 * @param[out] row_indices a vector containing the row indices of the
 * non-zero entries
 * @param[out] col_indices a vector containing the column indices of the
 * non-zero entries
 * @param[out] val a vector containing the values of the non-zero entries.
 */
template <typename Derived>
void SparseMatrixToRowColumnValueVectors(
    const Derived& matrix,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::vector<Eigen::Index>& row_indices,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::vector<Eigen::Index>& col_indices,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::vector<typename Derived::Scalar>& val) {
  row_indices.clear();
  col_indices.clear();
  val.clear();
  int nnz = matrix.nonZeros();
  row_indices.reserve(nnz);
  col_indices.reserve(nnz);
  val.reserve(nnz);
  for (int i = 0; i < matrix.outerSize(); i++) {
    for (typename Derived::InnerIterator it(matrix, i); it; ++it) {
      row_indices.push_back(it.row());
      col_indices.push_back(it.col());
      val.push_back(it.value());
    }
  }
}
}  // namespace math
}  // namespace drake
