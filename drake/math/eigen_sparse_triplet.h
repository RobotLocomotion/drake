#pragma once

#include <vector>

#include <Eigen/SparseCore>

namespace drake {
namespace math {
/**
 * For a sparse matrix, return the triplet, such that we can reconstruct the
 * matrix using setFromTriplet function
 * @param matrix A sparse matrix
 * @return A triplet with the row, column and value of the non-zero entries.
 * See https://eigen.tuxfamily.org/dox/group__TutorialSparse.html for more
 * information on the triplet
 */
template <typename Derived>
std::vector<Eigen::Triplet<typename Derived::Scalar>> SparseMatrixToTriplets(
    const Derived& matrix) {
  using Scalar = typename Derived::Scalar;
  std::vector<Eigen::Triplet<Scalar>> triplet;
  for (int i = 0; i < matrix.outerSize(); i++) {
    for (typename Derived::InnerIterator it(matrix, i); it; ++it) {
      triplet.push_back(Eigen::Triplet<Scalar>(it.row(), it.col(), it.value()));
    }
  }
  return triplet;
}

/**
 * For a sparse matrix, return the row index, the column index, and value of
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
 * @param[out] row a vector containing the row indices of the non-zero entries
 * @param[out] col a vector containing the column indices of the non-zero
 * entries
 * @param[out] val a vector containing the values of the non-zero entries.
 */
template <typename Derived>
void SparseMatrixToRowColumnValueVectors(
    const Derived& matrix, std::vector<int>& row, std::vector<int>& col,
    std::vector<typename Derived::Scalar>& val) {
  row.clear();
  col.clear();
  val.clear();
  int nnz = matrix.nonZeros();
  row.resize(nnz);
  col.resize(nnz);
  val.resize(nnz);
  int nnz_count = 0;
  for (int i = 0; i < matrix.outerSize(); i++) {
    for (typename Derived::InnerIterator it(matrix, i); it; ++it) {
      row[nnz_count] = it.row();
      col[nnz_count] = it.col();
      val[nnz_count] = it.value();
      nnz_count++;
    }
  }
}
}  // namespace math
}  // namespace drake
