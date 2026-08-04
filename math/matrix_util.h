#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
/// Determines if a matrix is symmetric. If std::equal_to<>()(matrix(i, j),
/// matrix(j, i)) is true for all i, j, then the matrix is symmetric.
template <typename Derived>
bool IsSymmetric(const Eigen::MatrixBase<Derived>& matrix) {
  using DerivedScalar = typename Derived::Scalar;
  if (matrix.rows() != matrix.cols()) {
    return false;
  }
  for (int i = 0; i < static_cast<int>(matrix.rows()); ++i) {
    for (int j = i + 1; j < static_cast<int>(matrix.cols()); ++j) {
      if (!std::equal_to<DerivedScalar>()(matrix(i, j), matrix(j, i))) {
        return false;
      }
    }
  }
  return true;
}

/// Determines if a matrix is symmetric based on whether the difference between
/// matrix(i, j) and matrix(j, i) is smaller than @p precision for all i, j.
/// The precision is absolute.
/// Matrix with nan or inf entries is not allowed.
template <typename Derived>
bool IsSymmetric(const Eigen::MatrixBase<Derived>& matrix,
                 const typename Derived::Scalar& precision) {
  if (!std::isfinite(precision)) {
    throw std::runtime_error("Cannot accept nans or inf is IsSymmetric");
  }
  using DerivedScalar = typename Derived::Scalar;
  if (matrix.rows() != matrix.cols()) {
    return false;
  }
  for (int i = 0; i < static_cast<int>(matrix.rows()); ++i) {
    if (!std::isfinite(matrix(i, i))) {
      throw std::runtime_error("Cannot accept nans or inf is IsSymmetric");
    }
    for (int j = i + 1; j < static_cast<int>(matrix.rows()); ++j) {
      if (!std::isfinite(matrix(i, j)) || !std::isfinite(matrix(j, i))) {
        throw std::runtime_error("Cannot accept nans or inf is IsSymmetric");
      }
      DerivedScalar diff = matrix(i, j) - matrix(j, i);
      if (!Eigen::NumTraits<DerivedScalar>::IsSigned) {
        if (diff > precision) {
          return false;
        }
      } else if (diff > precision || -diff > precision) {
        return false;
      }
    }
  }
  return true;
}

namespace internal {
template <typename Derived1, typename Derived2>
void to_symmetric_matrix_from_lower_triangular_columns_impl(
    int rows, const Eigen::MatrixBase<Derived1>& lower_triangular_columns,
    Eigen::MatrixBase<Derived2>* symmetric_matrix) {
  int count = 0;
  for (int j = 0; j < rows; ++j) {
    (*symmetric_matrix)(j, j) = lower_triangular_columns(count);
    ++count;
    for (int i = j + 1; i < rows; ++i) {
      (*symmetric_matrix)(i, j) = lower_triangular_columns(count);
      (*symmetric_matrix)(j, i) = lower_triangular_columns(count);
      ++count;
    }
  }
}
}  // namespace internal

/// Given a column vector containing the stacked columns of the lower triangular
/// part of a square matrix, returning a symmetric matrix whose lower
/// triangular part is the same as the original matrix.
/// @pydrake_mkdoc_identifier{dynamic_size}
template <typename Derived>
drake::MatrixX<typename Derived::Scalar>
ToSymmetricMatrixFromLowerTriangularColumns(
    const Eigen::MatrixBase<Derived>& lower_triangular_columns) {
  int rows = (-1 + sqrt(1 + 8 * lower_triangular_columns.rows())) / 2;

  DRAKE_ASSERT(rows * (rows + 1) / 2 == lower_triangular_columns.rows());
  DRAKE_ASSERT(lower_triangular_columns.cols() == 1);

  drake::MatrixX<typename Derived::Scalar> symmetric_matrix(rows, rows);

  internal::to_symmetric_matrix_from_lower_triangular_columns_impl(
      rows, lower_triangular_columns, &symmetric_matrix);
  return symmetric_matrix;
}

/// Given a column vector containing the stacked columns of the lower triangular
/// part of a square matrix, returning a symmetric matrix whose lower
/// triangular part is the same as the original matrix.
/// @tparam rows The number of rows in the symmetric matrix.
template <int rows, typename Derived>
Eigen::Matrix<typename Derived::Scalar, rows, rows>
ToSymmetricMatrixFromLowerTriangularColumns(
    const Eigen::MatrixBase<Derived>& lower_triangular_columns) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, rows * (rows + 1) / 2);

  Eigen::Matrix<typename Derived::Scalar, rows, rows> symmetric_matrix(rows,
                                                                       rows);

  internal::to_symmetric_matrix_from_lower_triangular_columns_impl(
      rows, lower_triangular_columns, &symmetric_matrix);
  return symmetric_matrix;
}

/// Given a square matrix, extract the lower triangular part as a stacked column
/// vector. This is a particularly useful operation when vectorizing symmetric
/// matrices.
template <typename Derived>
drake::VectorX<typename Derived::Scalar> ToLowerTriangularColumnsFromMatrix(
    const Eigen::MatrixBase<Derived>& matrix) {
  DRAKE_ASSERT(matrix.rows() == matrix.cols());
  const int num_rows = (matrix.rows() * (matrix.rows() + 1)) / 2;
  drake::VectorX<typename Derived::Scalar> stacked_lower_triangular(num_rows);
  int row_count = 0;
  for (int c = 0; c < matrix.cols(); ++c) {
    const int num_elts = matrix.cols() - c;
    stacked_lower_triangular.segment(row_count, num_elts) =
        matrix.col(c).tail(num_elts);
    row_count += num_elts;
  }
  return stacked_lower_triangular;
}

/// Checks if a matrix is symmetric (with tolerance @p symmetry_tolerance --
/// see IsSymmetric) and has all eigenvalues greater than @p
/// eigenvalue_tolerance.  @p eigenvalue_tolerance must be >= 0 -- where 0
/// implies positive semi-definite (but is of course subject to all of the
/// pitfalls of floating point).
///
/// To consider the numerical robustness of the eigenvalue estimation, we
/// specifically check that min_eigenvalue >= eigenvalue_tolerance * max(1,
/// max_abs_eigenvalue).
template <typename Derived>
bool IsPositiveDefinite(const Eigen::MatrixBase<Derived>& matrix,
                        double eigenvalue_tolerance = 0.0,
                        double symmetry_tolerance = 0.0) {
  DRAKE_DEMAND(eigenvalue_tolerance >= 0);
  DRAKE_DEMAND(symmetry_tolerance >= 0);
  if (!IsSymmetric(matrix, symmetry_tolerance)) return false;

  // Note: Eigen's documentation clearly warns against using the faster LDLT
  // for this purpose, as the algorithm cannot handle indefinite matrices.
  Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> eigensolver(
      matrix);
  DRAKE_THROW_UNLESS(eigensolver.info() == Eigen::Success);
  // According to the Lapack manual, the absolute accuracy of eigenvalues is
  // eps*max(|eigenvalues|), so I will write my tolerances relative to that.
  // Anderson et al., Lapack User's Guide, 3rd ed. section 4.7, 1999.
  const double max_abs_eigenvalue =
      eigensolver.eigenvalues().cwiseAbs().maxCoeff();
  return eigensolver.eigenvalues().minCoeff() >=
         eigenvalue_tolerance * std::max(1., max_abs_eigenvalue);
}

/// Converts a MatrixX<T> into a std::vector<MatrixX<T>>, taking each column of
/// the m-by-n matrix `mat` into an m-by-1 element of the returned std::vector.
template <typename T>
std::vector<MatrixX<T>> EigenToStdVector(
    const Eigen::Ref<const MatrixX<T>>& mat) {
  std::vector<MatrixX<T>> vec(mat.cols());
  for (int i = 0; i < mat.cols(); ++i) {
    vec[i] = mat.col(i);
  }
  return vec;
}

/// Converts a std::vector<MatrixX<T>> into a MatrixX<T>, composing each
/// element of `vec` into a column of the returned matrix.
///
/// @pre all elements of `vec` must have one column and the same number of
/// rows.
template <typename T>
MatrixX<T> StdVectorToEigen(const std::vector<MatrixX<T>>& vec) {
  DRAKE_DEMAND(vec.size() > 0);
  MatrixX<T> mat(vec[0].rows(), vec.size());
  for (int i = 0; i < static_cast<int>(vec.size()); ++i) {
    DRAKE_DEMAND(vec[i].rows() == mat.rows());
    DRAKE_DEMAND(vec[i].cols() == 1);
    mat.col(i) = vec[i];
  }
  return mat;
}

/// Extracts the principal submatrix from the ordered set of indices. The
/// indices must be in monotonically increasing order and non-empty. This method
/// makes no assumptions about the symmetry of the matrix, nor that the matrix
/// is square. However, all indices must be valid for both rows and columns.
template <typename Derived>
MatrixX<typename Derived::Scalar> ExtractPrincipalSubmatrix(
    const Eigen::MatrixBase<Derived>& mat, const std::set<int>& indices) {
  DRAKE_THROW_UNLESS(!indices.empty());
  // Stores the contiguous intervals of the index set of the principal
  // submatrix. These intervals include the first index but exclude the last,
  // i.e. [intervals[i][0], intervals[i][1]).
  std::vector<std::pair<int, int>> intervals;
  auto it = indices.begin();
  int interval_start{*it};
  int interval_end{*it};
  DRAKE_THROW_UNLESS(interval_start >= 0 && *it < mat.rows() &&
                     *it < mat.cols());
  // start iterating at the second element in the set.
  for (++it; it != indices.end(); ++it) {
    DRAKE_THROW_UNLESS(*it < mat.rows() && *it < mat.cols());
    if (*it != interval_end + 1) {
      intervals.emplace_back(interval_start, interval_end + 1);
      interval_start = *it;
    }
    interval_end = *it;
  }
  intervals.emplace_back(interval_start, interval_end + 1);

  MatrixX<typename Derived::Scalar> minor(indices.size(), indices.size());
  int minor_row_count = 0;
  for (auto row_it = intervals.begin(); row_it != intervals.cend(); ++row_it) {
    int minor_col_count = 0;
    const int cur_block_row_size = row_it->second - row_it->first;
    for (auto col_it = intervals.begin(); col_it != intervals.cend();
         ++col_it) {
      const int cur_block_col_size = col_it->second - col_it->first;
      minor.block(minor_row_count, minor_col_count, cur_block_row_size,
                  cur_block_col_size) =
          mat.block(row_it->first, col_it->first, cur_block_row_size,
                    cur_block_col_size);
      minor_col_count += cur_block_col_size;
    }
    minor_row_count += cur_block_row_size;
  }
  return minor;
}

/// Returns the python statement to construct scipy.sparse matrix. The generated
/// code will call sparse.csc_matrix() directly (please make sure you have
/// imported the module through `from scipy import sparse`), and will end with a
/// newline.
/// @param mat The Eigen matrix to be generated to python code.
/// @param name The name of the python variable for the sparse matrix.
/// @returns the generated python code
std::string GeneratePythonCsc(const Eigen::SparseMatrix<double>& mat,
                              std::string_view name);

}  // namespace math
}  // namespace drake
