#pragma once

#include <cmath>
#include <functional>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/number_traits.h"

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
}  // namespace math
}  // namespace drake
