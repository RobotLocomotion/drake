#include <Eigen/Dense>

# include <functional>

#include "drake/common/number_traits.h"

namespace drake {
namespace math {
/// Determines if a matrix is symmetric. If std::equal_to<>()(matrix(i, j),
/// matrix(j, i)) is
/// true for all i, j, then the matrix is symmetric.
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

/// Determines if a matrix is symmetric. If the difference between matrix(i, j)
/// and matrix(j, i) is smaller than precision for all i, j.
/// The precision is absolute.
template <typename Derived>
bool IsSymmetric(const Eigen::MatrixBase<Derived>& matrix,
                 const typename Derived::Scalar& precision) {
  if (std::isnan(precision) || std::isinf(precision)) {
    throw std::runtime_error("Cannot accept nans or inf is IsSymmetric");
  }
  using DerivedScalar = typename Derived::Scalar;
  if (matrix.rows() != matrix.cols()) {
    return false;
  }
  for (int i = 0; i < static_cast<int>(matrix.rows()); ++i) {
    if (std::isnan(matrix(i, i)) || std::isinf(matrix(i, i))) {
      throw std::runtime_error("Cannot accept nans or inf is IsSymmetric");
    }
    for (int j = i + 1; j < static_cast<int>(matrix.rows()); ++j) {
      if (std::isnan(matrix(i, j)) || std::isinf(matrix(i, j)) ||
          std::isnan(matrix(j, i)) || std::isinf(matrix(j, i))) {
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
}  // namespace math
}  // namespace drake
