#include <Eigen/Dense>

namespace drake {
namespace math {
// Determines if a matrix is symmetric.
template <typename Derived>
bool IsSymmetric(const Eigen::MatrixBase<Derived>& matrix) {
  static_assert(Eigen::NumTraits<typename Derived::Scalar>::IsInteger,
      "The scalar is not an integer type, use IsSymmetric(matrix, precision) instead.");
  if (matrix.rows() != matrix.cols()) { return false;}
  for (int i = 0; i < static_cast<int>(matrix.rows()); ++i) {
    for (int j = 0; j < static_cast<int>(matrix.cols()); ++j) {
      if (matrix(i, j) != matrix(j, i)) {return false;}
    }
  }
  return true;
}

/// Determines if a matrix is symmetric. If the difference between matrix(i, j)
///
template<typename Derived>
bool IsSymmetric(const Eigen::MatrixBase<Derived>& matrix, const typename Derived::Scalar& precision) {
  using DerivedScalar = typename Derived::Scalar;
  if (matrix.rows() != matrix.cols()) {return false;}
  for (int i = 0; i < static_cast<int>(matrix.rows()); ++i) {
    for (int j = i + 1; j < static_cast<int>(matrix.rows()); ++j) {
      DerivedScalar diff = matrix(i, j) - matrix(j, i);
      if (!Eigen::NumTraits<DerivedScalar>::IsSigned) {
        if (diff > precision) {
          return false;
        }
      }
      else if (diff > precision || -diff > precision) {
        return false;
      }
    }
  }
  return true;
}
} // namespace math
} // namespace drake