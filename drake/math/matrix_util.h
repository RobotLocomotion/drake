#include <Eigen/Dense>

namespace drake {
namespace math {
template<typename Derived>
bool IsSymmetric(const Eigen::MatrixBase<Derived>& matrix, const typename Derived::Scalar& precision) {
  if (matrix.rows() != matrix.cols()) {return false;}
  for (int i = 0; i < static_cast<int>(matrix.rows()); ++i) {
    for (int j = i + 1; j < static_cast<int>(matrix.rows()); ++j) {
      typename Derived::Scalar diff = matrix(i, j) - matrix(j, i);
      if (diff > precision || -diff > precision) {
        return false;
      }
    }
  }
  return true;
}
} // namespace math
} // namespace drake