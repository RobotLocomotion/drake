#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
/* Calculates an approximation of the condition number for the given matrix A.
@tparam_nonsymbolic_scalar T. */
template <typename T>
double CalcConditionNumber(const Eigen::Ref<const MatrixX<T>>& A) {
  Eigen::JacobiSVD<MatrixX<T>> svd(A);
  /* Prevents division by zero for singular matrix. */
  const T epsilon = 1e-14;
  const T cond =
      svd.singularValues()(0) /
      (svd.singularValues()(svd.singularValues().size() - 1) + epsilon);
  if constexpr (std::is_same_v<T, double>) {
    return cond;
  } else {
    return cond.value();
  }
}
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
