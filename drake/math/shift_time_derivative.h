#pragma once

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
template <typename v_Type, typename DB_v_Type, typename w_AB_Type>
Vector3<typename v_Type::Scalar> ShiftTimeDerivative(
    const Eigen::MatrixBase<v_Type>& v_E,
    const Eigen::MatrixBase<DB_v_Type>& DB_v_E,
    const Eigen::MatrixBase<w_AB_Type>& w_AB_E) {
  // All input vectors must be three dimensional vectors.
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<v_Type>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DB_v_Type>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<w_AB_Type>, 3);
  typedef typename v_Type::Scalar T;
  // All input vectors must be templated on the same scalar type.
  static_assert(std::is_same<typename DB_v_Type::Scalar, T>::value,
                "DB_v_E must be templated on the same scalar type as v_E");
  static_assert(std::is_same<typename w_AB_Type::Scalar, T>::value,
                "w_AB_E must be templated on the same scalar type as v_E");
  return DB_v_E + w_AB_E.cross(v_E);
}

}  // namespace math
}  // namespace drake
