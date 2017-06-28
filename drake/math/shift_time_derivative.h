#pragma once

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Given the time derivative ᴮd/dt(v) of an arbitrary 3D vector v in a frame B
/// moving with angular velocity w_AB with respect to another frame A, this
/// method computes (shifts) the time derivative ᴬd/dt(v) of the same vector v
/// in frame A.
/// Mathematically: <pre>
///   ᴬd/dt(v) = ᴮd/dt(v) + w_AB x v
/// </pre>
///
/// In source code and comments we use the monogram notation
/// `DtA_v_E = [ᴬd/dt(v)]_E` to denote the time derivative of the vector
/// quantity v in a frame A with the resulting new vector quantity expressed in
/// a frame E.
/// To perform this operation numerically all quantities must be expressed in
/// the same frame E. Using the monogram notation: <pre>
///   DtA_v_E = DtB_v_E + w_AB_E x v_E
/// </pre>
///
/// This operation is commonly known as the "Transport Theorem" while
/// [Mitiguy 2016, §7.3] refers to it as to the "Golden Rule for Vector
/// Differentiation".
///
/// [Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
template <typename v_Type, typename DtB_v_Type, typename w_AB_Type>
Vector3<typename v_Type::Scalar> ShiftTimeDerivative(
    const Eigen::MatrixBase<v_Type>& v_E,
    const Eigen::MatrixBase<DtB_v_Type>& DtB_v_E,
    const Eigen::MatrixBase<w_AB_Type>& w_AB_E) {
  // All input vectors must be three dimensional vectors.
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<v_Type>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DtB_v_Type>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<w_AB_Type>, 3);
  typedef typename v_Type::Scalar T;
  // All input vectors must be templated on the same scalar type.
  static_assert(std::is_same<typename DtB_v_Type::Scalar, T>::value,
                "DtB_v_E must be templated on the same scalar type as v_E");
  static_assert(std::is_same<typename w_AB_Type::Scalar, T>::value,
                "w_AB_E must be templated on the same scalar type as v_E");
  return DtB_v_E + w_AB_E.cross(v_E);
}

}  // namespace math
}  // namespace drake
