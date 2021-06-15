#pragma once

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Given ᴮd/dt(v) (the time derivative in frame B of an arbitrary 3D vector v)
/// and given ᴬωᴮ (frame B's angular velocity in another frame A), this method
/// computes ᴬd/dt(v) (the time derivative in frame A of v) by:
/// ᴬd/dt(v) = ᴮd/dt(v) + ᴬωᴮ x v
///
/// This mathematical operation is known as the "Transport Theorem" or the
/// "Golden Rule for Vector Differentiation" [Mitiguy 2016, §7.3]. It was
/// discovered by Euler in 1758. Its explicit notation with superscript
/// frames was invented by Thomas Kane in 1950. Its use as the defining
/// property of angular velocity was by Mitiguy in 1993.
///
/// In source code and comments, we use the following monogram notations:
/// DtA_v = ᴬd/dt(v) denotes the time derivative in frame A of the vector v.
/// DtA_v_E = [ᴬd/dt(v)]_E denotes the time derivative in frame A of vector v,
/// with the resulting new vector quantity expressed in a frame E.
///
/// In source code, this mathematical operation is performed with all vectors
/// expressed in the same frame E as [ᴬd/dt(v)]ₑ = [ᴮd/dt(v)]ₑ + [ᴬωᴮ]ₑ x [v]ₑ
/// which in monogram notation is: <pre>
///   DtA_v_E = DtB_v_E + w_AB_E x v_E
/// </pre>
///
/// [Mitiguy 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion Simulation.
template <typename v_Type, typename DtB_v_Type, typename w_AB_Type>
Vector3<typename v_Type::Scalar> ConvertTimeDerivativeToOtherFrame(
    const Eigen::MatrixBase<v_Type>& v_E,
    const Eigen::MatrixBase<DtB_v_Type>& DtB_v_E,
    const Eigen::MatrixBase<w_AB_Type>& w_AB_E) {
  // All input vectors must be three dimensional vectors.
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<v_Type>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DtB_v_Type>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<w_AB_Type>, 3);
  typedef typename v_Type::Scalar T;
  // All input vectors must be templated on the same scalar type.
  static_assert(std::is_same_v<typename DtB_v_Type::Scalar, T>,
                "DtB_v_E must be templated on the same scalar type as v_E");
  static_assert(std::is_same_v<typename w_AB_Type::Scalar, T>,
                "w_AB_E must be templated on the same scalar type as v_E");
  return DtB_v_E + w_AB_E.cross(v_E);
}

}  // namespace math
}  // namespace drake
