#include "drake/math/quaternion.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace math {

template <typename T>
boolean<T> AreQuaternionsEqualForOrientation(const Eigen::Quaternion<T>& quat1,
                                             const Eigen::Quaternion<T>& quat2,
                                             const T tolerance) {
  const Eigen::Quaternion<T> quat1_canonical = QuaternionToCanonicalForm(quat1);
  const Eigen::Quaternion<T> quat2_canonical = QuaternionToCanonicalForm(quat2);
  return (quat1_canonical.coeffs() - quat2_canonical.coeffs())
             .template lpNorm<Eigen::Infinity>() <= tolerance;
}

template <typename T>
Vector4<T> CalculateQuaternionDtFromAngularVelocityExpressedInB(
    const Eigen::Quaternion<T>& quat_AB, const Vector3<T>& w_AB_B) {
  const T e0 = quat_AB.w(), e1 = quat_AB.x(), e2 = quat_AB.y(),
          e3 = quat_AB.z();
  const T wx = w_AB_B[0], wy = w_AB_B[1], wz = w_AB_B[2];

  const T e0Dt = (-e1 * wx - e2 * wy - e3 * wz) / 2;
  const T e1Dt = (e0 * wx - e3 * wy + e2 * wz) / 2;
  const T e2Dt = (e3 * wx + e0 * wy - e1 * wz) / 2;
  const T e3Dt = (-e2 * wx + e1 * wy + e0 * wz) / 2;

  return Vector4<T>(e0Dt, e1Dt, e2Dt, e3Dt);
}

template <typename T>
Vector3<T> CalculateAngularVelocityExpressedInBFromQuaternionDt(
    const Eigen::Quaternion<T>& quat_AB, const Vector4<T>& quatDt) {
  const T e0 = quat_AB.w(), e1 = quat_AB.x(), e2 = quat_AB.y(),
          e3 = quat_AB.z();
  const T e0Dt = quatDt[0], e1Dt = quatDt[1], e2Dt = quatDt[2],
          e3Dt = quatDt[3];

  const T wx = 2 * (-e1 * e0Dt + e0 * e1Dt + e3 * e2Dt - e2 * e3Dt);
  const T wy = 2 * (-e2 * e0Dt - e3 * e1Dt + e0 * e2Dt + e1 * e3Dt);
  const T wz = 2 * (-e3 * e0Dt + e2 * e1Dt - e1 * e2Dt + e0 * e3Dt);

  return Vector3<T>(wx, wy, wz);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&AreQuaternionsEqualForOrientation<T>,
     &CalculateQuaternionDtFromAngularVelocityExpressedInB<T>,
     &CalculateAngularVelocityExpressedInBFromQuaternionDt<T>));

}  // namespace math
}  // namespace drake
