#include "drake/math/roll_pitch_yaw.h"

#include "drake/common/default_scalars.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {

template <typename T>
RollPitchYaw<T>::RollPitchYaw(const RotationMatrix<T>& R) :
    RollPitchYaw(R.ToQuaternion(), R) {}

template <typename T>
RollPitchYaw<T>::RollPitchYaw(const Eigen::Quaternion<T>& quaternion) :
    RollPitchYaw(quaternion, RotationMatrix<T>(quaternion)) {}

template<typename T>
RollPitchYaw<T>::RollPitchYaw(const Eigen::Quaternion<T>& quaternion,
                              const RotationMatrix<T>& rotation_matrix) {
  const Matrix3<T> R = rotation_matrix.matrix();

  using std::atan2;
  using std::sqrt;
  using std::abs;

  // This algorithm is specific to SpaceXYZ order, including the calculation
  // of q2, the formulas for xA,yA, xB,yB, and values of q1, q3.
  // It is easily modified for other SpaceIJK and BodyIJI rotation sequences.

  // Calculate q2 using lots of information in the rotation matrix.
  // Rsum = abs( cos(q2) ) is inherently non-negative.
  // R20 = -sin(q2) may be negative, zero, or positive.
  const T R22 = R(2, 2);
  const T R21 = R(2, 1);
  const T R10 = R(1, 0);
  const T R00 = R(0, 0);
  const T Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const T R20 = R(2, 0);
  const T q2 = atan2(-R20, Rsum);

  // Calculate q1 and q3 from Steps 2-6 (documented above).
  const T e0 = quaternion.w(), e1 = quaternion.x();
  const T e2 = quaternion.y(), e3 = quaternion.z();
  const T yA = e1 + e3, xA = e0 - e2;
  const T yB = e3 - e1, xB = e0 + e2;
  const T epsilon = Eigen::NumTraits<T>::epsilon();
  const bool isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const bool isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const T zA = isSingularA ? 0.0 : atan2(yA, xA);
  const T zB = isSingularB ? 0.0 : atan2(yB, xB);
  T q1 = zA - zB;  // First angle in rotation sequence.
  T q3 = zA + zB;  // Third angle in rotation sequence.

  // If necessary, modify angles q1 and/or q3 to be between -pi and pi.
  if (q1 > M_PI) q1 = q1 - 2 * M_PI;
  if (q1 < -M_PI) q1 = q1 + 2 * M_PI;
  if (q3 > M_PI) q3 = q3 - 2 * M_PI;
  if (q3 < -M_PI) q3 = q3 + 2 * M_PI;

  // Return in Drake/ROS conventional SpaceXYZ q1, q2, q3 (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX q3, q2, q1 order).
  roll_pitch_yaw_ = Vector3<T>(q1, q2, q3);

#ifdef DRAKE_ASSERT_IS_ARMED
  ThrowIfNotValid(roll_pitch_yaw_);

  // Verify that arguments to this method make sense.  Ensure the
  // rotation_matrix and quaternion correspond to the same orientation.
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const RotationMatrix<T> R_quaternion(quaternion);
  DRAKE_ASSERT(R_quaternion.IsNearlyEqualTo(rotation_matrix, 20 * kEpsilon));

  // This algorithm converts a quaternion and %RotationMatrix to %RollPitchYaw.
  // It is tested by converting the returned %RollPitchYaw to a %RotationMatrix
  // and verifying the rotation matrices are within kEpsilon of each other.
  // Assuming sine, cosine are accurate to 4*(standard double-precision epsilon
  // = 2.22E-16) and there are two sets of two multiplies and one addition for
  // each rotation matrix element, I decided to test with 20 * kEpsilon:
  // (1+4*eps)*(1+4*eps)*(1+4*eps) = 1 + 3*(4*eps) + 3*(4*eps)^2 + (4*eps)^3.
  // Each + or * or sqrt rounds-off, which can introduce 1/2 eps for each.
  // Use: (12*eps) + (4 mults + 1 add) * 1/2 eps = 17.5 eps.
  const RollPitchYaw<T> rpy(roll_pitch_yaw_);
  const RotationMatrix<T> R_rpy = RotationMatrix<T>(rpy);
  DRAKE_ASSERT(R_rpy.IsNearlyEqualTo(rotation_matrix, 20 * kEpsilon));
#endif
}

template <typename T>
bool RollPitchYaw<T>::IsNearlySameOrientation(const RollPitchYaw<T>& other,
                                              double tolerance) const {
  // Note: When pitch is close to PI/2 or -PI/2, derivative calculations for
  // Euler angles can encounter numerical problems (dividing by nearly 0).
  // Although values of angles may "jump around" (difficult derivatives), the
  // angles' values should be able to be accurately reproduced.
  const RotationMatrix<T> R1(*this);
  const RotationMatrix<T> R2(other);
  return R1.IsNearlyEqualTo(R2, tolerance);
}

}  // namespace math
}  // namespace drake

// Explicitly instantiate on non-symbolic scalar types.
// TODO(Mitiguy) Ensure this class handles RollPitchYaw<symbolic::Expression>.
// To enable symbolic expressions, remove _NONSYMBOLIC in next line.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::RollPitchYaw)

