#include "drake/math/roll_pitch_yaw.h"

#include <string>

#include <fmt/format.h>

#include "drake/common/cond.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {

template <typename T>
RotationMatrix<T> RollPitchYaw<T>::ToRotationMatrix() const {
  return RotationMatrix<T>(*this);
}

// Uses a quaternion and its associated rotation matrix `R` to accurately
// and efficiently calculate the roll-pitch-yaw angles (SpaceXYZ Euler angles)
// that underlie `this` @RollPitchYaw, even when the pitch angle p is very
// near a singularity (e.g., when p is within 1E-6 of π/2 or -π/2).
// @param[in] quaternion unit quaternion with elements `[e0, e1, e2, e3]`.
// @param[in] R The %RotationMatrix corresponding to `quaternion`.
// @return [r, p, y] with `-π <= r <= π`, `-π/2 <= p <= π/2, `-π <= y <= π`.
// @note The caller of this function is responsible for ensuring `quaternion`
// satisfies `e0^2 + e1^2 + e2^2 + e3^2 = 1` and that the matrix `R` is the
// rotation matrix that corresponds to `quaternion'.
//-----------------------------------------------------------------------------
// <h3>Theory</h3>
//
// This algorithm was created October 2016 by Paul Mitiguy for TRI (Toyota).
// We believe this is a new algorithm (not previously published).
// Some theory/formulation of this algorithm is provided below.  More detail
// is in Chapter 6 Rotation Matrices II [Mitiguy 2017] (reference below).
// <pre>
// Notation: Angles q1, q2, q3 designate SpaceXYZ "roll, pitch, yaw" angles.
//           A quaternion can be defined in terms of an angle-axis rotation by
//           an angle `theta` about a unit vector `lambda`.  For example,
//           consider right-handed orthogonal unit vectors Ax, Ay, Az and
//           Bx, By, Bz fixed in a frame A and a rigid body B, respectively.
//           Initially, Bx = Ax, By = Ay, Bz = Az, then B is subjected to a
//           right-handed rotation relative to frame A by an angle `theta`
//           about `lambda = L1*Ax + L2*Ay + L3*Az = L1*Bx + L2*By + L3*Bz`.
//           The elements of `quaternion` are defined e0, e1, e2, e3 as
//           `e0 = cos(theta/2)`, `e1 = L1*sin(theta/2)`,
//           `e2 = L2*sin(theta/2)`, `e3 = L3*sin(theta/2)`.
//
// Step 1. The 3x3 rotation matrix R is only used (in conjunction with the
//         atan2 function) to accurately calculate the pitch angle q2.
//         Note: Since only 5 elements of R are used, the algorithm could be
//         made slightly more efficient by computing/passing only those 5
//         elements (e.g., not calculating the other 4 elements) and/or
//         manipulating the relationship between `R` and quaternion to
//         further reduce calculations.
//
// Step 2. Realize the quaternion passed to the function can be regarded as
//         resulting from multiplication of certain 4x4 and 4x1 matrices, or
//         multiplying three rotation quaternions (Hamilton product), to give:
//         e0 = sin(q1/2)*sin(q2/2)*sin(q3/2) + cos(q1/2)*cos(q2/2)*cos(q3/2)
//         e1 = sin(q3/2)*cos(q1/2)*cos(q2/2) - sin(q1/2)*sin(q2/2)*cos(q3/2)
//         e2 = sin(q1/2)*sin(q3/2)*cos(q2/2) + sin(q2/2)*cos(q1/2)*cos(q3/2)
//         e3 = sin(q1/2)*cos(q2/2)*cos(q3/2) - sin(q2/2)*sin(q3/2)*cos(q1/2)
//
//         Reference for step 2: Chapter 6 Rotation Matrices II [Mitiguy 2017]
//
// Step 3.  Since q2 has already been calculated (in Step 1), substitute
//          cos(q2/2) = A and sin(q2/2) = f*A.
//          Note: The final results are independent of A and f = tan(q2/2).
//          Note: -pi/2 <= q2 <= pi/2  so -0.707 <= [A = cos(q2/2)] <= 0.707
//          and  -1 <= [f = tan(q2/2)] <= 1.
//
// Step 4.  Referring to Step 2 form: (1+f)*e1 + (1+f)*e3 and rearrange to:
//          sin(q1/2+q3/2) = (e1+e3)/(A*(1-f))
//
//          Referring to Step 2 form: (1+f)*e0 - (1+f)*e2 and rearrange to:
//          cos(q1/2+q3/2) = (e0-e2)/(A*(1-f))
//
//          Combine the two previous results to produce:
//          1/2*( q1 + q3 ) = atan2( e1+e3, e0-e2 )
//
// Step 5.  Referring to Step 2 form: (1-f)*e1 - (1-f)*e3 and rearrange to:
//          sin(q1/5-q3/5) = -(e1-e3)/(A*(1+f))
//
//          Referring to Step 2 form: (1-f)*e0 + (1-f)*e2 and rearrange to:
//          cos(q1/2-q3/2) = (e0+e2)/(A*(1+f))
//
//          Combine the two previous results to produce:
//          1/2*( q1 - q3 ) = atan2( e3-e1, e0+e2 )
//
// Step 6.  Combine Steps 4 and 5 and solve the linear equations for q1, q3.
//          Use zA, zB to handle case in which both atan2 arguments are 0.
//          zA = (e1+e3==0  &&  e0-e2==0) ? 0 : atan2( e1+e3, e0-e2 );
//          zB = (e3-e1==0  &&  e0+e2==0) ? 0 : atan2( e3-e1, e0+e2 );
//          Solve: 1/2*( q1 + q3 ) = zA     To produce:  q1 = zA + zB
//                 1/2*( q1 - q3 ) = zB                  q3 = zA - zB
//
// Step 7.  As necessary, modify angles by 2*PI to return angles in range:
//          -pi   <= q1 <= pi
//          -pi/2 <= q2 <= pi/2
//          -pi   <= q3 <= pi
//
// [Mitiguy, 2017]: "Advanced Dynamics and Motion Simulation,
//                   For professional engineers and scientists,"
//                   Prodigy Press, Sunnyvale CA, 2017 (Paul Mitiguy).
//                   Available at www.MotionGenesis.com
// </pre>
// @note This algorithm is specific to SpaceXYZ (roll-pitch-yaw) order.
// It is easily modified for other SpaceIJK and BodyIJI rotation sequences.
// @author Paul Mitiguy
template <typename T>
Vector3<T> CalcRollPitchYawFromQuaternionAndRotationMatrix(
    const Eigen::Quaternion<T>& quaternion, const Matrix3<T>& R) {
  // TODO(14927) This method needs testing with symbolic template type T.
  //  Check if it works or throw a nice exception message.
  using std::abs;
  using std::atan2;
  using std::sqrt;

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
  const auto isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const auto isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const T zA = if_then_else(isSingularA, T{0.0}, atan2(yA, xA));
  const T zB = if_then_else(isSingularB, T{0.0}, atan2(yB, xB));
  T q1 = zA - zB;  // First angle in rotation sequence.
  T q3 = zA + zB;  // Third angle in rotation sequence.

  // If necessary, modify angles q1 and/or q3 to be between -pi and pi.
  q1 = if_then_else(q1 > M_PI, q1 - 2 * M_PI, q1);
  q1 = if_then_else(q1 < -M_PI, q1 + 2 * M_PI, q1);
  q3 = if_then_else(q3 > M_PI, q3 - 2 * M_PI, q3);
  q3 = if_then_else(q3 < -M_PI, q3 + 2 * M_PI, q3);

  // Return in Drake/ROS conventional SpaceXYZ q1, q2, q3 (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX q3, q2, q1 order).
  return Vector3<T>(q1, q2, q3);
}

template <typename T>
Eigen::Quaternion<T> RollPitchYaw<T>::ToQuaternion() const {
  using std::cos;
  using std::sin;
  const T q0Half = roll_pitch_yaw_(0) / 2;
  const T q1Half = roll_pitch_yaw_(1) / 2;
  const T q2Half = roll_pitch_yaw_(2) / 2;
  const T c0 = cos(q0Half), s0 = sin(q0Half);
  const T c1 = cos(q1Half), s1 = sin(q1Half);
  const T c2 = cos(q2Half), s2 = sin(q2Half);
  const T c1_c2 = c1 * c2, s1_c2 = s1 * c2;
  const T s1_s2 = s1 * s2, c1_s2 = c1 * s2;
  const T w = c0 * c1_c2 + s0 * s1_s2;
  const T x = s0 * c1_c2 - c0 * s1_s2;
  const T y = c0 * s1_c2 + s0 * c1_s2;
  const T z = c0 * c1_s2 - s0 * s1_c2;
  return Eigen::Quaternion<T>(w, x, y, z);
}

template <typename T>
Matrix3<T> RollPitchYaw<T>::CalcRotationMatrixDt(
    const Vector3<T>& rpyDt) const {
  // For the rotation matrix R generated by `this` RollPitchYaw, calculate the
  // partial derivatives of R with respect to roll `r`, pitch `p` and yaw `y`.
  Matrix3<T> R_r, R_p, R_y;  // ∂R/∂r, ∂R/∂p, ∂R/∂y
  CalcRotationMatrixDrDpDy(&R_r, &R_p, &R_y);

  // When rotation matrix R is regarded as an implicit function of t as
  // R(r(t), p(t), y(t)), the ordinary derivative of R with respect to t is
  // Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ
  const T rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  return R_r * rDt + R_p * pDt + R_y * yDt;
}

template <typename T>
Vector3<T> RollPitchYaw<T>::CalcRpyDDtFromRpyDtAndAngularAccelInParent(
    const Vector3<T>& rpyDt, const Vector3<T>& alpha_AD_A) const {
  // TODO(Mitiguy) Improve accuracy when `cos(p) ≈ 0`.
  // TODO(Mitiguy) Improve speed: The last column of M is (0, 0, 1), the last
  // column of MDt is (0, 0, 1) and there are repeated sin/cos calculations.
  const Matrix3<T> Minv =
      CalcMatrixRelatingRpyDtToAngularVelocityInParent(__func__);
  const Matrix3<T> MDt =
      CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(rpyDt);
  return Minv * (alpha_AD_A - MDt * rpyDt);
}

template <typename T>
Vector3<T> RollPitchYaw<T>::CalcRpyDDtFromAngularAccelInChild(
    const Vector3<T>& rpyDt, const Vector3<T>& alpha_AD_D) const {
  const T& r = roll_angle();
  const T& p = pitch_angle();
  using std::cos;
  using std::sin;
  const T sr = sin(r), cr = cos(r);
  const T sp = sin(p), cp = cos(p);
  if (DoesCosPitchAngleViolateGimbalLockTolerance(cp)) {
    ThrowPitchAngleViolatesGimbalLockTolerance(__func__, p);
  }
  const T one_over_cp = 1.0 / cp;
  const T cr_over_cp = cr * one_over_cp;
  const T sr_over_cp = sr * one_over_cp;
  // clang-format off
  Matrix3<T> M;
  M << 1.0, sr_over_cp * sp, cr_over_cp * sp,
       0.0,              cr,             -sr,
       0.0, sr_over_cp,  cr_over_cp;
  // clang-format on

  // Remainder terms (terms not multiplying α).
  const T tanp = sp * one_over_cp;
  const T rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
  const T pDt_yDt = pDt * yDt;
  const T rDt_pDt = rDt * pDt;
  const Vector3<T> remainder(tanp * rDt_pDt + one_over_cp * pDt_yDt,
                             -cp * rDt * yDt,
                             tanp * pDt_yDt + one_over_cp * rDt_pDt);

  // Combine terms that contains alpha with remainder terms.
  // TODO(Mitiguy) M * alpha_AD_D can be calculated faster since the first
  // column of M is (1, 0, 0).
  return M * alpha_AD_D + remainder;
}

template <typename T>
void RollPitchYaw<T>::CalcRotationMatrixDrDpDy(Matrix3<T>* R_r, Matrix3<T>* R_p,
                                               Matrix3<T>* R_y) const {
  DRAKE_ASSERT(R_r != nullptr && R_p != nullptr && R_y != nullptr);
  const T& r = roll_angle();
  const T& p = pitch_angle();
  const T& y = yaw_angle();
  using std::cos;
  using std::sin;
  const T c0 = cos(r), c1 = cos(p), c2 = cos(y);
  const T s0 = sin(r), s1 = sin(p), s2 = sin(y);
  const T c2_s1 = c2 * s1, s2_s1 = s2 * s1, s2_s0 = s2 * s0, s2_c0 = s2 * c0;
  const T c2_c1 = c2 * c1, s2_c1 = s2 * c1, c2_s0 = c2 * s0, c2_c0 = c2 * c0;
  // clang-format off
  *R_r <<     0,   c2_s1 * c0 + s2_s0,   -c2_s1 * s0 + s2_c0,
              0,   s2_s1 * c0 - c2_s0,   -s2_s1 * s0 - c2_c0,
              0,              c1 * c0,              -c1 * s0;

  *R_p << -c2_s1,          c2_c1 * s0,            c2_c1 * c0,
          -s2_s1,          s2_c1 * s0,            s2_c1 * c0,
             -c1,            -s1 * s0,              -s1 * c0;

  *R_y << -s2_c1,  -s2_s1 * s0 - c2_c0,  -s2_s1 * c0 + c2_s0,
           c2_c1,   c2_s1 * s0 - s2_c0,   c2_s1 * c0 + s2_s0,
               0,                    0,                    0;
  // clang-format on
}

template <typename T>
Matrix3<T> RollPitchYaw<T>::CalcMatrixRelatingAngularVelocityInParentToRpyDt()
    const {
  using std::cos;
  using std::sin;
  const T& p = pitch_angle();
  const T& y = yaw_angle();
  const T sp = sin(p), cp = cos(p);
  const T sy = sin(y), cy = cos(y);
  Matrix3<T> M;
  // clang-format off
  M << cp * cy,  -sy, 0.0,
       cp * sy,   cy, 0.0,
           -sp,  0.0, 1.0;
  // clang-format on
  return M;
}

template <typename T>
Matrix3<T> RollPitchYaw<T>::CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(
    const Vector3<T>& rpyDt) const {
  using std::cos;
  using std::sin;
  const T& p = pitch_angle();
  const T& y = yaw_angle();
  const T sp = sin(p), cp = cos(p);
  const T sy = sin(y), cy = cos(y);
  const T pDt = rpyDt(1);
  const T yDt = rpyDt(2);
  const T sp_pDt = sp * pDt;
  const T cp_yDt = cp * yDt;
  Matrix3<T> M;
  // clang-format off
  M << -cy * sp_pDt - sy * cp_yDt,   -cy * yDt,    0.0,
       -sy * sp_pDt + cy * cp_yDt,   -sy * yDt,    0.0,
                        -cp * pDt,         0.0,    0.0;
  // clang-format on
  return M;
}

template <typename T>
Matrix3<T> RollPitchYaw<T>::CalcMatrixRelatingAngularVelocityInChildToRpyDt()
    const {
  using std::cos;
  using std::sin;
  const T& r = roll_angle();
  const T& p = pitch_angle();
  const T sr = sin(r), cr = cos(r);
  const T sp = sin(p), cp = cos(p);
  Matrix3<T> M;
  // clang-format off
    M << 1.0,   0.0,      -sp,
         0.0,    cr,  sr * cp,
         0.0,   -sr,  cr * cp;
  // clang-format on
  return M;
}

template <typename T>
Matrix3<T> RollPitchYaw<T>::CalcMatrixRelatingRpyDtToAngularVelocityInParent(
    const char* function_name) const {
  using std::cos;
  using std::sin;
  const T& p = pitch_angle();
  const T& y = yaw_angle();
  const T sp = sin(p), cp = cos(p);
  // TODO(Mitiguy) Improve accuracy when `cos(p) ≈ 0`.
  if (scalar_predicate<T>::is_bool &&
      DoesCosPitchAngleViolateGimbalLockTolerance(cp)) {
    ThrowPitchAngleViolatesGimbalLockTolerance(function_name, p);
  }
  const T one_over_cp = 1.0 / cp;
  const T sy = sin(y), cy = cos(y);
  const T cy_over_cp = cy * one_over_cp;
  const T sy_over_cp = sy * one_over_cp;
  Matrix3<T> M;
  // clang-format off
  M <<     cy_over_cp,       sy_over_cp,  0.0,
                  -sy,               cy,  0.0,
      cy_over_cp * sp,  sy_over_cp * sp,  1.0;
  // clang-format on
  return M;
}

template <typename T>
Matrix3<T> RollPitchYaw<T>::CalcMatrixRelatingRpyDtToAngularVelocityInChild(
    const char* function_name) const {
  using std::cos;
  using std::sin;
  const T& p = pitch_angle();
  const T& r = roll_angle();
  const T sp = sin(p), cp = cos(p);
  // TODO(Mitiguy) Improve accuracy when `cos(p) ≈ 0`.
  if (DoesCosPitchAngleViolateGimbalLockTolerance(cp)) {
    ThrowPitchAngleViolatesGimbalLockTolerance(function_name, p);
  }
  const T one_over_cp = 1.0 / cp;
  const T sr = sin(r), cr = cos(r);
  const T cr_over_cp = cr * one_over_cp;
  const T sr_over_cp = sr * one_over_cp;
  Matrix3<T> M;
  // clang-format off
  M << 1.0, sr_over_cp * sp,  cr_over_cp * sp,
       0.0,              cr,              -sr,
       0.0,      sr_over_cp,       cr_over_cp;
  // clang-format on
  return M;
}

template <typename T>
void RollPitchYaw<T>::SetFromRotationMatrix(const RotationMatrix<T>& R) {
  SetFromQuaternionAndRotationMatrix(R.ToQuaternion(), R);
}

template <typename T>
void RollPitchYaw<T>::SetFromQuaternion(
    const Eigen::Quaternion<T>& quaternion) {
  SetFromQuaternionAndRotationMatrix(quaternion, RotationMatrix<T>(quaternion));
}

template <typename T>
void RollPitchYaw<T>::SetFromQuaternionAndRotationMatrix(
    const Eigen::Quaternion<T>& quaternion, const RotationMatrix<T>& R) {
  const Vector3<T> rpy =
      CalcRollPitchYawFromQuaternionAndRotationMatrix(quaternion, R.matrix());
  SetOrThrowIfNotValidInDebugBuild(rpy);

#ifdef DRAKE_ASSERT_IS_ARMED
  // Verify that arguments to this method make sense.  Ensure the
  // rotation_matrix and quaternion correspond to the same orientation.
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  const RotationMatrix<T> R_quaternion(quaternion);
  constexpr double tolerance = 20 * kEpsilon;
  if (scalar_predicate<T>::is_bool &&
      !R_quaternion.IsNearlyEqualTo(R, tolerance)) {
    std::string message = fmt::format(
        "RollPitchYaw::{}():"
        " An element of the RotationMatrix R passed to this method differs by"
        " more than {:G} from the corresponding element of the RotationMatrix"
        " formed by the Quaternion passed to this method.  To avoid this"
        " inconsistency, ensure the orientation of R and Quaternion align.",
        __func__, tolerance);
    throw std::runtime_error(message);
  }

  // This algorithm converts a quaternion and %RotationMatrix to %RollPitchYaw.
  // It is tested by converting the returned %RollPitchYaw to a %RotationMatrix
  // and verifying the rotation matrices are within kEpsilon of each other.
  // Assuming sine, cosine are accurate to 4*(standard double-precision epsilon
  // = 2.22E-16) and there are two sets of two multiplies and one addition for
  // each rotation matrix element, I decided to test with 20 * kEpsilon:
  // (1+4*eps)*(1+4*eps)*(1+4*eps) = 1 + 3*(4*eps) + 3*(4*eps)^2 + (4*eps)^3.
  // Each + or * or sqrt rounds-off, which can introduce 1/2 eps for each.
  // Use: (12*eps) + (4 mults + 1 add) * 1/2 eps = 17.5 eps.
  const RollPitchYaw<T> roll_pitch_yaw(rpy);
  const RotationMatrix<T> R_rpy = RotationMatrix<T>(roll_pitch_yaw);
  DRAKE_ASSERT(R_rpy.IsNearlyEqualTo(R, 20 * kEpsilon));
#endif
}

template <typename T>
boolean<T> RollPitchYaw<T>::IsNearlySameOrientation(
    const RollPitchYaw<T>& other, double tolerance) const {
  // Note: When pitch is close to PI/2 or -PI/2, derivative calculations for
  // Euler angles can encounter numerical problems (dividing by nearly 0).
  // Although values of angles may "jump around" (difficult derivatives), the
  // angles' values should be able to be accurately reproduced.
  const RotationMatrix<T> R1(*this);
  const RotationMatrix<T> R2(other);
  return R1.IsNearlyEqualTo(R2, tolerance);
}

template <typename T>
void RollPitchYaw<T>::ThrowPitchAngleViolatesGimbalLockTolerance(
    const char* function_name, const T& pitch_angle) {
  const double pitch_radians = ExtractDoubleOrThrow(pitch_angle);
  const double cos_pitch_angle = std::cos(pitch_radians);
  DRAKE_ASSERT(DoesCosPitchAngleViolateGimbalLockTolerance(cos_pitch_angle));
  const double tolerance_degrees = GimbalLockPitchAngleTolerance() * 180 / M_PI;
  std::string message = fmt::format(
      "RollPitchYaw::{}():"
      " Pitch angle p = {:G} degrees is within {:G} degrees of gimbal-lock."
      " There is a divide-by-zero error (singularity) at gimbal-lock.  Pitch"
      " angles near gimbal-lock cause numerical inaccuracies.  To avoid this"
      " orientation singularity, use a quaternion -- not RollPitchYaw.",
      function_name, pitch_radians * 180 / M_PI, tolerance_degrees);
  throw std::runtime_error(message);
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const RollPitchYaw<T>& rpy) {
  // Helper to represent an angle as a terse string.  If the angle is symbolic
  // and ends up a string that's too long, return a placeholder instead.
  auto repr = [](const T& angle) {
    std::string result = fmt::to_string(angle);
    if (std::is_same_v<T, symbolic::Expression> && (result.size() >= 30)) {
      result = "<symbolic>";
    }
    return result;
  };
  const T& roll = rpy.roll_angle();
  const T& pitch = rpy.pitch_angle();
  const T& yaw = rpy.yaw_angle();
  out << fmt::format("rpy = {} {} {}", repr(roll), repr(pitch), repr(yaw));
  return out;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const RollPitchYaw<T>&)>(
        &operator<< )
));
// clang-format on

}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RollPitchYaw);
