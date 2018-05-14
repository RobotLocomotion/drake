#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace math {

template <typename T>
class RotationMatrix;

/// This class represents the orientation between two arbitrary frames A and D
/// associated with a Space-fixed (extrinsic) X-Y-Z rotation by "roll-pitch-yaw"
/// angles `[r, p, y]`, which is equivalent to a Body-fixed (intrinsic) Z-Y-X
/// rotation by "yaw-pitch-roll" angles `[y, p, r]`.  The rotation matrix `R_AD`
/// associated with this roll-pitch-yaw `[r, p, y]` rotation sequence is equal
/// to the matrix multiplication shown below.
/// ```
///        ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
/// R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
///        ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
///      =       R_AB          *        R_BC          *        R_CD
/// ```
/// Note: In this discussion, A is the Space frame and D is the Body frame.
/// One way to visualize this rotation sequence is by introducing intermediate
/// frames B and C (useful constructs to understand this rotation sequence).
/// Initially, the frames are aligned so `Di = Ci = Bi = Ai (i = x, y, z)`
/// where Dx, Dy, Dz and Ax, Ay, Az are orthogonal unit vectors fixed in frames
/// D and A respectively. Similarly for Bx, By, Bz and Cx, Cy, Cz in frame B, C.
/// Then D is subjected to successive right-handed rotations relative to A.
/// @li 1st rotation R_CD: %Frame D rotates relative to frames C, B, A by a
/// roll angle `r` about `Dx = Cx`.  Note: D and C are no longer aligned.
/// @li 2nd rotation R_BC: Frames D, C (collectively -- as if welded together)
/// rotate relative to frame B, A by a pitch angle `p` about `Cy = By`.
/// Note: C and B are no longer aligned.
/// @li 3rd rotation R_AB: Frames D, C, B (collectively -- as if welded)
/// rotate relative to frame A by a roll angle `y` about `Bz = Az`.
/// Note: B and A are no longer aligned.
/// The monogram notation for the rotation matrix relating A to D is `R_AD`.
///
/// @see @ref multibody_quantities for monogram notation for dynamics and
/// @ref orientation_discussion "a discussion on rotation matrices".
///
/// @note This class does not store the frames associated with this rotation
/// sequence.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
// TODO(@mitiguy) Add Sherm/Goldstein's way to visualize rotation sequences.
// TODO(Mitiguy) Ensure this class handles RotationMatrix<symbolic::Expression>.
template <typename T>
class RollPitchYaw {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RollPitchYaw)

  /// Constructs a %RollPitchYaw from a 3x1 array of angles.
  /// @param[in] rpy roll, pitch, yaw angles (units of radians).
  /// @throws std::logic_error in debug builds if !IsValid(rpy).
  explicit RollPitchYaw(const Vector3<T>& rpy) {
    SetOrThrowIfNotValidInDebugBuild(rpy);
  }

  /// Constructs a %RollPitchYaw from roll, pitch, yaw angles (radian units).
  /// @param[in] roll x-directed angle in SpaceXYZ rotation sequence.
  /// @param[in] pitch y-directed angle in SpaceXYZ rotation sequence.
  /// @param[in] yaw z-directed angle in SpaceXYZ rotation sequence.
  /// @throws std::logic_error in debug builds if
  /// !IsValid(Vector3<T>(roll, pitch, yaw)).
  RollPitchYaw(const T& roll, const T& pitch, const T& yaw) {
    const Vector3<T> rpy(roll, pitch, yaw);
    SetOrThrowIfNotValidInDebugBuild(rpy);
  }

  /// Constructs a %RollPitchYaw from a %RotationMatrix with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2, `-π <= y <= π`.
  /// @param[in] R a %RotationMatrix.
  /// @note This new high-accuracy algorithm avoids numerical round-off issues
  /// encountered by some algorithms when pitch is within 1E-6 of π/2 or -π/2.
  explicit RollPitchYaw(const RotationMatrix<T>& R);

  /// Constructs a %RollPitchYaw from a %Quaternion with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2, `-π <= y <= π`.
  /// @param[in] quaternion unit %Quaternion.
  /// @note This new high-accuracy algorithm avoids numerical round-off issues
  /// encountered by some algorithms when pitch is within 1E-6 of π/2 or -π/2.
  explicit RollPitchYaw(const Eigen::Quaternion<T>& quaternion);

  /// Returns the Vector3 underlying a %RollPitchYaw.
  const Vector3<T>& vector() const { return roll_pitch_yaw_; }

  /// Returns the roll-angle underlying a %RollPitchYaw.
  const T& get_roll_angle() const { return roll_pitch_yaw_(0); }

  /// Returns the pitch-angle underlying a %RollPitchYaw.
  const T& get_pitch_angle() const { return roll_pitch_yaw_(1); }

  /// Returns the yaw-angle underlying a %RollPitchYaw.
  const T& get_yaw_angle() const { return roll_pitch_yaw_(2); }

  /// Returns a quaternion representation of `this` %RollPitchYaw.
  Eigen::Quaternion<T> ToQuaternion() const {
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

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RollPitchYaw to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if `‖this - other‖∞ <= tolerance`.
  bool IsNearlyEqualTo(const RollPitchYaw<T>& other, double tolerance) const {
    const Vector3<T> difference = vector() - other.vector();
    return difference.template lpNorm<Eigen::Infinity>() <= tolerance;
  }

  /// Compares each element of the %RotationMatrix R1 produced by `this` to the
  /// corresponding element of the %RotationMatrix R2 produced by `other` to
  /// check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RollPitchYaw to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between R1, R2.
  /// @returns `true` if `‖R1 - R2‖∞ <= tolerance`.
  bool IsNearlySameOrientation(const RollPitchYaw<T>& other,
                               double tolerance) const;

  /// Returns true if roll-pitch-yaw angles `[r, p, y]` are the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2, `-π <= y <= π`.
  bool IsRollPitchYawInCanonicalRange() const {
    const T& r = get_roll_angle();
    const T& p = get_pitch_angle();
    const T& y = get_yaw_angle();
    return (-M_PI <= r && r <= M_PI) && (-M_PI / 2 <= p && p <= M_PI / 2) &&
        (-M_PI <= y && y <= M_PI);
  }

  /// Returns true if `rpy` contains valid roll, pitch, yaw angles.
  /// @param[in] rpy allegedly valid roll, pitch, yaw angles.
  /// @note an angle is invalid if it is NaN or infinite.
  static bool IsValid(const Vector3<T>& rpy) { return rpy.allFinite(); }

  /// Forms Ṙ, the ordinary derivative of the %RotationMatrix `R` with respect
  /// to an independent variable `t` (`t` usually denotes time) and `R` is the
  /// %RotationMatrix formed by `this` %RollPitchYaw.  The roll-pitch-yaw angles
  /// r, p, y are regarded as functions of `t` [i.e., r(t), p(t), y(t)].
  /// @param[in] rpyDt Ordinary derivative of rpy with respect to an independent
  /// variable `t` (`t` usually denotes time, but not necessarily).
  /// @returns Ṙ, the ordinary derivative of `R` with respect to `t`, calculated
  /// as Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ.  In other words, the returned
  /// (i, j) element is ∂Rij/∂r * ṙ + ∂Rij/∂p * ṗ + ∂Rij/∂y * ẏ.
  Matrix3<T> OrdinaryDerivativeRotationMatrix(const Vector3<T>& rpyDt) const {
    // For the rotation matrix R generated by `this` RollPitchYaw, calculate the
    // partial derivatives of R with respect to roll `r`, pitch `p` and yaw `y`.
    Matrix3<T> R_r, R_p, R_y;  // ∂R/∂r, ∂R/∂p, ∂R/∂y
    PartialDerivativeRotationMatrix(&R_r, &R_p, &R_y);

    // When rotation matrix R is regarded as an implicit function of t as
    // R(r(t), p(t), y(t)), the ordinary derivative of R with respect to t is
    // Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ
    const T rDt = rpyDt(0), pDt = rpyDt(1), yDt = rpyDt(2);
    return R_r * rDt + R_p * pDt + R_y * yDt;
  }

 private:
  // Constructs roll-pitch-yaw angles (i.e., SpaceXYZ Euler angles) from a
  // quaternion and its associated rotation matrix.
  // @param[in] quaternion unit quaternion with elements [e0, e1, e2, e3].
  // @param[in] R The %RotationMatrix corresponding to `quaternion`.
  // @return %RollPitchYaw containing angles `[r, p, y]` with range
  // `-π <= r <= π`, `-π/2 <= p <= π/2, `-π <= y <= π`.
  //
  // This accurate algorithm avoids numerical round-off issues encountered by
  // some algorithms when pitch angle is within 1E-6 of π/2 or -π/2.
  //
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
  RollPitchYaw(const Eigen::Quaternion<T>& quaternion,
               const RotationMatrix<T>& rotation_matrix);

  // Throws an exception if rpy is not a valid %RollPitchYaw.
  // @param[in] rpy an allegedly valid rotation matrix.
  static void ThrowIfNotValid(const Vector3<T>& rpy) {
    if (!RollPitchYaw<T>::IsValid(rpy)) {
      throw std::logic_error(
       "Error: One (or more) of the roll-pitch-yaw angles is infinity or NaN.");
    }
  }

  // For the %RotationMatrix `R` generated by `this` %RollPitchYaw, this method
  // calculates the partial derivatives of `R` with respect to roll, pitch, yaw.
  // @param[out] R_r ∂R/∂r Partial derivative of `R` with respect to roll `r`.
  // @param[out] R_p ∂R/∂p Partial derivative of `R` with respect to pitch `p`.
  // @param[out] R_y ∂R/∂y Partial derivative of `R` with respect to yaw `y`.
  void PartialDerivativeRotationMatrix(Matrix3<T>* R_r, Matrix3<T>* R_p,
                                       Matrix3<T>* R_y) const {
    DRAKE_ASSERT(R_r != nullptr && R_r != nullptr && R_r != nullptr);
    const T& r = get_roll_angle();
    const T& p = get_pitch_angle();
    const T& y = get_yaw_angle();
    using std::sin;
    using std::cos;
    const T c0 = cos(r), c1 = cos(p), c2 = cos(y);
    const T s0 = sin(r), s1 = sin(p), s2 = sin(y);
    const T c2_s1 = c2 * s1, s2_s1 = s2 * s1, s2_s0 = s2 * s0, s2_c0 = s2 * c0;
    const T c2_c1 = c2 * c1, s2_c1 = s2 * c1, c2_s0 = c2 * s0, c2_c0 = c2 * c0;
    // clang-format on
    *R_r <<     0,   c2_s1 * c0 + s2_s0,   -c2_s1 * s0 + s2_c0,
                0,   s2_s1 * c0 - c2_s0,   -s2_s1 * s0 - c2_c0,
                0,              c1 * c0,              -c1 * s0;

    *R_p << -c2_s1,          c2_c1 * s0,            c2_c1 * c0,
            -s2_s1,          s2_c1 * s0,            s2_c1 * c0,
               -c1,            -s1 * s0,              -s1 * c0;

    *R_y << -s2_c1,  -s2_s1 * s0 - c2_c0,  -s2_s1 * c0 + c2_s0,
             c2_c1,   c2_s1 * s0 - s2_c0,   c2_s1 * c0 + s2_s0,
                 0,                    0,                    0;
    // clang-format off
  }

  /// Sets `this` %RollPitchYaw from a Vector3.
  /// @param[in] rpy allegedly valid roll-pitch-yaw angles.
  /// @throws std::logic_error in debug builds if rpy fails IsValid(rpy).
  void SetOrThrowIfNotValidInDebugBuild(const Vector3<T>& rpy) {
    DRAKE_ASSERT_VOID(ThrowIfNotValid(rpy));
    roll_pitch_yaw_ = rpy;
  }

  // Stores the underlying roll-pitch-yaw angles.
  // There is no default initialization needed.
  Vector3<T> roll_pitch_yaw_;
};

/// (Deprecated), use @ref math::RollPitchYaw(quaternion).
// TODO(mitiguy) Delete this code that was deprecated on April 27, 2018.
template <typename T>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                 "Use constructor RollPitchYaw(quaternion).")
Vector3<T> QuaternionToSpaceXYZ(const Eigen::Quaternion<T>& quaternion) {
  return RollPitchYaw<T>(quaternion).vector();
}

/// (Deprecated), use @ref math::RollPitchYaw(rpy).ToQuaternion().
// TODO(mitiguy) Delete this code that was deprecated on April 16, 2018.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                 "Use RollPitchYaw::ToQuaternion().")
Quaternion<typename Derived::Scalar> RollPitchYawToQuaternion(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  using Scalar = typename Derived::Scalar;
  const RollPitchYaw<Scalar> roll_pitch_yaw(rpy(0), rpy(1), rpy(2));
  const Eigen::Quaternion<Scalar> quaternion = roll_pitch_yaw.ToQuaternion();
}

/// (Deprecated), use @ref math::RollPitchYaw(rpy).ToQuaternion().
// TODO(mitiguy) Delete this code that was deprecated on April 16, 2018.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                 "Use RollPitchYaw::ToQuaternion().")
Vector4<typename Derived::Scalar> rpy2quat(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  using Scalar = typename Derived::Scalar;
  const RollPitchYaw<Scalar> roll_pitch_yaw(rpy(0), rpy(1), rpy(2));
  const Eigen::Quaternion<Scalar> q = roll_pitch_yaw.ToQuaternion();
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

}  // namespace math
}  // namespace drake
