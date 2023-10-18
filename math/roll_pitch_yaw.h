#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/hash.h"

namespace drake {
namespace math {

template <typename T>
class RotationMatrix;

// TODO(@mitiguy) Add Sherm/Goldstein's way to visualize rotation sequences.
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
/// @note In this discussion, A is the Space frame and D is the Body frame.
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
/// rotate relative to frame A by a yaw angle `y` about `Bz = Az`.
/// Note: B and A are no longer aligned.
/// The monogram notation for the rotation matrix relating A to D is `R_AD`.
///
/// @see @ref multibody_quantities for monogram notation for dynamics and
/// @ref orientation_discussion "a discussion on rotation matrices".
///
/// @note This class does not store the frames associated with this rotation
/// sequence.
///
/// @tparam_default_scalar
template <typename T>
class RollPitchYaw {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RollPitchYaw)

  /// Constructs a %RollPitchYaw from a 3x1 array of angles.
  /// @param[in] rpy 3x1 array with roll, pitch, yaw angles (units of radians).
  /// @throws std::exception in debug builds if !IsValid(rpy).
  explicit RollPitchYaw(const Vector3<T>& rpy) { set(rpy); }

  /// Constructs a %RollPitchYaw from roll, pitch, yaw angles (radian units).
  /// @param[in] roll x-directed angle in SpaceXYZ rotation sequence.
  /// @param[in] pitch y-directed angle in SpaceXYZ rotation sequence.
  /// @param[in] yaw z-directed angle in SpaceXYZ rotation sequence.
  /// @throws std::exception in debug builds if
  /// !IsValid(Vector3<T>(roll, pitch, yaw)).
  RollPitchYaw(const T& roll, const T& pitch, const T& yaw) {
    set(roll, pitch, yaw);
  }

  /// Uses a %RotationMatrix to construct a %RollPitchYaw with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  /// @param[in] R a %RotationMatrix.
  /// @note This new high-accuracy algorithm avoids numerical round-off issues
  /// encountered by some algorithms when pitch is within 1E-6 of π/2 or -π/2.
  explicit RollPitchYaw(const RotationMatrix<T>& R) {
    SetFromRotationMatrix(R);
  }

  /// Uses a %Quaternion to construct a %RollPitchYaw with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  /// @param[in] quaternion unit %Quaternion.
  /// @note This new high-accuracy algorithm avoids numerical round-off issues
  /// encountered by some algorithms when pitch is within 1E-6 of π/2 or -π/2.
  /// @throws std::exception in debug builds if !IsValid(rpy).
  explicit RollPitchYaw(const Eigen::Quaternion<T>& quaternion) {
    SetFromQuaternion(quaternion);
  }

  /// Sets `this` %RollPitchYaw from a 3x1 array of angles.
  /// @param[in] rpy 3x1 array with roll, pitch, yaw angles (units of radians).
  /// @throws std::exception in debug builds if !IsValid(rpy).
  RollPitchYaw<T>& set(const Vector3<T>& rpy) {
    return SetOrThrowIfNotValidInDebugBuild(rpy);
  }

  /// Sets `this` %RollPitchYaw from roll, pitch, yaw angles (units of radians).
  /// @param[in] roll x-directed angle in SpaceXYZ rotation sequence.
  /// @param[in] pitch y-directed angle in SpaceXYZ rotation sequence.
  /// @param[in] yaw z-directed angle in SpaceXYZ rotation sequence.
  /// @throws std::exception in debug builds if
  /// !IsValid(Vector3<T>(roll, pitch, yaw)).
  RollPitchYaw<T>& set(const T& roll, const T& pitch, const T& yaw) {
    return set(Vector3<T>(roll, pitch, yaw));
  }

  /// Uses a %Quaternion to set `this` %RollPitchYaw with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  /// @param[in] quaternion unit %Quaternion.
  /// @note This new high-accuracy algorithm avoids numerical round-off issues
  /// encountered by some algorithms when pitch is within 1E-6 of π/2 or -π/2.
  /// @throws std::exception in debug builds if !IsValid(rpy).
  void SetFromQuaternion(const Eigen::Quaternion<T>& quaternion);

  /// Uses a high-accuracy efficient algorithm to set the roll-pitch-yaw
  /// angles `[r, p, y]` that underlie `this` @RollPitchYaw, even when the pitch
  /// angle p is very near a singularity (when p is within 1E-6 of π/2 or -π/2).
  /// After calling this method, the underlying roll-pitch-yaw `[r, p, y]` has
  /// range `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  /// @param[in] R a %RotationMatrix.
  /// @note This high-accuracy algorithm was invented at TRI in October 2016 and
  /// avoids numerical round-off issues encountered by some algorithms when
  /// pitch is within 1E-6 of π/2 or -π/2.
  void SetFromRotationMatrix(const RotationMatrix<T>& R);

  /// Returns the Vector3 underlying `this` %RollPitchYaw.
  const Vector3<T>& vector() const { return roll_pitch_yaw_; }

  /// Returns the roll-angle underlying `this` %RollPitchYaw.
  const T& roll_angle() const { return roll_pitch_yaw_(0); }

  /// Returns the pitch-angle underlying `this` %RollPitchYaw.
  const T& pitch_angle() const { return roll_pitch_yaw_(1); }

  /// Returns the yaw-angle underlying `this` %RollPitchYaw.
  const T& yaw_angle() const { return roll_pitch_yaw_(2); }

  /// Set the roll-angle underlying `this` %RollPitchYaw.
  /// @param[in] r roll angle (in units of radians).
  void set_roll_angle(const T& r) { roll_pitch_yaw_(0) = r; }

  /// Set the pitch-angle underlying `this` %RollPitchYaw.
  /// @param[in] p pitch angle (in units of radians).
  void set_pitch_angle(const T& p) { roll_pitch_yaw_(1) = p; }

  /// Set the yaw-angle underlying `this` %RollPitchYaw.
  /// @param[in] y yaw angle (in units of radians).
  void set_yaw_angle(const T& y) { roll_pitch_yaw_(2) = y; }

  /// Returns a quaternion representation of `this` %RollPitchYaw.
  Eigen::Quaternion<T> ToQuaternion() const;

  /// Returns the RotationMatrix representation of `this` %RollPitchYaw.
  RotationMatrix<T> ToRotationMatrix() const;

  /// Returns the 3x3 matrix representation of the %RotationMatrix that
  /// corresponds to `this` %RollPitchYaw.  This is a convenient "sugar" method
  /// that is exactly equivalent to RotationMatrix(rpy).ToMatrix3().
  Matrix3<T> ToMatrix3ViaRotationMatrix() const {
    return ToRotationMatrix().matrix();
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RollPitchYaw to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if `‖this - other‖∞ <= tolerance`.
  boolean<T> IsNearlyEqualTo(const RollPitchYaw<T>& other,
                             double tolerance) const {
    const Vector3<T> difference = vector() - other.vector();
    return difference.template lpNorm<Eigen::Infinity>() <= tolerance;
  }

  /// Compares each element of the %RotationMatrix R1 produced by `this` to the
  /// corresponding element of the %RotationMatrix R2 produced by `other` to
  /// check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RollPitchYaw to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between R1, R2.
  /// @returns `true` if `‖R1 - R2‖∞ <= tolerance`.
  boolean<T> IsNearlySameOrientation(const RollPitchYaw<T>& other,
                                     double tolerance) const;

  /// Returns true if roll-pitch-yaw angles `[r, p, y]` are in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  boolean<T> IsRollPitchYawInCanonicalRange() const {
    const T& r = roll_angle();
    const T& p = pitch_angle();
    const T& y = yaw_angle();
    return (-M_PI <= r && r <= M_PI) && (-M_PI / 2 <= p && p <= M_PI / 2) &&
           (-M_PI <= y && y <= M_PI);
  }

  /// Returns true if the pitch-angle `p` is close to gimbal-lock, which means
  /// `cos(p) ≈ 0` or `p ≈ (n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  /// More specifically, returns true if `abs(cos_pitch_angle)` is less than an
  /// internally-defined tolerance of gimbal-lock.
  /// @param[in] cos_pitch_angle cosine of the pitch angle, i.e., `cos(p)`.
  /// @note Pitch-angles close to gimbal-lock can cause problems with numerical
  /// precision and numerical integration.
  static boolean<T> DoesCosPitchAngleViolateGimbalLockTolerance(
      const T& cos_pitch_angle) {
    using std::abs;
    return abs(cos_pitch_angle) < kGimbalLockToleranceCosPitchAngle;
  }

  /// Returns true if the pitch-angle `p` is within an internally-defined
  /// tolerance of gimbal-lock.  In other words, this method returns true if
  /// `p ≈ (n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  /// @note To improve efficiency when cos(pitch_angle()) is already calculated,
  /// instead use the function DoesCosPitchAngleViolateGimbalLockTolerance().
  /// @see DoesCosPitchAngleViolateGimbalLockTolerance()
  boolean<T> DoesPitchAngleViolateGimbalLockTolerance() const {
    using std::cos;
    return DoesCosPitchAngleViolateGimbalLockTolerance(cos(pitch_angle()));
  }

  /// Returns the internally-defined allowable closeness (in radians) of the
  /// pitch angle `p` to gimbal-lock, i.e., the allowable proximity of `p` to
  /// `(n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  static double GimbalLockPitchAngleTolerance() {
    return M_PI_2 - std::acos(kGimbalLockToleranceCosPitchAngle);
  }

  /// Returns true if `rpy` contains valid roll, pitch, yaw angles.
  /// @param[in] rpy allegedly valid roll, pitch, yaw angles.
  /// @note an angle is invalid if it is NaN or infinite.
  static boolean<T> IsValid(const Vector3<T>& rpy) {
    using std::isfinite;
    return isfinite(rpy[0]) && isfinite(rpy[1]) && isfinite(rpy[2]);
  }

  /// Forms Ṙ, the ordinary derivative of the %RotationMatrix `R` with respect
  /// to an independent variable `t` (`t` usually denotes time) and `R` is the
  /// %RotationMatrix formed by `this` %RollPitchYaw.  The roll-pitch-yaw angles
  /// r, p, y are regarded as functions of `t` [i.e., r(t), p(t), y(t)].
  /// @param[in] rpyDt Ordinary derivative of rpy with respect to an independent
  /// variable `t` (`t` usually denotes time, but not necessarily).
  /// @returns Ṙ, the ordinary derivative of `R` with respect to `t`, calculated
  /// as Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ.  In other words, the returned
  /// (i, j) element is ∂Rij/∂r * ṙ + ∂Rij/∂p * ṗ + ∂Rij/∂y * ẏ.
  Matrix3<T> CalcRotationMatrixDt(const Vector3<T>& rpyDt) const;

  /// Calculates angular velocity from `this` %RollPitchYaw whose roll-pitch-yaw
  /// angles `[r; p; y]` relate the orientation of two generic frames A and D.
  /// @param[in] rpyDt Time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  /// @returns w_AD_A, frame D's angular velocity in frame A, expressed in
  /// "parent" frame A. In other words, returns [ωx; ωy; ωz]ᴀ, where
  /// `w_AD_A = ωx Ax + ωy Ay + ωz Az`, and where [ωx; ωy; ωz]ᴀ is calculated
  /// via the the 3x3 matrix Np⁻¹ (the inverse of the matrix Np documented in
  /// CalcMatrixRelatingRpyDtToAngularVelocityInParent()).
  /// ```
  /// ⌈ ωx ⌉         ⌈ ṙ ⌉            ⌈ cos(y)*cos(p)  -sin(y)  0 ⌉
  /// | ωy |  = Np⁻¹ | ṗ |     Np⁻¹ = | sin(y)*cos(p)   cos(y)  0 |
  /// ⌊ ωz ⌋ᴀ        ⌊ ẏ ⌋            ⌊   -sin(p)         0     1 ⌋
  /// ```
  Vector3<T> CalcAngularVelocityInParentFromRpyDt(
      const Vector3<T>& rpyDt) const {
    // Get the 3x3 coefficient matrix M that contains the partial derivatives of
    // w_AD_A with respect to ṙ, ṗ, ẏ.  In other words, `w_AD_A = M * rpyDt`.
    // TODO(Mitiguy) Improve speed -- last column of M is (0, 0, 1).
    const Matrix3<T> M = CalcMatrixRelatingAngularVelocityInParentToRpyDt();
    return M * rpyDt;
  }

  /// Calculates angular velocity from `this` %RollPitchYaw whose roll-pitch-yaw
  /// angles `[r; p; y]` relate the orientation of two generic frames A and D.
  /// @param[in] rpyDt Time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  /// @returns w_AD_D, frame D's angular velocity in frame A, expressed in
  /// "child" frame D. In other words, returns [ω0; ω1; ω2]ᴅ, where
  /// `w_AD_D = ω0 Dx + ω1 Dy + ω2 Dz`, and where [ω0; ω1; ω2]ᴅ is calculated
  /// via the 3x3 matrix Nc⁻¹ (the inverse of the matrix Nc documented in
  /// CalcMatrixRelatingRpyDtToAngularVelocityInChild()).
  /// ```
  /// ⌈ ω0 ⌉         ⌈ ṙ ⌉            ⌈ 1      0        -sin(p)    ⌉
  /// | ω1 |  = Nc⁻¹ | ṗ |     Nc⁻¹ = | 0   cos(r)   sin(r)*cos(p) |
  /// ⌊ ω2 ⌋ᴅ        ⌊ ẏ ⌋            ⌊ 0  -sin(r)   cos(r)*cos(p) ⌋
  /// ```
  Vector3<T> CalcAngularVelocityInChildFromRpyDt(
      const Vector3<T>& rpyDt) const {
    // Get the 3x3 coefficient matrix M that contains the partial derivatives of
    // w_AD_D with respect to ṙ, ṗ, ẏ.  In other words, `w_AD_D = M * rpyDt`.
    // TODO(Mitiguy) Improve speed -- first column of M is (1, 0, 0).
    const Matrix3<T> M = CalcMatrixRelatingAngularVelocityInChildToRpyDt();
    return M * rpyDt;
  }

  /// Uses angular velocity to compute the 1ˢᵗ time-derivative of `this`
  /// %RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  /// @param[in] w_AD_A, frame D's angular velocity in frame A, expressed in A.
  /// @returns `[ṙ; ṗ; ẏ]`, the 1ˢᵗ time-derivative of `this` %RollPitchYaw.
  /// @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  /// @note Enhanced documentation for this method and its gimbal-lock (divide-
  /// by-zero error) is in CalcMatrixRelatingRpyDtToAngularVelocityInParent().
  /// @see CalcRpyDtFromAngularVelocityInChild()
  Vector3<T> CalcRpyDtFromAngularVelocityInParent(
      const Vector3<T>& w_AD_A) const {
    // Get the 3x3 M matrix containing the partial derivatives of [ṙ, ṗ, ẏ] with
    // respect to [wx; wy; wz]ₐ (which is w_AD_A expressed in "parent" frame A).
    // In other words, `rpyDt = M * w_AD_A`.
    // TODO(Mitiguy) Improve speed -- last column of M is (0, 0, 1).
    // TODO(Mitiguy) Improve accuracy when `cos(p) ≈ 0`.
    const Matrix3<T> M =
        CalcMatrixRelatingRpyDtToAngularVelocityInParent(__func__);
    return M * w_AD_A;
  }

  /// Uses angular velocity to compute the 1ˢᵗ time-derivative of `this`
  /// %RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  /// @param[in] w_AD_D, frame D's angular velocity in frame A, expressed in D.
  /// @returns `[ṙ; ṗ; ẏ]`, the 1ˢᵗ time-derivative of `this` %RollPitchYaw.
  /// @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  /// @note Enhanced documentation for this method and its gimbal-lock (divide-
  /// by-zero error) is in CalcMatrixRelatingRpyDtToAngularVelocityInChild().
  /// @see CalcRpyDtFromAngularVelocityInParent()
  Vector3<T> CalcRpyDtFromAngularVelocityInChild(
      const Vector3<T>& w_AD_D) const {
    // Get the 3x3 M matrix containing the partial derivatives of [ṙ, ṗ, ẏ] with
    // respect to [wx; wy; wz]ᴅ (which is w_AD_D expressed in "child" frame D).
    // In other words, `rpyDt = M * w_AD_D`.
    // TODO(Mitiguy) Improve speed -- first column of M is (1, 0, 0).
    // TODO(Mitiguy) Improve accuracy when `cos(p) ≈ 0`.
    const Matrix3<T> M =
        CalcMatrixRelatingRpyDtToAngularVelocityInChild(__func__);
    return M * w_AD_D;
  }

  /// For `this` %RollPitchYaw with roll-pitch-yaw angles [r; p; y] which relate
  /// the orientation of two generic frames A and D, returns the 3x3 matrix Np
  /// relating ṙ, ṗ, ẏ to ωx, ωy, ωz, where frame D's angular velocity in A,
  /// expressed in "parent" A is `w_AD_A = ωx Ax + ωy Ay + ωz Az`. Hence, Np
  /// contains partial derivatives of [ṙ, ṗ, ẏ] with respect to [ωx; ωy; ωz]ᴀ.
  /// ```
  /// ⌈ ṙ ⌉      ⌈ ωx ⌉          ⌈ cos(y)/cos(p)  sin(y)/cos(p)   0 ⌉
  /// | ṗ | = Np | ωy |     Np = |   −sin(y)          cos(y)      0 |
  /// ⌊ ẏ ⌋      ⌊ ωz ⌋ᴀ         ⌊ cos(y)*tan(p)   sin(y)*tan(p)  1 ⌋
  /// ```
  /// @param[in] function_name name of the calling function/method.
  /// @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  /// @note This method has a divide-by-zero error (singularity) when the cosine
  /// of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  /// "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  /// There are associated precision problems (inaccuracies) in the neighborhood
  /// of these pitch angles, i.e., when `cos(p) ≈ 0`.
  /// @see CalcMatrixRelatingRpyDtToAngularVelocityInChild()
  const Matrix3<T> CalcMatrixRelatingRpyDtToAngularVelocityInParent() const {
    const Matrix3<T> M =
        CalcMatrixRelatingRpyDtToAngularVelocityInParent(__func__);
    return M;
  }

  /// For `this` %RollPitchYaw with roll-pitch-yaw angles [r; p; y] which relate
  /// the orientation of two generic frames A and D, returns the 3x3 matrix Nc
  /// relating ṙ, ṗ, ẏ to ω0, ω1, ω2, where frame D's angular velocity in A,
  /// expressed in "child" D is `w_AD_D = ω0 Dx + ω1 Dy + ω2 Dz`. Hence, Nc
  /// contains partial derivatives of [ṙ, ṗ, ẏ] with respect to [ω0; ω1; ω2]ᴅ.
  /// ```
  /// ⌈ ṙ ⌉      ⌈ ω0 ⌉          ⌈ 1  sin(r)*tan(p)  cos(r)*tan(p) ⌉
  /// | ṗ | = Nc | ω1 |     Nc = | 0      cos(r)        −sin(r)    |
  /// ⌊ ẏ ⌋      ⌊ ω2 ⌋ᴅ         ⌊ 0  sin(r)/cos(p)  cos(r)/cos(p) ⌋
  /// ```
  /// @param[in] function_name name of the calling function/method.
  /// @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  /// @note This method has a divide-by-zero error (singularity) when the cosine
  /// of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  /// "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  /// There are associated precision problems (inaccuracies) in the neighborhood
  /// of these pitch angles, i.e., when `cos(p) ≈ 0`.
  /// @see CalcMatrixRelatingRpyDtToAngularVelocityInParent()
  const Matrix3<T> CalcMatrixRelatingRpyDtToAngularVelocityInChild() const {
    const Matrix3<T> M =
        CalcMatrixRelatingRpyDtToAngularVelocityInChild(__func__);
    return M;
  }

  /// Uses angular acceleration to compute the 2ⁿᵈ time-derivative of `this`
  /// %RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  /// @param[in] rpyDt time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  /// @param[in] alpha_AD_A, frame D's angular acceleration in frame A,
  /// expressed in frame A.
  /// @returns `[r̈, p̈, ÿ]`, the 2ⁿᵈ time-derivative of `this` %RollPitchYaw.
  /// @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  /// @note This method has a divide-by-zero error (singularity) when the cosine
  /// of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  /// "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  /// There are associated precision problems (inaccuracies) in the neighborhood
  /// of these pitch angles, i.e., when `cos(p) ≈ 0`.
  Vector3<T> CalcRpyDDtFromRpyDtAndAngularAccelInParent(
      const Vector3<T>& rpyDt, const Vector3<T>& alpha_AD_A) const;

  /// Uses angular acceleration to compute the 2ⁿᵈ time-derivative of `this`
  /// %RollPitchYaw whose angles `[r; p; y]` orient two generic frames A and D.
  /// @param[in] rpyDt time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  /// @param[in] alpha_AD_D, frame D's angular acceleration in frame A,
  /// expressed in frame D.
  /// @returns `[r̈, p̈, ÿ]`, the 2ⁿᵈ time-derivative of `this` %RollPitchYaw.
  /// @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  /// @note This method has a divide-by-zero error (singularity) when the cosine
  /// of the pitch angle `p` is zero [i.e., `cos(p) = 0`].  This problem (called
  /// "gimbal lock") occurs when `p = n π  + π / 2`, where n is any integer.
  /// There are associated precision problems (inaccuracies) in the neighborhood
  /// of these pitch angles, i.e., when `cos(p) ≈ 0`.
  Vector3<T> CalcRpyDDtFromAngularAccelInChild(
      const Vector3<T>& rpyDt, const Vector3<T>& alpha_AD_D) const;

  /// Implements the @ref hash_append concept.
  /// @pre T implements the hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const RollPitchYaw& rpy) noexcept {
    const T* begin = rpy.roll_pitch_yaw_.data();
    const T* end = rpy.roll_pitch_yaw_.data() + rpy.roll_pitch_yaw_.size();
    using drake::hash_append_range;
    hash_append_range(hasher, begin, end);
  }

 private:
  // Throws an exception if rpy is not a valid %RollPitchYaw.
  // @param[in] rpy an allegedly valid rotation matrix.
  // @note If the underlying scalar type T is non-numeric (symbolic), no
  // validity check is made and no assertion is thrown.
  template <typename S = T>
  static typename std::enable_if_t<scalar_predicate<S>::is_bool>
  ThrowIfNotValid(const Vector3<T>& rpy) {
    if (!RollPitchYaw<T>::IsValid(rpy)) {
      throw std::logic_error(
          "Error: One (or more) of the roll-pitch-yaw angles is infinity or "
          "NaN.");
    }
  }

  template <typename S = T>
  static typename std::enable_if_t<!scalar_predicate<S>::is_bool>
  ThrowIfNotValid(const Vector3<S>&) {}

  // Throws an exception with a message that the pitch-angle `p` violates the
  // internally-defined gimbal-lock tolerance, which occurs when `cos(p) ≈ 0`,
  // which means `p ≈ (n*π + π/2)` where `n = 0, ±1, ±2, ...`.
  // @param[in] function_name name of the calling function/method.
  // @param[in] pitch_angle pitch angle `p` (in radians).
  // @throws std::exception with a message that `p` is too near gimbal-lock.
  static void ThrowPitchAngleViolatesGimbalLockTolerance(
      const char* function_name, const T& pitch_angle);

  // Uses a quaternion and its associated rotation matrix `R` to accurately
  // and efficiently set the roll-pitch-yaw angles (SpaceXYZ Euler angles)
  // that underlie `this` @RollPitchYaw, even when the pitch angle p is very
  // near a singularity (e.g., when p is within 1E-6 of π/2 or -π/2).
  // After calling this method, the underlying roll-pitch-yaw `[r, p, y]` has
  // range `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  // @param[in] quaternion unit quaternion with elements `[e0, e1, e2, e3]`.
  // @param[in] R The %RotationMatrix corresponding to `quaternion`.
  // @throws std::exception in debug builds if rpy fails IsValid(rpy).
  // @note Aborts in debug builds if `quaternion` does not correspond to `R`.
  void SetFromQuaternionAndRotationMatrix(
      const Eigen::Quaternion<T>& quaternion, const RotationMatrix<T>& R);

  // For the %RotationMatrix `R` generated by `this` %RollPitchYaw, this method
  // calculates the partial derivatives of `R` with respect to roll, pitch, yaw.
  // @param[out] R_r ∂R/∂r Partial derivative of `R` with respect to roll `r`.
  // @param[out] R_p ∂R/∂p Partial derivative of `R` with respect to pitch `p`.
  // @param[out] R_y ∂R/∂y Partial derivative of `R` with respect to yaw `y`.
  void CalcRotationMatrixDrDpDy(Matrix3<T>* R_r, Matrix3<T>* R_p,
                                Matrix3<T>* R_y) const;

  // For `this` %RollPitchYaw with roll-pitch-yaw angles `[r; p; y]` which
  // relate the orientation of two generic frames A and D, returns the 3x3
  // coefficient matrix M that contains the partial derivatives of `w_AD_A`
  // (D's angular velocity in A, expressed in A) with respect to ṙ, ṗ, ẏ.
  // In other words, `w_AD_A = M * rpyDt` where `rpyDt` is `[ṙ; ṗ; ẏ]`.
  Matrix3<T> CalcMatrixRelatingAngularVelocityInParentToRpyDt() const;

  // Returns the time-derivative of the 3x3 matrix returned by the `this`
  // %RollPitchYaw method MatrixRelatingAngularVelocityAToRpyDt().
  // @param[in] rpyDt Time-derivative of `[r; p; y]`, i.e., `[ṙ; ṗ; ẏ]`.
  // @see MatrixRelatingAngularVelocityAToRpyDt()
  Matrix3<T> CalcDtMatrixRelatingAngularVelocityInParentToRpyDt(
      const Vector3<T>& rpyDt) const;

  // For `this` %RollPitchYaw with roll-pitch-yaw angles `[r; p; y]` which
  // relate the orientation of two generic frames A and D, returns the 3x3
  // coefficient matrix M that contains the partial derivatives of `w_AD_D`
  // (D's angular velocity in A, expressed in D) with respect to ṙ, ṗ, ẏ.
  // In other words, `w_AD_D = M * rpyDt` where `rpyDt` is `[ṙ; ṗ; ẏ]`.
  Matrix3<T> CalcMatrixRelatingAngularVelocityInChildToRpyDt() const;

  // @param[in] function_name name of the calling function/method.
  // @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // @note Detailed information about this function is in the public method
  // CalcMatrixRelatingRpyDtToAngularVelocityInParent(). Generally, this utility
  // method is called from a user-relevant API. If gimbal-lock is detected, the
  // issued error message includes the calling function's name.
  Matrix3<T> CalcMatrixRelatingRpyDtToAngularVelocityInParent(
      const char* function_name) const;

  // @param[in] function_name name of the calling function/method.
  // @throws std::exception if `cos(p) ≈ 0` (`p` is near gimbal-lock).
  // @note Detailed information about this function is in the public method
  // CalcMatrixRelatingRpyDtToAngularVelocityInChild(). Generally, this utility
  // method is called from a user-relevant API. If gimbal-lock is detected, the
  // issued error message includes the calling function's name.
  Matrix3<T> CalcMatrixRelatingRpyDtToAngularVelocityInChild(
      const char* function_name) const;

  // Sets `this` %RollPitchYaw from a Vector3.
  // @param[in] rpy allegedly valid roll-pitch-yaw angles.
  // @throws std::exception in debug builds if rpy fails IsValid(rpy).
  RollPitchYaw<T>& SetOrThrowIfNotValidInDebugBuild(const Vector3<T>& rpy) {
    DRAKE_ASSERT_VOID(ThrowIfNotValid(rpy));
    roll_pitch_yaw_ = rpy;
    return *this;
  }

  // Stores the underlying roll-pitch-yaw angles.
  // There is no default initialization needed.
  Vector3<T> roll_pitch_yaw_;

  // Internally-defined value for the allowable proximity of the cosine of the
  // pitch-angle `p` to gimbal-lock [the proximity of `cos(p)` to 0].
  // @note For small values (<= 0.1), this value approximates the allowable
  // proximity of the pitch-angle (in radians) to gimbal-lock.  Example: A value
  // of 0.01 corresponds to `p` within ≈ 0.01 radians (≈ 0.57°) of gimbal-lock,
  // i.e., `p` is within 0.01 radians of `(n*π + π/2)`, `n = 0, ±1, ±2, ...`.
  // A value 0.008 corresponds to `p` ≈ 0.008 radians (≈ 0.46°) of gimbal-lock.
  // @note The conversion from angular velocity to rpyDt (the time-derivative of
  // %RollPitchYaw) has a calculation that divides by `cos(p)` (the cosine of
  // the pitch angle).  This results in values of rpyDt that scale with angular
  // velocity multiplied by `1/cos(p)`, which can cause problems with numerical
  // precision and numerical integration.
  // @note There is a numerical test (in roll_pitch_yaw_test.cc) that converts
  // an angular velocity w(1, 1, 1) to rpyDt (time-derivative of roll-pitch-yaw)
  // and then back to angular velocity.  This test revealed an imprecision in
  // the back-and-forth calculation near gimbal-lock by calculating max_error
  // (how much the angular velocity changed in the back-and-forth calculation).
  // The table below shows various values of kGimbalLockToleranceCosPitchAngle,
  // it associated proximity to gimbal-lock (in degrees) and the associated
  // max_error (in terms of machine kEpsilon = 1/2^52 ≈ 2.22E-16).
  // ---------------------------------------------------------------------------
  //  kGimbalLockToleranceCosPitchAngle  |  max_error
  // ---------------------------------------------------------------------------
  //  0.001 ≈ 0.06°                      | (2^10 = 1024) * kEpsilon ≈ 2.274E-13
  //  0.002 ≈ 0.11°                      |  (2^9 = 512)  * kEpsilon ≈ 1.137E-13
  //  0.004 ≈ 0.23°                      |  (2^8 = 256)  * kEpsilon ≈ 5.684E-14
  //  0.008 ≈ 0.46°                      |  (2^7 = 128)  * kEpsilon ≈ 2.842E-14
  //  0.016 ≈ 0.92°                      |  (2^6 =  64)  * kEpsilon ≈ 1.421E-14
  //  0.032 ≈ 1.83°                      |  (2^5 =  32)  * kEpsilon ≈ 7.105E-15
  // ---------------------------------------------------------------------------
  // Hence if kGimbalLockToleranceCosPitchAngle = 0.008 and the pitch angle is
  // in the proximity of ≈ 0.46° of gimbal lock, there may be inaccuracies in
  // 7 of the 52 bits in max_error's mantissa, which we deem acceptable.
  static constexpr double kGimbalLockToleranceCosPitchAngle = 0.008;
};

/// Stream insertion operator to write an instance of RollPitchYaw into a
/// `std::ostream`. Especially useful for debugging.
/// @relates RollPitchYaw.
template <typename T>
std::ostream& operator<<(std::ostream& out, const RollPitchYaw<T>& rpy);

/// Abbreviation (alias/typedef) for a RollPitchYaw double scalar type.
/// @relates RollPitchYaw
using RollPitchYawd = RollPitchYaw<double>;

}  // namespace math
}  // namespace drake

// Format RollPitchYaw using its operator<<.
// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <typename T>
struct formatter<drake::math::RollPitchYaw<T>> : drake::ostream_formatter {};
}  // namespace fmt

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RollPitchYaw)
