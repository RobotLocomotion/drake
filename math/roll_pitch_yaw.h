#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"

namespace drake {
namespace math {

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
    const T w = c0 * c1 * c2 + s0 * s1 * s2;
    const T x = s0 * c1 * c2 - c0 * s1 * s2;
    const T y = c0 * s1 * c2 + s0 * c1 * s2;
    const T z = c0 * c1 * s2 - s0 * s1 * c2;
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

  /// Returns true if `rpy` contains valid roll, pitch, yaw angles.
  /// @param[in] rpy allegedly valid roll, pitch, yaw angles.
  /// @note an angle is invalid if it is NaN or infinite.
  static bool IsValid(const Vector3<T>& rpy) { return rpy.allFinite(); }

 private:
  // Throws an exception if rpy is not a valid %RollPitchYaw.
  // @param[in] rpy an allegedly valid rotation matrix.
  static void ThrowIfNotValid(const Vector3<T>& rpy) {
    if (!RollPitchYaw<T>::IsValid(rpy)) {
      throw std::logic_error(
       "Error: One (or more) of the roll-pitch-yaw angles is infinity or NaN.");
    }
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

}  // namespace math
}  // namespace drake


