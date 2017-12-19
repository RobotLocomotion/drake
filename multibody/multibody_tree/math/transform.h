#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/rotation_matrix.h"

namespace drake {
namespace multibody {

/// This class represents a transform between two arbitrary frames A and B.  It
/// contains data and methods for both rotation (orientation) and translation.
/// The class stores a 3x3 rotation matrix that relates right-handed orthogonal
/// unit vectors Ax, Ay, Az fixed in frame A to right-handed orthogonal
/// unit vectors Bx, By, Bz fixed in frame B.
/// The class also stores a position vector from a point Ao fixed on frame A to
/// a point Bo fixed on frame B.  The position vector is expressed in frame A.
/// The monogram notation for the transform relating frame A to B is `X_AB`.
/// The monogram notation for the rotation matrix relating A to B is `R_AB`.
/// The monogram notation for the position vector from Ao to Bo is `p_AoBo_A`.
/// See @ref multibody_quantities for monogram notation for dynamics.
///
/// @note This class does not store the frames associated with a transform,
/// nor does it enforce strict proper usage of this class with vectors.
///
/// @note This class is not a 4x4 transformation matrix -- even though its
/// operator*() methods act like 4x4 matrix multiplication.  Instead, this
/// class contains a rotation matrix class as well as a 3x1 position vector.
/// To form a 4x4 matrix, use GetAsMatrix(). similarly for GetAsIsometry().
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class Transform {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Transform)

  /// Constructs the %Transform that corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, the constructed %Transform contains a 3x3 identity matrix and a
  /// zero position vector.
  Transform() { SetIdentityMatrixAndZeroPositionVector(); }

  /// Constructs a %Transform from a rotation matrix and a position vector.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  Transform(const RotationMatrix<T>& R, const Vector3<T>& p) :
      R_AB_(R), p_AoBo_A_(p) {}

  /// Constructs a Transform from an Eigen Isometry3.
  /// @param[in] pose Isometry3 that contains an allegedly valid rotation matrix
  /// `R_AB` and also contains a position vector `p_AoBo_A` from frame A's
  /// origin to frame B's origin.  `p_AoBo_A` must be expressed in frame A.
  /// @throws exception std::logic_error in debug builds if R_AB is not a proper
  /// orthonormal 3x3 matrix.
  explicit Transform(const Isometry3<T>& pose) :
      Transform(RotationMatrix<T>(pose.linear()), pose.translation()) {}

  /// @returns the %Transform that corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, the returned %Transform contains a 3x3 identity matrix and a
  /// zero position vector.
  static Transform<T> MakeIdentity() { return Transform(); }

  /// @retval `R_AB`, the rotation matrix portion of `this` transform.
  const RotationMatrix<T>& rotation() const { return R_AB_; }

  /// @retval `p_AoBo_A`, the position vector portion of `this` transform, i.e.,
  /// the position vector from Ao (frame A's origin) to Bo (frame B's origin).
  const Vector3<T>& translation() const { return p_AoBo_A_; }

  /// @returns the 4x4 matrix associated with a %Transform.
  Matrix4<T> GetAsMatrix() const {
    Matrix4<T> pose;
    pose.topLeftCorner(3, 3) = rotation().matrix();
    const Vector3<T> p = translation();
    pose(0, 3) = p(0);
    pose(1, 3) = p(1);
    pose(2, 3) = p(2);
    pose(3, 0) = pose(3, 1) = pose(3, 2) = 0;
    pose(3, 3) = 1;
    return pose;
  }

  /// @returns the Isometry associated with a %Transform.
  Isometry3<T> GetAsIsometry3() const {
    // pose.linear() returns a mutable reference to the 3x3 rotation matrix part
    // of Isometry3 and pose.translation() returns a mutable reference to the
    // 3x1 position vector part of the Isometry3.
    Isometry3<T> pose;
    pose.makeAffine();            // Sets the last row to [0, 0, 0, 1]
    pose.linear() = rotation().matrix();
    pose.translation() = translation();
    return pose;
  }

  /// Sets `this` %Transform so it corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, `this` %Transform contains a 3x3 identity matrix and a
  /// zero position vector.
  const Transform<T>& SetIdentityMatrixAndZeroPositionVector() {
    R_AB_ = RotationMatrix<T>::MakeIdentity();
    p_AoBo_A_.setZero();
    return *this;
  }

  /// @retval X_BA = X_AB⁻¹, the inverse of this %Transform.
  /// @note The inverse of transform X_AB is X_BA, which contains the rotation
  /// matrix R_BA = R_AB⁻¹ = R_ABᵀand the position vector `p_BoAo_B_`,
  /// (position from B's origin Bo to A's origin Ao, expressed in frame B).
  Transform<T> inverse() const {
    const RotationMatrix<T> R_BA = R_AB_.inverse();
    return Transform<T>(R_BA, R_BA * (-p_AoBo_A_));
  }

  /// Operator to multiply `this` transform `X_AB` by `other` transform `X_BC`.
  /// @param[in] other %Transform that post-multiplies `this`.
  /// @returns `this` transform which has been multiplied by `other`.
  /// On return, `this = X_AC`, where `X_AC = X_AB * X_BC`.
  Transform<T>& operator*=(const Transform<T>& other) {
    p_AoBo_A_ = *this * other.translation();
    R_AB_ *= other.rotation();
    return *this;
  }

  /// Operator to multiply `this` transform `X_AB` by `other` transform `X_BC`.
  /// @param[in] other %Transform that post-multiplies `this`.
  /// @returns transform `X_AC` that results from `X_AB * X_BC`.
  Transform<T> operator*(const Transform<T>& other) const {
    const Vector3<T> p_AoCo_A = *this * other.translation();
    return Transform<T>(rotation() * other.rotation(), p_AoCo_A);
  }

  /// Operator to multiply `this` transform `X_AB` by the position vector from
  /// Bo (B's origin) to Co (the origin of an arbitrary frame C).
  /// @param[in] p_BoCo_B position vector from Bo to Co, expressed in frame B.
  /// @retval p_AoCo_A, position vector from Ao to Co, expressed in frame A.
  Vector3<T> operator*(const Vector3<T>& p_BoCo_B) const {
    return p_AoBo_A_ + R_AB_ * p_BoCo_B;
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %Transform to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// elements in `this` and `other`.
  /// @returns `true` if ‖`this` - `other`‖∞ <= tolerance.
  bool IsNearlyEqualTo(const Transform<T>& other, double tolerance) const {
    return GetMaximumAbsoluteDifference(other) <= tolerance;
  }

  /// Computes the infinity norm of `this` - `other` (i.e., the maximum absolute
  /// value of the difference between the elements of `this` and `other`).
  /// @param[in] other %Transform to subtract from `this`.
  /// @returns ‖`this` - `other`‖∞
  T GetMaximumAbsoluteDifference(const Transform<T>& other) const {
    const T R_difference = R_AB_.GetMaximumAbsoluteDifference(other.rotation());
    const T p_difference = GetMaximumAbsoluteTranslationDifference(other);
    return R_difference > p_difference ? R_difference : p_difference;
  }

  /// Returns the maximum absolute value of the difference in the position
  /// vectors (translation) in `this` and `other`.  In other words, returns
  /// the infinity norm of the difference in the position vectors.
  /// @param[in] other %Transform whose position vector is subtracted from
  /// the position vector in `this`.
  T GetMaximumAbsoluteTranslationDifference(const Transform<T>& other) const {
    const Vector3<T> p_difference = translation() - other.translation();
    return p_difference.template lpNorm<Eigen::Infinity>();
  }

 private:
  // @retval `R_AB`, the rotation matrix portion of `this` transform.
  // @note There is both a mutable and const version of rotation().
  RotationMatrix<T>& get_mutable_rotation() { return R_AB_; }

  // @retval `p_AoBo_A`, the position vector portion of `this` transform, i.e.,
  // the position vector from Ao (frame A's origin) to Bo (frame B's origin).
  // @note There is both a mutable and const version of translation().
  Vector3<T>& get_mutable_translation() { return p_AoBo_A_; }

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  RotationMatrix<T> R_AB_;

  // Position vector from A's origin Ao to B's origin Bo, expressed in A.
  Vector3<T> p_AoBo_A_;
};

}  // namespace multibody
}  // namespace drake
