#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {

/// This class represents a rigid transform between two frames, which can be
/// regarded in two ways.  It can be regarded as a distance-preserving linear
/// operator (i.e., one that rotates and/or translates) which (for example) can
/// add one position vector to another and express the result in a particular
/// basis as `p_AoQ_A = X_AB * p_BoQ_B` (Q is any point).  Alternately, a rigid
/// transform describes the pose between two frames A and B (i.e., the relative
/// orientation and position of A to B).  Herein, the terms rotation/orientation
/// and translation/position are used interchangeably.
/// The class stores a RotationMatrix that relates right-handed orthogonal
/// unit vectors Ax, Ay, Az fixed in frame A to right-handed orthogonal
/// unit vectors Bx, By, Bz fixed in frame B.
/// The class also stores a position vector from Ao (the origin of frame A) to
/// Bo (the origin of frame B).  The position vector is expressed in frame A.
/// The monogram notation for the transform relating frame A to B is `X_AB`.
/// The monogram notation for the rotation matrix relating A to B is `R_AB`.
/// The monogram notation for the position vector from Ao to Bo is `p_AoBo_A`.
/// See @ref multibody_quantities for monogram notation for dynamics.
///
/// @note This class does not store the frames associated with the transform and
/// cannot enforce proper usage of this class.  For example, it makes sense to
/// multiply transforms as `X_AB * X_BC`, but not `X_AB * X_CB`.
///
/// @note This class is not a 4x4 transformation matrix -- even though its
/// operator*() methods act like 4x4 matrix multiplication.  Instead, this class
/// contains a rotation matrix class as well as a 3x1 position vector.  To form
/// a 4x4 matrix, use GetAsMatrix().  GetAsIsometry() is treated similarly.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class Transform {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Transform)

  /// Constructs the %Transform that corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, the constructed %Transform contains an identity RotationMatrix and
  /// a zero position vector.
  Transform() { SetIdentity(); }

  /// Constructs a %Transform from a rotation matrix and a position vector.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  Transform(const RotationMatrix<T>& R, const Vector3<T>& p)
      : R_AB_(R), p_AoBo_A_(p) {}

  /// Constructs a Transform from an Eigen Isometry3.
  /// @param[in] pose Isometry3 that contains an allegedly valid rotation matrix
  /// `R_AB` and also contains a position vector `p_AoBo_A` from frame A's
  /// origin to frame B's origin.  `p_AoBo_A` must be expressed in frame A.
  /// @throws std::logic_error in debug builds if R_AB is not a proper
  /// orthonormal 3x3 rotation matrix.
  /// @note no attempt is made to orthogonalize the 3x3 rotation matrix part of
  /// `pose`.  As needed, use RotationMatrix::ProjectToRotationMatrix().
  explicit Transform(const Isometry3<T>& pose) :
      Transform(RotationMatrix<T>(pose.linear()), pose.translation()) {}

  /// Creates a %Transform templatized on a scalar type U from a
  /// %Transform templatized on scalar type T.  For example,
  /// ```
  /// Transform<double> source = Transform<double>::Identity();
  /// Transform<AutoDiffXd> foo = source.cast<AutoDiffXd>();
  /// ```
  /// @tparam U Scalar type on which the returned %Transform is templated.
  /// @note `Transform<From>::cast<To>()` creates a new `Transform<To>` from a
  /// `Transform<From>` but only if type `To` is constructible from type `From`.
  /// This cast method works in accordance with Eigen's cast method for Eigen's
  /// objects that underlie this %Transform.  For example, Eigen currently
  /// allows cast from type double to AutoDiffXd, but not vice-versa.
  template <typename U>
  Transform<U> cast() const {
    const RotationMatrix<U> R = R_AB_.template cast<U>();
    const Vector3<U> p = p_AoBo_A_.template cast<U>();
    return Transform<U>(R, p);
  }

  /// Returns the identity %Transform (which corresponds to coincident frames).
  /// @returns the %Transform that corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, the returned %Transform contains a 3x3 identity matrix and a
  /// zero position vector.
  // @internal This method's name was chosen to mimic Eigen's Identity().
  static const Transform<T>& Identity() {
    static const never_destroyed<Transform<T>> kIdentity;
    return kIdentity.access();
  }

  /// Returns R_AB, the rotation matrix portion of `this` transform.
  /// @retval R_AB the rotation matrix portion of `this` transform.
  const RotationMatrix<T>& rotation() const { return R_AB_; }

  /// Sets the %RotationMatrix portion of `this` transform.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  void set_rotation(const RotationMatrix<T>& R) { R_AB_ = R; }

  /// Returns `p_AoBo_A`, the position vector portion of `this` transform.
  /// @retval p_AoBo_A the position vector portion of `this` transform, i.e.,
  /// the position vector from Ao (frame A's origin) to Bo (frame B's origin).
  const Vector3<T>& translation() const { return p_AoBo_A_; }

  /// Sets the position vector portion of `this` transform.
  /// @param[in] p position vector from Ao (frame A's origin) to Bo (frame B's
  /// origin) expressed in frame A.  In monogram notation p is denoted p_AoBo_A.
  void set_translation(const Vector3<T>& p) { p_AoBo_A_ = p; }

  /// Returns the 4x4 matrix associated with this %Transform.
  /// @return the 4x4 matrix associated with this %Transform, i.e., returns X_AB
  ///  ┌                ┐
  ///  │ R_AB  p_AoBo_A │
  ///  │                │
  ///  │   0      1     │
  ///  └                ┘
  Matrix4<T> GetAsMatrix4() const {
    Matrix4<T> pose;
    pose.template topLeftCorner<3, 3>() = rotation().matrix();
    pose.template topRightCorner<3, 1>() = translation();
    pose.row(3) = Vector4<T>(0, 0, 0, 1);
    return pose;
  }

  /// Returns the Isometry associated with a %Transform.
  /// @returns the Isometry associated with a %Transform.
  Isometry3<T> GetAsIsometry3() const {
    // pose.linear() returns a mutable reference to the 3x3 rotation matrix part
    // of Isometry3 and pose.translation() returns a mutable reference to the
    // 3x1 position vector part of the Isometry3.
    Isometry3<T> pose;
    pose.linear() = rotation().matrix();
    pose.translation() = translation();
    return pose;
  }

  /// Sets `this` %Transform so it corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, `this` %Transform contains a 3x3 identity matrix and a
  /// zero position vector.
  const Transform<T>& SetIdentity() {
    R_AB_ = RotationMatrix<T>::Identity();
    p_AoBo_A_.setZero();
    return *this;
  }

  /// Returns `true` if `this` is exactly the identity transform.
  /// @returns `true` if `this` is exactly the identity transform.
  /// @see IsIdentityToEpsilon().
  bool IsExactlyIdentity() const {
    return rotation().IsExactlyIdentity() && (translation().array() == 0).all();
  }

  /// Returns true if `this` is within tolerance of the identity transform.
  /// @returns `true` if the RotationMatrix portion of `this` satisfies
  /// RotationMatrix::IsIdentityToInternalTolerance() and if the position vector
  /// portion of `this` is equal to zero vector within `translation_tolerance`.
  /// @param[in] translation_tolerance a non-negative number.  One way to choose
  /// `translation_tolerance` is to multiply a characteristic length
  /// (e.g., the magnitude of a characteristic position vector) by an epsilon
  /// (e.g., RotationMatrix::get_internal_tolerance_for_orthonormality()).
  /// @see IsExactlyIdentity().
  bool IsIdentityToEpsilon(double translation_tolerance) const {
    const T max_component = translation().template lpNorm<Eigen::Infinity>();
    return max_component <= translation_tolerance &&
        rotation().IsIdentityToInternalTolerance();
  }

  /// Calculates X_BA = X_AB⁻¹, the inverse of `this` %Transform.
  /// @retval X_BA = X_AB⁻¹ the inverse of `this` %Transform.
  /// @note The inverse of transform X_AB is X_BA, which contains the rotation
  /// matrix R_BA = R_AB⁻¹ = R_ABᵀ and the position vector `p_BoAo_B_`,
  /// (position from B's origin Bo to A's origin Ao, expressed in frame B).
  /// @note: The square-root of the condition number for a Transform is roughly
  /// the magnitude of the position vector.  The accuracy of the calculation for
  /// the inverse of a Transform drops off with the sqrt condition number.
  // @internal This method's name was chosen to mimic Eigen's inverse().
  Transform<T> inverse() const {
    const RotationMatrix<T> R_BA = R_AB_.inverse();
    return Transform<T>(R_BA, R_BA * (-p_AoBo_A_));
  }

  /// In-place multiply of `this` transform `X_AB` by `other` transform `X_BC`.
  /// @param[in] other %Transform that post-multiplies `this`.
  /// @returns `this` transform which has been multiplied by `other`.
  /// On return, `this = X_AC`, where `X_AC = X_AB * X_BC`.
  Transform<T>& operator*=(const Transform<T>& other) {
    p_AoBo_A_ = *this * other.translation();
    R_AB_ *= other.rotation();
    return *this;
  }

  /// Calculates `this` transform `X_AB` multiplied by `other` transform `X_BC`.
  /// @param[in] other %Transform that post-multiplies `this`.
  /// @retval X_AC = X_AB * X_BC
  Transform<T> operator*(const Transform<T>& other) const {
    const Vector3<T> p_AoCo_A = *this * other.translation();
    return Transform<T>(rotation() * other.rotation(), p_AoCo_A);
  }

  /// Calculates `this` transform `X_AB` multiplied by the position vector
  /// 'p_BoQ_B` which is from Bo (B's origin) to an arbitrary point Q.
  /// @param[in] p_BoQ_B position vector from Bo to Q, expressed in frame B.
  /// @retval p_AoQ_A position vector from Ao to Q, expressed in frame A.
  Vector3<T> operator*(const Vector3<T>& p_BoQ_B) const {
    return p_AoBo_A_ + R_AB_ * p_BoQ_B;
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %Transform to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// elements in `this` and `other`.
  /// @returns `true` if `‖this.matrix() - other.matrix()‖∞ <= tolerance`.
  /// @note Consider scaling tolerance with the largest of magA and magB, where
  /// magA and magB denoted the magnitudes of `this` position vector and `other`
  /// position vectors, respectively.
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
  // Make Transform<U> templatized on any typename U be a friend of a %Transform
  // templatized on any other typename T. This is needed for the method
  // Transform<T>::cast<U>() to be able to use the required private constructor.
  template <typename U>
  friend class Transform;

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  RotationMatrix<T> R_AB_;

  // Position vector from A's origin Ao to B's origin Bo, expressed in A.
  Vector3<T> p_AoBo_A_;
};

}  // namespace math
}  // namespace drake
