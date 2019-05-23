#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {

/// This class represents a proper rigid transform between two frames which can
/// be regarded in two ways.  It can be regarded as a distance-preserving linear
/// operator that can rotate and/or translate a rigid body without changing its
/// shape or size (rigid) and without mirroring/reflecting the body (proper),
/// e.g., it can add one position vector to another and express the result in a
/// particular basis as `p_AoQ_A = X_AB * p_BoQ_B` (Q is any point).
/// Alternately, a rigid transform describes the pose between two frames A and B
/// (i.e., the relative orientation and position of A to B).  Herein, the terms
/// rotation/orientation and translation/position are used interchangeably.
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
/// cannot enforce correct usage of this class.  For example, it makes sense to
/// multiply %RigidTransforms as `X_AB * X_BC`, but not `X_AB * X_CB`.
///
/// @note This class is not a 4x4 transformation matrix -- even though its
/// operator*() methods act like 4x4 matrix multiplication.  Instead, this class
/// contains a rotation matrix class as well as a 3x1 position vector.  To form
/// a 4x4 matrix, use GetAsMatrix().  GetAsIsometry() is treated similarly.
///
/// @note An isometry is sometimes regarded as synonymous with rigid transform.
/// The %RigidTransform class has important advantages over Eigen::Isometry.
/// - %RigidTransform is built on an underlying rigorous 3x3 RotationMatrix
///   class that has significant functionality for 3D orientation.
/// - In Debug builds, %RigidTransform requires a valid 3x3 rotation matrix
///   and a valid (non-NAN) position vector.  Eigen::Isometry does not.
/// - %RigidTransform catches bugs that are undetected by Eigen::Isometry.
/// - %RigidTransform has additional functionality and ease-of-use,
///   resulting in shorter, easier to write, and easier to read code.
/// - The name Isometry is unfamiliar to many roboticists and dynamicists and
///   for them Isometry.linear() is (for example) a counter-intuitive method
///   name to return a rotation matrix.
///
/// @note One of the constructors in this class provides an implicit conversion
/// from an Eigen Translation to %RigidTransform.
///
/// @authors Paul Mitiguy (2018) Original author.
/// @authors Drake team (see https://drake.mit.edu/credits).
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class RigidTransform {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidTransform)

  /// Constructs the %RigidTransform that corresponds to aligning the two frames
  /// so unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with
  /// Bo.  Hence, the constructed %RigidTransform contains an identity
  /// RotationMatrix and a zero position vector.
  RigidTransform() {
    // @internal The default RotationMatrix constructor is an identity matrix.
    set_translation(Vector3<T>::Zero());
  }

  /// Constructs a %RigidTransform from a rotation matrix and a position vector.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  RigidTransform(const RotationMatrix<T>& R, const Vector3<T>& p) { set(R, p); }

  /// Constructs a %RigidTransform from a RollPitchYaw and a position vector.
  /// @param[in] rpy a %RollPitchYaw which is a Space-fixed (extrinsic) X-Y-Z
  /// rotation with "roll-pitch-yaw" angles `[r, p, y]` or equivalently a Body-
  /// fixed (intrinsic) Z-Y-X rotation with "yaw-pitch-roll" angles `[y, p, r]`.
  /// @see RotationMatrix::RotationMatrix(const RollPitchYaw<T>&)
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  RigidTransform(const RollPitchYaw<T>& rpy, const Vector3<T>& p)
      : RigidTransform(RotationMatrix<T>(rpy), p) {}

  /// Constructs a %RigidTransform from a Quaternion and a position vector.
  /// @param[in] quaternion a non-zero, finite quaternion which may or may not
  /// have unit length [i.e., `quaternion.norm()` does not have to be 1].
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  /// @throws std::logic_error in debug builds if the rotation matrix
  /// that is built from `quaternion` is invalid.
  /// @see RotationMatrix::RotationMatrix(const Eigen::Quaternion<T>&)
  RigidTransform(const Eigen::Quaternion<T>& quaternion, const Vector3<T>& p)
      : RigidTransform(RotationMatrix<T>(quaternion), p) {}

  /// Constructs a %RigidTransform from a AngleAxis and a position vector.
  /// @param[in] theta_lambda an Eigen::AngleAxis whose associated axis (vector
  /// direction herein called `lambda`) is non-zero and finite, but which may or
  /// may not have unit length [i.e., `lambda.norm()` does not have to be 1].
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A
  /// @throws std::logic_error in debug builds if the rotation matrix
  /// that is built from `theta_lambda` is invalid.
  /// @see RotationMatrix::RotationMatrix(const Eigen::AngleAxis<T>&)
  RigidTransform(const Eigen::AngleAxis<T>& theta_lambda, const Vector3<T>& p)
      : RigidTransform(RotationMatrix<T>(theta_lambda), p) {}

  /// Constructs a %RigidTransform with a given RotationMatrix and a zero
  /// position vector.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  explicit RigidTransform(const RotationMatrix<T>& R) {
    set(R, Vector3<T>::Zero());
  }

  /// Constructs a %RigidTransform with an identity RotationMatrix and a given
  /// position vector `p`.
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  explicit RigidTransform(const Vector3<T>& p) { set_translation(p); }

  /// Constructs a %RigidTransform that contains an identity RotationMatrix and
  /// the position vector underlying the given `translation`.
  /// @param[in] translation translation-only transform that stores p_AoQ_A, the
  /// position vector from frame A's origin to a point Q, expressed in frame A.
  /// @note The constructed %RigidTransform `X_AAq` relates frame A to a frame
  /// Aq whose basis unit vectors are aligned with Ax, Ay, Az and whose origin
  /// position is located at point Q.
  /// @note This constructor provides an implicit conversion from Translation to
  /// %RigidTransform.
  RigidTransform(  // NOLINT(runtime/explicit)
      const Eigen::Translation<T, 3>& translation) {
    set_translation(translation.translation());
  }

  /// Constructs a %RigidTransform from an Eigen Isometry3.
  /// @param[in] pose Isometry3 that contains an allegedly valid rotation matrix
  /// `R_AB` and also contains a position vector `p_AoBo_A` from frame A's
  /// origin to frame B's origin.  `p_AoBo_A` must be expressed in frame A.
  /// @throws std::logic_error in debug builds if R_AB is not a proper
  /// orthonormal 3x3 rotation matrix.
  /// @note No attempt is made to orthogonalize the 3x3 rotation matrix part of
  /// `pose`.  As needed, use RotationMatrix::ProjectToRotationMatrix().
  explicit RigidTransform(const Isometry3<T>& pose) { SetFromIsometry3(pose); }

  /// Sets `this` %RigidTransform from a RotationMatrix and a position vector.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  /// @param[in] p position vector from frame A's origin to frame B's origin,
  /// expressed in frame A.  In monogram notation p is denoted `p_AoBo_A`.
  void set(const RotationMatrix<T>& R, const Vector3<T>& p) {
    set_rotation(R);
    set_translation(p);
  }

  /// Sets `this` %RigidTransform from an Eigen Isometry3.
  /// @param[in] pose Isometry3 that contains an allegedly valid rotation matrix
  /// `R_AB` and also contains a position vector `p_AoBo_A` from frame A's
  /// origin to frame B's origin.  `p_AoBo_A` must be expressed in frame A.
  /// @throws std::logic_error in debug builds if R_AB is not a proper
  /// orthonormal 3x3 rotation matrix.
  /// @note No attempt is made to orthogonalize the 3x3 rotation matrix part of
  /// `pose`.  As needed, use RotationMatrix::ProjectToRotationMatrix().
  void SetFromIsometry3(const Isometry3<T>& pose) {
    set(RotationMatrix<T>(pose.linear()), pose.translation());
  }

  /// Creates a %RigidTransform templatized on a scalar type U from a
  /// %RigidTransform templatized on scalar type T.  For example,
  /// ```
  /// RigidTransform<double> source = RigidTransform<double>::Identity();
  /// RigidTransform<AutoDiffXd> foo = source.cast<AutoDiffXd>();
  /// ```
  /// @tparam U Scalar type on which the returned %RigidTransform is templated.
  /// @note `RigidTransform<From>::cast<To>()` creates a new
  /// `RigidTransform<To>` from a `RigidTransform<From>` but only if type `To`
  /// is constructible from type `From`.  This cast method works in accordance
  /// with Eigen's cast method for Eigen's objects that underlie this
  /// %RigidTransform.  For example, Eigen currently allows cast from type
  /// double to AutoDiffXd, but not vice-versa.
  template <typename U>
  RigidTransform<U> cast() const {
    const RotationMatrix<U> R = R_AB_.template cast<U>();
    const Vector3<U> p = p_AoBo_A_.template cast<U>();
    return RigidTransform<U>(R, p);
  }

  /// Returns the identity %RigidTransform (corresponds to coincident frames).
  /// @return the %RigidTransform that corresponds to aligning the two frames so
  /// unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with Bo.
  /// Hence, the returned %RigidTransform contains a 3x3 identity matrix and a
  /// zero position vector.
  static const RigidTransform<T>& Identity() {
    // @internal This method's name was chosen to mimic Eigen's Identity().
    static const never_destroyed<RigidTransform<T>> kIdentity;
    return kIdentity.access();
  }

  /// Returns R_AB, the rotation matrix portion of `this` %RigidTransform.
  /// @retval R_AB the rotation matrix portion of `this` %RigidTransform.
  const RotationMatrix<T>& rotation() const { return R_AB_; }

  /// Sets the %RotationMatrix portion of `this` %RigidTransform.
  /// @param[in] R rotation matrix relating frames A and B (e.g., `R_AB`).
  void set_rotation(const RotationMatrix<T>& R) { R_AB_ = R; }

  /// Returns `p_AoBo_A`, the position vector portion of `this` %RigidTransform,
  /// i.e., position vector from Ao (frame A's origin) to Bo (frame B's origin).
  const Vector3<T>& translation() const { return p_AoBo_A_; }

  /// Sets the position vector portion of `this` %RigidTransform.
  /// @param[in] p position vector from Ao (frame A's origin) to Bo (frame B's
  /// origin) expressed in frame A.  In monogram notation p is denoted p_AoBo_A.
  void set_translation(const Vector3<T>& p) { p_AoBo_A_ = p; }

  /// Returns the 4x4 matrix associated with this %RigidTransform, i.e., X_AB.
  /// <pre>
  ///  ┌                ┐
  ///  │ R_AB  p_AoBo_A │
  ///  │                │
  ///  │   0      1     │
  ///  └                ┘
  /// </pre>
  Matrix4<T> GetAsMatrix4() const {
    Matrix4<T> pose;
    pose.template topLeftCorner<3, 3>() = rotation().matrix();
    pose.template topRightCorner<3, 1>() = translation();
    pose.row(3) = Vector4<T>(0, 0, 0, 1);
    return pose;
  }

  /// Returns the 3x4 matrix associated with this %RigidTransform, i.e., X_AB.
  /// <pre>
  ///  ┌                ┐
  ///  │ R_AB  p_AoBo_A │
  ///  └                ┘
  /// </pre>
  Eigen::Matrix<T, 3, 4> GetAsMatrix34() const {
    Eigen::Matrix<T, 3, 4> pose;
    pose.template topLeftCorner<3, 3>() = rotation().matrix();
    pose.template topRightCorner<3, 1>() = translation();
    return pose;
  }

  /// Returns the isometry in ℜ³ that is equivalent to a %RigidTransform.
  Isometry3<T> GetAsIsometry3() const {
    // pose.linear() returns a mutable reference to the 3x3 rotation matrix part
    // of Isometry3 and pose.translation() returns a mutable reference to the
    // 3x1 position vector part of the Isometry3.
    Isometry3<T> pose;
    pose.linear() = rotation().matrix();
    pose.translation() = translation();
    pose.makeAffine();
    return pose;
  }

  /// Sets `this` %RigidTransform so it corresponds to aligning the two frames
  /// so unit vectors Ax = Bx, Ay = By, Az = Bz and point Ao is coincident with
  /// Bo.  Hence, `this` %RigidTransform contains a 3x3 identity matrix and a
  /// zero position vector.
  const RigidTransform<T>& SetIdentity() {
    set(RotationMatrix<T>::Identity(), Vector3<T>::Zero());
    return *this;
  }

  /// Returns `true` if `this` is exactly the identity %RigidTransform.
  /// @see IsIdentityToEpsilon().
  boolean<T> IsExactlyIdentity() const {
    const boolean<T> is_position_zero = (translation() == Vector3<T>::Zero());
    return is_position_zero && rotation().IsExactlyIdentity();
  }

  /// Return true if `this` is within tolerance of the identity %RigidTransform.
  /// @returns `true` if the RotationMatrix portion of `this` satisfies
  /// RotationMatrix::IsIdentityToInternalTolerance() and if the position vector
  /// portion of `this` is equal to zero vector within `translation_tolerance`.
  /// @param[in] translation_tolerance a non-negative number.  One way to choose
  /// `translation_tolerance` is to multiply a characteristic length
  /// (e.g., the magnitude of a characteristic position vector) by an epsilon
  /// (e.g., RotationMatrix::get_internal_tolerance_for_orthonormality()).
  /// @see IsExactlyIdentity().
  boolean<T> IsIdentityToEpsilon(double translation_tolerance) const {
    const T max_component = translation().template lpNorm<Eigen::Infinity>();
    return max_component <= translation_tolerance &&
        rotation().IsIdentityToInternalTolerance();
  }

  /// Returns X_BA = X_AB⁻¹, the inverse of `this` %RigidTransform.
  /// @note The inverse of %RigidTransform X_AB is X_BA, which contains the
  /// rotation matrix R_BA = R_AB⁻¹ = R_ABᵀ and the position vector `p_BoAo_B_`
  /// (position from B's origin Bo to A's origin Ao, expressed in frame B).
  /// @note: The square-root of a %RigidTransform's condition number is roughly
  /// the magnitude of the position vector.  The accuracy of the calculation for
  /// the inverse of a %RigidTransform drops off with the sqrt condition number.
  RigidTransform<T> inverse() const {
    // @internal This method's name was chosen to mimic Eigen's inverse().
    const RotationMatrix<T> R_BA = R_AB_.inverse();
    return RigidTransform<T>(R_BA, R_BA * (-p_AoBo_A_));
  }

#ifndef DRAKE_DOXYGEN_CXX
  /// Until #9865 is resolved, this operator temporarily allows users mixing the
  /// use of %RigidTransform with Isometry3.
  DRAKE_DEPRECATED("2019-07-01",
                   "Do not mix RigidTransform with Isometry3. Only use "
                   "RigidTransform per #9865.")
  RigidTransform<T> operator*(const Isometry3<T>& isometry3) const {
    return *this * RigidTransform<T>(isometry3);
  }

  // DO NOT USE. These methods will soon be deprecated as #9865 is resolved.
  // They are only provided to support backwards compatibility with
  // Isometry3 as we migrate Drake's codebase to use RigidTransform. New uses of
  // Isometry3 are discouraged.
  operator Isometry3<T>() const { return GetAsIsometry3(); }
  const Matrix3<T>& linear() const { return R_AB_.matrix(); }
  Matrix4<T> matrix() const { return GetAsMatrix4(); }
#endif

  /// In-place multiply of `this` %RigidTransform `X_AB` by `other`
  /// %RigidTransform `X_BC`.
  /// @param[in] other %RigidTransform that post-multiplies `this`.
  /// @returns `this` %RigidTransform which has been multiplied by `other`.
  /// On return, `this = X_AC`, where `X_AC = X_AB * X_BC`.
  RigidTransform<T>& operator*=(const RigidTransform<T>& other) {
    p_AoBo_A_ = *this * other.translation();
    R_AB_ *= other.rotation();
    return *this;
  }

  /// Multiplies `this` %RigidTransform `X_AB` by the `other` %RigidTransform
  /// `X_BC` and returns the %RigidTransform `X_AC = X_AB * X_BC`.
  RigidTransform<T> operator*(const RigidTransform<T>& other) const {
    const Vector3<T> p_AoCo_A = *this * other.translation();
    return RigidTransform<T>(rotation() * other.rotation(), p_AoCo_A);
  }

  /// Multiplies `this` %RigidTransform `X_AB` by the translation-only transform
  /// `X_BBq` and returns the %RigidTransform `X_ABq = X_AB * X_BBq`.
  /// @note The rotation matrix in the returned %RigidTransform `X_ABq` is equal
  /// to the rotation matrix in `X_AB`.  `X_ABq` and `X_AB` only differ by
  /// origin location.
  RigidTransform<T> operator*(const Eigen::Translation<T, 3>& X_BBq) const {
    const RigidTransform<T>& X_AB = *this;
    const Vector3<T> p_ABq_A = X_AB * X_BBq.translation();
    return RigidTransform<T>(rotation(), p_ABq_A);
  }

  /// Multiplies the translation-only transform `X_AAq` by the %RigidTransform
  /// `X_AqB` and returns the %RigidTransform `X_AB = X_AAq * X_AqB`.
  /// @note The rotation matrix in the returned %RigidTransform `X_AB` is equal
  /// to the rotation matrix in `X_AqB`.  `X_AB` and `X_AqB` only differ by
  /// origin location.
  friend RigidTransform<T> operator*(const Eigen::Translation<T, 3>& X_AAq,
      const RigidTransform<T>& X_AqB) {
    const Vector3<T>& p_AAq_A = X_AAq.translation();
    const Vector3<T>& p_AqB_A = X_AqB.translation();
    const Vector3<T>& p_AB_A = p_AAq_A + p_AqB_A;
    const RotationMatrix<T>& R_AB = X_AqB.rotation();
    return RigidTransform<T>(R_AB, p_AB_A);
  }

  /// Multiplies `this` %RigidTransform `X_AB` by the position vector
  /// `p_BoQ_B` which is from Bo (B's origin) to an arbitrary point Q.
  /// @param[in] p_BoQ_B position vector from Bo to Q, expressed in frame B.
  /// @retval p_AoQ_A position vector from Ao to Q, expressed in frame A.
  Vector3<T> operator*(const Vector3<T>& p_BoQ_B) const {
    return p_AoBo_A_ + R_AB_ * p_BoQ_B;
  }

  /// Multiplies `this` %RigidTransform `X_AB` by the n position vectors
  /// `p_BoQ1_B` ... `p_BoQn_B`, where `p_BoQi_B` is the iᵗʰ position vector
  /// from Bo (frame B's origin) to an arbitrary point Qi, expressed in frame B.
  /// @param[in] p_BoQ_B `3 x n` matrix with n position vectors `p_BoQi_B`.
  /// @retval p_AoQ_A `3 x n` matrix with n position vectors `p_AoQi_A`, i.e., n
  /// position vectors from Ao (frame A's origin) to Qi, expressed in frame A.
  /// @code{.cc}
  /// const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  /// const RigidTransform<double> X_AB(rpy, Vector3d(1, 2, 3));
  /// Eigen::Matrix<double, 3, 2> p_BoQ_B;
  /// p_BoQ_B.col(0) = Vector3d(4, 5, 6);
  /// p_BoQ_B.col(1) = Vector3d(9, 8, 7);
  /// const Eigen::Matrix<double, 3, 2> p_AoQ_A = X_AB * p_BoQ_B;
  /// @endcode
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime>
  operator*(const Eigen::MatrixBase<Derived>& p_BoQ_B) const {
    if (p_BoQ_B.rows() != 3) {
      throw std::logic_error(
          "Error: Inner dimension for matrix multiplication is not 3.");
    }
    // Express position vectors in terms of frame A as p_BoQ_A = R_AB * p_BoQ_B.
    const RotationMatrix<typename Derived::Scalar> &R_AB = rotation();
    const Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime>
        p_BoQ_A = R_AB.matrix() * p_BoQ_B;

    // Reserve space (on stack on heap) to store the result.
    const int number_of_position_vectors = p_BoQ_B.cols();
    Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime>
        p_AoQ_A(3, number_of_position_vectors);

    // Create each returned position vector as p_AoQi_A = p_AoBo_A + p_BoQi_A.
    for (int i = 0;  i < number_of_position_vectors;  ++i)
      p_AoQ_A.col(i) = translation() + p_BoQ_A.col(i);

    return p_AoQ_A;
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RigidTransform to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// elements in `this` and `other`.
  /// @returns `true` if `‖this.matrix() - other.matrix()‖∞ <= tolerance`.
  /// @note Consider scaling tolerance with the largest of magA and magB, where
  /// magA and magB denoted the magnitudes of `this` position vector and `other`
  /// position vectors, respectively.
  boolean<T> IsNearlyEqualTo(const RigidTransform<T>& other,
                             double tolerance) const {
    return GetMaximumAbsoluteDifference(other) <= tolerance;
  }

  /// Returns true if `this` is exactly equal to `other`.
  /// @param[in] other %RigidTransform to compare to `this`.
  /// @returns `true` if each element of `this` is exactly equal to the
  /// corresponding element of `other`.
  boolean<T> IsExactlyEqualTo(const RigidTransform<T>& other) const {
    return rotation().IsExactlyEqualTo(other.rotation()) &&
           translation() == other.translation();
  }

  /// Computes the infinity norm of `this` - `other` (i.e., the maximum absolute
  /// value of the difference between the elements of `this` and `other`).
  /// @param[in] other %RigidTransform to subtract from `this`.
  /// @returns ‖`this` - `other`‖∞
  T GetMaximumAbsoluteDifference(const RigidTransform<T>& other) const {
    const Eigen::Matrix<T, 3, 4> diff = GetAsMatrix34() - other.GetAsMatrix34();
    return diff.template lpNorm<Eigen::Infinity>();
  }

  /// Returns the maximum absolute value of the difference in the position
  /// vectors (translation) in `this` and `other`.  In other words, returns
  /// the infinity norm of the difference in the position vectors.
  /// @param[in] other %RigidTransform whose position vector is subtracted from
  /// the position vector in `this`.
  T GetMaximumAbsoluteTranslationDifference(
      const RigidTransform<T>& other) const {
    const Vector3<T> p_difference = translation() - other.translation();
    return p_difference.template lpNorm<Eigen::Infinity>();
  }

 private:
  // Make RigidTransform<U> templatized on any typename U be a friend of a
  // %RigidTransform templatized on any other typename T. This is needed for the
  // method RigidTransform<T>::cast<U>() to be able to use the required private
  // constructor.
  template <typename U>
  friend class RigidTransform;

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  // The default constructor for R_AB_ is an identity matrix.
  RotationMatrix<T> R_AB_;

  // Position vector from A's origin Ao to B's origin Bo, expressed in A.
  Vector3<T> p_AoBo_A_;
};

/// Abbreviation (alias/typedef) for a RigidTransform double scalar type.
/// @relates RigidTransform
using RigidTransformd = RigidTransform<double>;

}  // namespace math
}  // namespace drake
