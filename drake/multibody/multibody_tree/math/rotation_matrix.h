#pragma once

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class represents a 3x3 rotation matrix between two arbitrary frames
/// A and B.  It relates right-handed orthogonal unit vectors Ax, Ay, Az
/// fixed in frame A to right-handed orthogonal unit vectors Bx, By, Bz
/// fixed in frame B.  The monogram notation for the rotation matrix relating A
/// to B is `R_AB`.  See @ref multibody_notation_basics for monogram notation.
/// @note This class helps ensure users create valid rotation matrices.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
// TODO(Mitiguy) Add link to orientation in multibody_doxygen.h
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix)

  /// Constructs a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning the two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() {}

  /// Construct a %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid right-handed rotation matrix.
  /// @throws exception std::logic_error if R violates IsValid(R, tolerance),
  /// where tolerance is a small multiplier of double-precision epsilon.
  explicit RotationMatrix(const Matrix3<T>& R) : R_AB_() {
    SetOrThrowIfNotValid(R, internal_tolerance_);
  }

  /// Sets the underlying Matrix3 in a %RotationMatrix.
  /// @param[in] R an allegedly valid right-handed rotation matrix.
  /// @throws exception std::logic_error if R violates IsValid(R, tolerance),
  /// where tolerance is a small multiplier of double-precision epsilon.
  void SetOrThrowIfNotValid(const Matrix3<T>& R) {
    ThrowIfNotValid(R, internal_tolerance_);
    SetUnchecked(R);
  }

  /// @returns a 3x3 identity %RotationMatrix.
  static RotationMatrix<T> MakeIdentity() { return RotationMatrix(); }

  /// @retval R_BA = R_AB⁻¹, the inverse (transpose) of this %RotationMatrix.
  /// @note For a valid rotation matrix R_BA = R_AB⁻¹ = R_ABᵀ.
  RotationMatrix<T> Inverse() const {
    return RotationMatrix<T>(R_AB_.transpose());
  }

  /// Const access to the Matrix3 underlying a %RotationMatrix.
  const Matrix3<T>& matrix() const { return R_AB_; }

  /// Const access to the i, j component of this %RotationMatrix.
  /// The bounds on i, j are only checked in Debug builds.
  /// @remark A mutable version of operator() is intentionally absent to prevent
  /// users from directly setting elements or making invalid rotation matrices.
  const T& operator()(int i, int j) const {
    DRAKE_ASSERT(0 <= i && i < 3 && 0 <= j && j < 3);
    return R_AB_(i, j);
  }

  /// Operator to multiply `this` rotation matrix by `other` rotation matrix.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns `this` rotation matrix which has been multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T>& operator*=(const RotationMatrix<T>& other) {
    SetUnchecked(matrix() * other.matrix());
    return *this;
  }

  /// Operator to multiply `this` rotation matrix by `other` rotation matrix.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns rotation matrix that results from `this` multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    return RotationMatrix<T>(matrix() * other.matrix(), true);
  }

  /// Returns how close the matrix R is to to being a 3x3 orthonormal matrix by
  /// computing ‖R ⋅ R⁻¹ - I‖∞ R - I (i.e., the maximum absolute value of the
  /// difference between the elements of R ⋅ R⁻¹ and the 3x3 identity matrix).
  /// @param[in] R matrix being checked for orthonormality.
  /// @returns ‖R ⋅ R⁻¹ - I‖∞
  static T GetMeasureOfOrthonormality(const Matrix3<T>& R) {
    const Matrix3<T> m = R * R.transpose();
    return GetMaximumAbsoluteDifference(m, Matrix3<T>::Identity());
  }

  /// Tests if a generic Matrix3 has vectors that form right-handed bases.
  /// @param[in] R a matrix allegedly associated with right-handed bases.
  /// @return `true` if R has right-handed bases, otherwise `false`.
  /// @internal The determinant of an orthogonal rotation matrix that has right-
  /// handed bases is +1, whereas for left-handed bases the determinant is -1.
  /// For non-orthogonal non-unitary bases, the determinant is positive if
  /// the bases are right-handed, whereas the determinant is negative if
  /// the bases are left-handed (if determinant = 0, bases do not span 3D).
  /// Hence, check if the matrix is more right-handed than left-handed.
  static bool IsRightHanded(const Matrix3<T>& R) { return R.determinant() > 0; }

  /// Tests if a generic Matrix3 has orthogonal unit vectors to within the
  /// threshold specified by `tolerance`.
  /// @param[in] R an allegedly orthonormal rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference between R * R⁻¹
  /// and the identity matrix I, i.e., checks if ‖R ⋅ R⁻¹ - I‖∞ <= tolerance.
  /// @return `true` if R is an othogonal matrix, otherwise `false`.
  static bool IsOrthogonal(const Matrix3<T>& R, double tolerance) {
    return GetMeasureOfOrthonormality(R) <= tolerance;
  }

  /// Tests if a generic Matrix3 seems to be a valid right-handed, orthonormal
  /// rotation matrix to within the threshold specified by `tolerance`.
  /// @param[in] R an allegedly valid right-handed orthonormal rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference between R * R⁻¹
  /// and the identity matrix I, i.e., checks if ‖R ⋅ R⁻¹ - I‖∞ <= tolerance.
  /// @return `true` if R is a valid rotation matrix, otherwise `false`.
  static bool IsValid(const Matrix3<T>& R, double tolerance) {
    return IsOrthogonal(R, tolerance) && IsRightHanded(R);
  }

  /// Tests if `this` rotation matrix is a valid right-handed, orthonormal
  /// rotation matrix to within the threshold specified by `tolerance`.
  /// @param[in] tolerance maximum allowable absolute difference between
  /// `this` * Inverse(`this`) and the identity matrix I.
  /// In other words, checks if ‖R_AB_ ⋅ R_AB_⁻¹ - I‖∞ <= tolerance.
  /// @return `true` if the rotation matrix is valid, otherwise `false`.
  /// @note It is unlikely for a rotation matrix (post-construction) to be
  /// invalid.  One possible way is accumulated error with operator*().
  bool IsValid(double tolerance) const { return IsValid(matrix(), tolerance); }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if ‖`this` - `other`‖∞ <= tolerance.
  bool IsNearlyEqualTo(const RotationMatrix& other, double tolerance) const {
    return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
  }

  /// Computes the infinity norm of `this` - `other` (i.e., the maximum absolute
  /// value of the difference between the elements of `this` and `other`).
  /// @param[in] other %RotationMatrix to subtract from `this`.
  /// @returns ‖`this` - `other`‖∞
  T GetMaximumAbsoluteDifference(const RotationMatrix& other) const {
    return GetMaximumAbsoluteDifference(matrix(), other.matrix());
  }

 private:
  // Construct a %RotationMatrix from a Matrix3.  No check is performed to test
  // whether or not the parameter R is a valid rotation matrix.
  // @param[in] R an allegedly valid right-handed rotation matrix.
  RotationMatrix(const Matrix3<T>& R, bool) : R_AB_(R) {}

  // Sets the underlying Matrix3 in a %RotationMatrix.  No check is performed
  // to test whether or not the parameter R is a valid rotation matrix.
  // @param[in] R an allegedly valid 3x3 right-handed rotation matrix.
  void SetUnchecked(const Matrix3<T>& R) { R_AB_ = R; }

  // Sets the underlying Matrix3 in a %RotationMatrix.
  // @param[in] R an allegedly valid right-handed rotation matrix.
  // @param[in] tolerance parameter used in call to IsValid(R, `tolerance`),
  // which is usually a small multiplier of double-precision epsilon.
  // @throws exception std::logic_error if R violates IsValid(R, `tolerance`).
  void SetOrThrowIfNotValid(const Matrix3<T>& R, const double tolerance) {
    ThrowIfNotValid(R, tolerance);
    SetUnchecked(R);
  }

  // Computes the infinity norm of R - `other` (i.e., the maximum absolute
  // value of the difference between the elements of R and `other`).
  // @param[in] R matrix from which `other` is subtracted.
  // @param[in] other matrix to subtract from R.
  // @returns ‖R - `other`‖∞
  static T GetMaximumAbsoluteDifference(const Matrix3<T>& R,
                                        const Matrix3<T>& other) {
    const Matrix3<T> R_difference = R - other;
    return R_difference.template lpNorm<Eigen::Infinity>();
  }

  // Compares corresponding elements in two matrices to check if they are the
  // same to within a specified `tolerance`.
  // @param[in] R matrix to compare to `other`.
  // @param[in] other matrix to compare to R.
  // @param[in] tolerance maximum allowable absolute difference between the
  // matrix elements in R and `other`.
  // @returns `true` if ‖R - `other`‖∞ <= tolerance.
  static bool IsNearlyEqualTo(const Matrix3<T>& R, const Matrix3<T>& other,
                              double tolerance) {
    const T R_max_difference = GetMaximumAbsoluteDifference(R, other);
    return R_max_difference <= tolerance;
  }

  // Throws an exception if R is not a valid %RotationMatrix.
  // @param[in] R an allegedly valid right-handed rotation matrix.
  // @param[in] tolerance maximum allowable absolute difference between R * R⁻¹
  // and the identity matrix I, i.e., if ‖R ⋅ R⁻¹ - I‖∞ <= tolerance.
  static void ThrowIfNotValid(const Matrix3<T>& R, double tolerance) {
    if (!IsOrthogonal(R, tolerance))
      throw std::logic_error("Error: Rotation matrix is not orthogonal.");
    if (!IsRightHanded(R))
      throw std::logic_error("Error: Rotation matrix is not right-handed.");
  }

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  // The default initialization is the identity matrix.
  Matrix3<T> R_AB_{Matrix3<T>::Identity()};

  // Declare the allowable internal tolerance for a valid rotation matrix.
  static constexpr double internal_tolerance_{
      100 * std::numeric_limits<double>::epsilon() };
};

}  // namespace multibody
}  // namespace drake
