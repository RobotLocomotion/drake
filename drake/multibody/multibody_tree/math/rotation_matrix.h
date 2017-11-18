#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class represents a 3x3 rotation matrix between two arbitrary frames
/// A and B.  It relates right-handed orthogonal unit vectors Ax, Ay, Az
/// fixed in frame A to right-handed orthogonal unit vectors Bx, By, Bz
/// fixed in frame B.  In monogram notation, the rotation matrix relating A to B
/// is denoted R_AB.  For monogram notation, see @ref multibody_spatial_pose.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix)

  /// Constructs a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning the two frames (unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() {}

  /// @returns a 3x3 identity %RotationMatrix.
  static RotationMatrix<T> MakeIdentity() { return RotationMatrix(); }

  /// Makes a %RotationMatrix from an Matrix3 R.  No check is performed
  /// to test whether or not R is a valid rotation matrix.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @retval R_AB rotation matrix with underlying Matrix3 R.
  static RotationMatrix<T> MakeUnchecked(const Matrix3<T>& R) {
     return RotationMatrix(R);
  }

  /// Makes a %RotationMatrix from a Matrix3 R. This method throws
  /// an exception if R violates IsValidRotationMatrix().
  /// @param[in] R an allegedly valid rotation matrix.
  /// @param[in] tolerance parameter used in call to IsValidRotationMatrix.
  /// @retval R_AB rotation matrix with underlying Matrix3 R.
  /// @see IsValidRotationMatrix.
  static RotationMatrix<T> MakeChecked(const Matrix3<T>& R,
                                       const double tolerance) {
    RotationMatrix R_to_check = RotationMatrix<T>::MakeUnchecked(R);
    DRAKE_DEMAND(R_to_check.IsValid(tolerance));
    return R_to_check;
  }

  /// Const access to the Matrix3 underlying a %RotationMatrix.
  const Matrix3<T>& matrix() const { return R_AB_; }

  /// @retval R_ABᵀ, the transpose of this %RotationMatrix.
  /// @note For a valid rotation matrix R_BA = R_AB⁻¹ = R_ABᵀ.
  RotationMatrix<T> transpose() const {
    return RotationMatrix(R_AB_.transpose());
  }

  /// @retval R_AB⁻¹, the inverse (transpose) of this %RotationMatrix.
  /// @note For a valid rotation matrix R_BA = R_AB⁻¹ = R_ABᵀ.
  RotationMatrix<T> inverse() const { return transpose(); }

  /// Const access to the i, j component of this %RotationMatrix.
  /// The bounds on i, j are only checked in Debug builds.
  const T& operator()(int i, int j) const {
    DRAKE_ASSERT(0 <= i && i < 3 && 0 <= j && j < 3);
    return R_AB_(i, j);
  }

  /// Operator to multiply `this` rotation matrix by `other` rotation matrix.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns rotation matrix that results from `this` multiplied by `other`.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    return RotationMatrix<T>(matrix() * other.matrix());
  }

  /// Tests if `this` rotation matrix is a valid right-handed, orthonormal
  /// matrix to within the threshold specified by `tolerance`.
  /// @param[in] tolerance maximum allowable absolute difference between
  /// `this` * Transpose(`this`) and the identity matrix I.
  /// In other words, ‖R_AB ⋅ R_ABᵀ - I‖∞ <= tolerance.
  /// @return `true` if the rotation matrix is valid, otherwise `false`.
  bool IsValid(double tolerance) const {
    const RotationMatrix this_multiplied_by_transpose = (*this) * transpose();
    const RotationMatrix identity_matrix;
    if (this_multiplied_by_transpose.IsNearlyEqualTo(identity_matrix,
      tolerance) == false) return false;
    // A rotation matrix with right-handed bases has a determinant of +1.
    // A rotation matrix with left-handed bases has a determinant of -1.
    // Check that the rotation matrix is more right-handed than left-handed.
    return R_AB_.determinant() > 0;
  }

  /// Computes the infinity norm of `this` - `other`, i.e., the maximum absolute
  /// value of the difference between the elements of `this` and `other`.
  /// @param[in] other %RotationMatrix to subtract from `this`.
  /// @returns ‖`this` - `other`‖∞
  T GetMaximumAbsoluteDifference(const RotationMatrix& other) const {
    Matrix3<T> R_difference = matrix() - other.matrix();
    return R_difference.template lpNorm<Eigen::Infinity>();
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if ‖`this` - `other`‖∞ <= tolerance.
  bool IsNearlyEqualTo(const RotationMatrix& other, double tolerance) const {
    const T R_max_difference = GetMaximumAbsoluteDifference(other);
    return R_max_difference <= tolerance;
  }

 private:
  // Construct a %RotationMatrix from a Matrix3.
  // @param[in] R 3x3 matrix that should be a valid rotation matrix.
  // @note No check is performed to ensure R is a valid rotation matrix.
  explicit RotationMatrix(const Matrix3<T>& R) { R_AB_ = R;}

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  // The default initialization is the identity matrix.
  Matrix3<T> R_AB_{Matrix3<T>::Identity()};
};

}  // namespace multibody
}  // namespace drake
