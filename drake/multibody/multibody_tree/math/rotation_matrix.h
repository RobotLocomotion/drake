#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This class is a 3x3 rotation matrix between two arbitrary frames A and B.
/// It relates right-handed orthogonal unit vectors Ax, Ay, Az fixed in frame A
/// to right-handed orthogonal unit vectors Bx, By, Bz fixed in frame B.
/// In monogram notation, the rotation matrix relating A to B is denoted R_AB.
/// For more on monogram notation, see @ref multibody_spatial_vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix)

  /// Constructs a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning the two frames (unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() {}

  /// Makes a %RotationMatrix from an Eigen Matrix3 `R`.  No check is
  /// performed to test whether or not `R' is a valid rotation matrix.
  /// @param[in] `R` an allegedly valid rotation matrix.
  static RotationMatrix<T> MakeUnchecked(const Eigen Matrix3<T>& R) {
     return RotationMatrix(R);
  }

  /// Makes a %RotationMatrix from an Eigen Matrix3 `R`. This method
  /// throws an exception if `R` violates IsValidRotationMatrix().
  /// @param[in] `R` an allegedly valid rotation matrix.
  /// @param[in] tolerance parameter used in call to IsValidRotationMatrix.
  /// @see IsValidRotationMatrix.
  static RotationMatrix<T> MakeChecked(const Eigen Matrix3<T>& R,
                                        const double tolerance) {
     DRAKE_DEMAND(IsValidRotationMatrix(tolerance));
     return MakeUnchecked(R);
  }

  /// Const access to the Eigen Matrix3 underlying a %RotationMatrix.
  const Eigen::Matrix3<T>& get_as_Matrix3() const { return R_AB_; }

  /// @returns R_BA, the tranpose of this %RotationMatrix.
  const RotationMatrix<T> transpose() const { return R_AB_.transpose(); }

  /// @returns R_BA, the inverse (transpose) of this %RotationMatrix.
  const RotationMatrix<T> inverse() const { return R_AB_.transpose(); }

  /// Const access to the i, j component of this %RotationMatrix. The bounds on
  /// i, j are only checked in Debug builds (avoids overhead in Release builds).
  const T& operator[](int i, int j) const {
    DRAKE_ASSERT(0 <= i && i < 3 && 0 <= j && j < 3);
    return R_AB_(i, j);
  }

  /// Operator to multiply `this` rotation matrix by `other` rotation matrix.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns rotation matrix that results from`this` multiplied by `other`.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    return RotationMatrix(get_as_Matrix3() * other.get_as_Matrix3());
  }

  /// Checks whether `this` rotation matrix seems valid by multiplying it by
  /// its tranpose (the transpose should be its inverse) and checking if
  /// `this` * Tranpose(`this`) is nearly the 3x3 identity matrix.  This method
  /// also checks for a right-handed basis by testing if the determinant of R_AB
  /// is positive (near 1).  Note: Left-handed bases yield a determinant of -1.
  /// @param[in] tolerance maximum allowable absolute difference between
  /// `this` * Transpose(`this`) and the identity matrix.  Tolerance has no
  /// units and normally has a value between 0 and 1 (normally near 0).
  /// @return `true` if the rotation matrix seems valid, otherwise `false`.
  bool IsValidRotationMatrix(const double tolerance) const {
    const RotationMatrix this_multiplied_by_transpose = (*this) * Transpose();
    const RotationMatrix identity_matrix();
    if (this_multiplied_by_transpose.IsNearlyEqualTo(identity_matrix, tolerance)
      == false) return false;
    const T determinant = get_as_Matrix3().determinant();
    return determinant >= 0.5;
  }

  /// Returns the maximum absolute value of the difference between elements of
  /// `this` and `other` (infinity norm of the difference in the elements).
  T GetMaximumAbsoluteDifference(const RotationMatrix& other) const {
    RotationMatrix<T> R_difference = get_as_Matrix3() - other.get_as_Matrix3();
    return R_difference.template lpNorm<Eigen::Infinity>();
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.  Tolerance has no units and
  /// normally has a value between 0 and 1 (normally near 0).
  /// @returns `true` if each element of `this` is equal to the corresponding
  /// element of `other`, within `tolerance`.
  bool IsNearlyEqualTo(const RotationMatrix& other, const T& tolerance) const {
    const T R_max_difference = GetMaximumAbsoluteDifferences(other);
    return R_max_difference <= rotational_tolerance;
  }

 private:
  // Construct a %RotationMatrix from an Eigen Matrix3.
  // @param[in] R 3x3 matrix that should be a valid rotation matrix.
  // @note No check is performed to ensure `R` is a valid rotation matrix.
  explicit RotationMatrix(const Eigen Matrix3<T>& R) { R_AB_(R);}

  // Mutable access to i, j component of this %RotationMatrix. The bounds on
  // i, j are only checked in Debug builds (avoids overhead in Release builds).
  T& operator[](int i, int j) {
    DRAKE_ASSERT(0 <= i && i < 3 && 0 <= j && j < 3);
    return R_AB_(i, j);
  }

  // Mutable access to the underlying Eigen Matrix3 in a %RotationMatrix.
  Eigen::Matrix3<T>& get_as_Matrix3() { return R_AB_; }

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  // The default initialization is the identity matrix.
  Matrix3<T> R_AB_{Eigen::Matrix3<T>::Identity()};
};

}  // namespace multibody
}  // namespace drake
