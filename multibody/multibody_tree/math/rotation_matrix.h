#pragma once

#include <limits>
#include <type_traits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {

/// This class represents a 3x3 rotation matrix between two arbitrary frames
/// A and B and helps ensure users create valid rotation matrices.  This class
/// relates right-handed orthogonal unit vectors Ax, Ay, Az fixed in frame A
/// to right-handed orthogonal unit vectors Bx, By, Bz fixed in frame B.
/// The monogram notation for the rotation matrix relating A to B is `R_AB`.
/// See @ref multibody_quantities for monogram notation for dynamics.
/// See @ref orientation_discussion "for a discussion on rotation matrices".
///
/// @note This class does not store the frames associated with a rotation matrix
/// nor does it enforce strict proper usage of this class with vectors.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
// TODO(Mitiguy) Ensure this class handles RotationMatrix<symbolic::Expression>.
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix)

  /// Construct a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() {}

  /// Construct a %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws exception std::logic_error in debug builds if R fails IsValid(R).
  explicit RotationMatrix(const Matrix3<T>& R) : R_AB_() {
#ifdef DRAKE_ASSERT_IS_ARMED
    SetOrThrowIfNotValid(R);
#else
    SetUnchecked(R);
#endif
  }

  /// Set `this` %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws exception std::logic_error in debug builds if R fails IsValid(R).
  void SetOrThrowIfNotValid(const Matrix3<T>& R) {
    ThrowIfNotValid(R);
    SetUnchecked(R);
  }

  /// @returns a 3x3 identity %RotationMatrix.
  static RotationMatrix<T> MakeIdentity() { return RotationMatrix(); }

  /// @retval R_BA = R_AB⁻¹, the inverse (transpose) of this %RotationMatrix.
  /// @note For a valid rotation matrix R_BA = R_AB⁻¹ = R_ABᵀ.
  RotationMatrix<T> inverse() const {
    return RotationMatrix<T>(R_AB_.transpose());
  }

  /// Const access to the Matrix3 underlying a %RotationMatrix.
  const Matrix3<T>& matrix() const { return R_AB_; }

  /// Operator to multiply `this` rotation matrix `R_AB` by `other` rotation
  /// matrix `R_BC`.  On return, `this` is set to equal `R_AB * R_BC`.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns `this` rotation matrix which has been multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T>& operator*=(const RotationMatrix<T>& other) {
    SetUnchecked(matrix() * other.matrix());
    return *this;
  }

  /// Operator to multiply `this` rotation matrix `R_AB` by `other` rotation
  /// matrix `R_BC`, returning the composition `R_AB * R_BC`.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns rotation matrix that results from `this` multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    return RotationMatrix<T>(matrix() * other.matrix(), true);
  }

  /// Operator to multiply `this` rotation matrix `R` by an arbitrary Vector3.
  /// @param[in] v 3x1 vector that post-multiplies `this`.
  /// @returns 3x1 vector that results from `R * v`.
  Vector3<T> operator*(const Vector3<T>& v) const {
    return Vector3<T>(matrix() * v);
  }

  /// Returns how close the matrix R is to to being a 3x3 orthonormal matrix by
  /// computing ‖R ⋅ R⁻¹ - I‖∞ (i.e., the maximum absolute value of the
  /// difference between the elements of R ⋅ R⁻¹ and the 3x3 identity matrix).
  /// @param[in] R matrix being checked for orthonormality.
  /// @returns ‖R ⋅ R⁻¹ - I‖∞
  static T GetMeasureOfOrthonormality(const Matrix3<T>& R) {
    const Matrix3<T> m = R * R.transpose();
    return GetMaximumAbsoluteDifference(m, Matrix3<T>::Identity());
  }

  /// Tests if the determinant of a generic Matrix3 is positive or negative.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @return `true` if the determinant of R is positive, otherwise `false`.
  /// @internal The determinant of an proper rotation matrix is +1, whereas for
  /// an improper rotation matrix it is -1.  To avoid testing near +1 (which
  /// can return a false negative if the matrix is slightly non-orthonormal),
  /// all that is needed is to ensure the matrix is reasonably "proper" is to
  /// ensure the determinant is positive.  If the determinant is 0, it means
  /// the basis vectors are not linearly independent (do not span 3D space).
  static bool IsDeterminantPositive(const Matrix3<T>& R) {
    return R.determinant() > 0;
  }

  /// Tests if a generic Matrix3 has orthonormal vectors to within the threshold
  /// specified by `tolerance`.
  /// @param[in] R an allegedly orthonormal rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference between R * R⁻¹
  /// and the identity matrix I, i.e., checks if ‖R ⋅ R⁻¹ - I‖∞ <= tolerance.
  /// @return `true` if R is an orthonormal matrix, otherwise `false`.
  static bool IsOrthonormal(const Matrix3<T>& R, double tolerance) {
    return GetMeasureOfOrthonormality(R) <= tolerance;
  }

  /// Tests if a generic Matrix3 seems to be a proper orthonormal rotation
  /// matrix to within the threshold specified by `tolerance`.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference of `R * R⁻¹`
  /// and the identity matrix I (i.e., checks if ‖R ⋅ R⁻¹ - I‖∞ <= tolerance).
  /// @return `true` if R is a valid rotation matrix, otherwise `false`.
  static bool IsValid(const Matrix3<T>& R, double tolerance) {
    return IsOrthonormal(R, tolerance) && IsDeterminantPositive(R);
  }

  /// Tests if a generic Matrix3 is a proper orthonormal rotation matrix to
  /// within the threshold of get_internal_tolerance_for_orthonormality().
  /// @param[in] R an allegedly valid rotation matrix.
  /// @return `true` if R is a valid rotation matrix, otherwise `false`.
  static bool IsValid(const Matrix3<T>& R) {
    return IsValid(R, get_internal_tolerance_for_orthonormality());
  }

  /// Tests if `this` rotation matrix R is a proper orthonormal rotation matrix
  /// to within the threshold of get_internal_tolerance_for_orthonormality().
  /// @return `true` if `this` is a valid rotation matrix, otherwise `false`.
  bool IsValid() const { return IsValid(matrix()); }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if ‖`this` - `other`‖∞ <= tolerance.
  bool IsNearlyEqualTo(const RotationMatrix<T>& other, double tolerance) const {
    return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
  }

  /// Computes the infinity norm of `this` - `other` (i.e., the maximum absolute
  /// value of the difference between the elements of `this` and `other`).
  /// @param[in] other %RotationMatrix to subtract from `this`.
  /// @returns ‖`this` - `other`‖∞
  T GetMaximumAbsoluteDifference(const RotationMatrix<T>& other) const {
    return GetMaximumAbsoluteDifference(matrix(), other.matrix());
  }

  /// Create an orthonormal matrix 'R" from a 3x3 matrix `M` by
  /// minimizing ‖`R` - `M`‖²,  subject to  R * Rᵀ = I.
  /// @param[in] M a full-rank 3x3 matrix.
  /// @returns proper orthonormal matrix `R` that is close to `M`.
  // @internal This function is not generated for symbolic Expression.
  template <typename T1 = T>
  static typename std::enable_if<!std::is_same<T1, symbolic::Expression>::value,
                                 RotationMatrix<T1>>::type
  ProjectToRotationMatrix(const Matrix3<T1>& M) {
    return RotationMatrix<T1>(math::ProjectMatToOrthonormalMat(M));
  }

  /// @return internal tolerance (small multiplier of double-precision epsilon)
  /// used to check whether or not a rotation matrix is orthonormal.
  /// @note The tolerance is chosen by developers to ensure a reasonably
  /// valid (orthonormal) rotation matrix.
  /// @note To orthonormalize a 3x3 matrix, use ProjectMatrixToRotationMatrix().
  static double get_internal_tolerance_for_orthonormality() {
    return kInternalToleranceForOrthonormality_;
  }

 private:
  // Declare the allowable tolerance (small multiplier of double-precision
  // epsilon) used to check whether or not a rotation matrix is orthonormal.
  static constexpr double kInternalToleranceForOrthonormality_{
      128 * std::numeric_limits<double>::epsilon() };

  // Construct a %RotationMatrix from a Matrix3.  No check is performed to test
  // whether or not the parameter R is a valid rotation matrix.
  // @param[in] R an allegedly valid rotation matrix.
  // @note The second parameter is just a dummy to distinguish this constructor
  // from one of the public constructors.
  RotationMatrix(const Matrix3<T>& R, bool) : R_AB_(R) {}

  // Set `this` %RotationMatrix from a Matrix3.  No check is performed to
  // test whether or not the parameter R is a valid rotation matrix.
  // @param[in] R an allegedly valid rotation matrix.
  void SetUnchecked(const Matrix3<T>& R) { R_AB_ = R; }

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
  // @param[in] R an allegedly valid rotation matrix.
  static void ThrowIfNotValid(const Matrix3<T>& R) {
    if (!IsOrthonormal(R, get_internal_tolerance_for_orthonormality()))
      throw std::logic_error("Error: Rotation matrix is not orthonormal.");
    if (!IsDeterminantPositive(R))
      throw std::logic_error("Error: Rotation matrix determinant is negative. "
                             "It is possible a basis is left-handed");
  }

  // Rotation matrix relating two frames, e.g. frame A and frame B.
  // The default initialization is the identity matrix.
  Matrix3<T> R_AB_{Matrix3<T>::Identity()};
};

}  // namespace multibody
}  // namespace drake
