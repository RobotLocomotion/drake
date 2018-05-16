#pragma once

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace math {

/// This class represents a 3x3 rotation matrix between two arbitrary frames
/// A and B and helps ensure users create valid rotation matrices.  This class
/// relates right-handed orthogonal unit vectors Ax, Ay, Az fixed in frame A
/// to right-handed orthogonal unit vectors Bx, By, Bz fixed in frame B.
/// The monogram notation for the rotation matrix relating A to B is `R_AB`.
/// See @ref multibody_quantities for monogram notation for dynamics.
/// See @ref orientation_discussion "a discussion on rotation matrices".
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
// TODO(Mitiguy) Ensure class handles RotationMatrix<symbolic::Expression>.
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix)

  /// Constructs a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() {}

  /// Constructs a %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws std::logic_error in debug builds if R fails IsValid(R).
  explicit RotationMatrix(const Matrix3<T>& R) : R_AB_() {
#ifdef DRAKE_ASSERT_IS_ARMED
    SetOrThrowIfNotValid(R);
#else
    SetUnchecked(R);
#endif
  }

  /// Constructs a %RotationMatrix from an Eigen::Quaternion.
  /// @param[in] quaternion a non-zero, finite quaternion which may or may not
  /// have unit length [i.e., `quaterion.norm()` does not have to be 1].
  /// @throws std::logic_error in debug builds if the rotation matrix
  /// R that is built from `quaternion` fails IsValid(R).  For example, an
  /// exception is thrown if `quaternion` is zero or contains a NaN or infinity.
  /// @note This method has the effect of normalizing its `quaternion` argument,
  /// without the inefficiency of the square-root associated with normalization.
  // TODO(mitiguy) Although this method is fairly efficient, consider adding an
  // optional second argument if `quaternion` is known to be normalized apriori
  // or for some reason the calling site does not want `quaternion` normalized.
  explicit RotationMatrix(const Eigen::Quaternion<T>& quaternion) {
    // Cost for various way to create a rotation matrix from a quaternion.
    // Eigen quaternion.toRotationMatrix() = 12 multiplies, 12 adds.
    // Drake  QuaternionToRotationMatrix() = 12 multiplies, 12 adds.
    // Extra cost for two_over_norm_squared =  4 multiplies,  3 adds, 1 divide.
    // Extra cost if normalized = 4 multiplies, 3 adds, 1 sqrt, 1 divide.
    const T two_over_norm_squared = T(2) / quaternion.squaredNorm();
    R_AB_ = QuaternionToRotationMatrix(quaternion, two_over_norm_squared);
    DRAKE_ASSERT_VOID(ThrowIfNotValid(R_AB_));
  }

  /// Constructs a %RotationMatrix from an Eigen::AngleAxis.
  /// @param[in] theta_lambda an Eigen::AngleAxis whose associated axis (vector
  /// direction herein called `lambda`) is non-zero and finite, but which may or
  /// may not have unit length [i.e., `lambda.norm()` does not have to be 1].
  /// @throws std::logic_error in debug builds if the rotation matrix
  /// R that is built from `theta_lambda` fails IsValid(R).  For example, an
  /// exception is thrown if `lambda` is zero or contains a NaN or infinity.
  // @internal In general, the %RotationMatrix constructed by passing a non-unit
  // `lambda` to this method is different than the %RotationMatrix produced by
  // converting `lambda` to an un-normalized quaternion and calling the
  // %RotationMatrix constructor (above) with that un-normalized quaternion.
  // TODO(mitiguy) Consider adding an optional second argument if `lambda` is
  // known to be normalized apriori or calling site does not want normalization.
  explicit RotationMatrix(const Eigen::AngleAxis<T>& theta_lambda) {
    const Vector3<T>& lambda = theta_lambda.axis();
    const T norm = lambda.norm();
    const T& theta = theta_lambda.angle();
    R_AB_ = Eigen::AngleAxis<T>(theta, lambda / norm).toRotationMatrix();
    DRAKE_ASSERT_VOID(ThrowIfNotValid(R_AB_));
  }

  /// Constructs a %RotationMatrix from an %RollPitchYaw.  In other words,
  /// makes the %RotationMatrix for a Space-fixed (extrinsic) X-Y-Z rotation by
  /// "roll-pitch-yaw" angles `[r, p, y]`, which is equivalent to a Body-fixed
  /// (intrinsic) Z-Y-X rotation by "yaw-pitch-roll" angles `[y, p, r]`.
  /// @param[in] rpy radian measures of three angles [roll, pitch, yaw].
  /// @param[in] rpy a %RollPitchYaw which is a Space-fixed (extrinsic) X-Y-Z
  /// rotation with "roll-pitch-yaw" angles `[r, p, y]` or equivalently a Body-
  /// fixed (intrinsic) Z-Y-X rotation with "yaw-pitch-roll" angles `[y, p, r]`.
  /// @note Denoting roll `r`, pitch `p`, yaw `y`, this method returns a
  /// rotation matrix `R_AD` equal to the matrix multiplication shown below.
  /// ```
  ///        ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
  /// R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
  ///        ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
  ///      =       R_AB          *        R_BC          *        R_CD
  /// ```
  /// Note: In this discussion, A is the Space frame and D is the Body frame.
  /// One way to visualize this rotation sequence is by introducing intermediate
  /// frames B and C (useful constructs to understand this rotation sequence).
  /// Initially, the frames are aligned so `Di = Ci = Bi = Ai (i = x, y, z)`.
  /// Then D is subjected to successive right-handed rotations relative to A.
  /// @li 1st rotation R_CD: %Frame D rotates relative to frames C, B, A by a
  /// roll angle `r` about `Dx = Cx`.  Note: D and C are no longer aligned.
  /// @li 2nd rotation R_BC: Frames D, C (collectively -- as if welded together)
  /// rotate relative to frame B, A by a pitch angle `p` about `Cy = By`.
  /// Note: C and B are no longer aligned.
  /// @li 3rd rotation R_AB: Frames D, C, B (collectively -- as if welded)
  /// rotate relative to frame A by a roll angle `y` about `Bz = Az`.
  /// Note: B and A are no longer aligned.
  /// TODO(@mitiguy) Add Sherm/Goldstein's way to visualize rotation sequences.
  explicit RotationMatrix(const RollPitchYaw<T>& rpy) {
    const T &r = rpy.get_roll_angle();
    const T &p = rpy.get_pitch_angle();
    const T &y = rpy.get_yaw_angle();
    using std::sin;
    using std::cos;
    const T c0 = cos(r), c1 = cos(p), c2 = cos(y);
    const T s0 = sin(r), s1 = sin(p), s2 = sin(y);
    const T c2_s1 = c2 * s1, s2_s1 = s2 * s1;
    const T Rxx = c2 * c1;
    const T Rxy = c2_s1 * s0 - s2 * c0;
    const T Rxz = c2_s1 * c0 + s2 * s0;
    const T Ryx = s2 * c1;
    const T Ryy = s2_s1 * s0 + c2 * c0;
    const T Ryz = s2_s1 * c0 - c2 * s0;
    const T Rzx = -s1;
    const T Rzy = c1 * s0;
    const T Rzz = c1 * c0;
    R_AB_.row(0) << Rxx, Rxy, Rxz;
    R_AB_.row(1) << Ryx, Ryy, Ryz;
    R_AB_.row(2) << Rzx, Rzy, Rzz;
  }

  /// Makes the %RotationMatrix `R_AB` associated with rotating a frame B
  /// relative to a frame A by an angle `theta` about unit vector `Ax = Bx`.
  /// @param[in] theta radian measure of rotation angle about Ax.
  /// @note `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
  /// Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
  /// a right-handed rotation relative to A by an angle `theta` about `Ax = Bx`.
  /// ```
  ///        ⎡ 1       0                 0  ⎤
  /// R_AB = ⎢ 0   cos(theta)   -sin(theta) ⎥
  ///        ⎣ 0   sin(theta)    cos(theta) ⎦
  /// ```
  static RotationMatrix<T> MakeXRotation(const T& theta) {
    Matrix3<T> R;
    using std::sin;
    using std::cos;
    const T c = cos(theta), s = sin(theta);
    // clang-format off
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    // clang-format on
    return RotationMatrix(R);
  }

  /// Makes the %RotationMatrix `R_AB` associated with rotating a frame B
  /// relative to a frame A by an angle `theta` about unit vector `Ay = By`.
  /// @param[in] theta radian measure of rotation angle about Ay.
  /// @note `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
  /// Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
  /// a right-handed rotation relative to A by an angle `theta` about `Ay = By`.
  /// ```
  ///        ⎡  cos(theta)   0   sin(theta) ⎤
  /// R_AB = ⎢          0    1           0  ⎥
  ///        ⎣ -sin(theta)   0   cos(theta) ⎦
  /// ```
  static RotationMatrix<T> MakeYRotation(const T& theta) {
    Matrix3<T> R;
    using std::sin;
    using std::cos;
    const T c = cos(theta), s = sin(theta);
    // clang-format off
    R <<  c,  0,  s,
          0,  1,  0,
         -s,  0,  c;
    // clang-format on
    return RotationMatrix(R);
  }

  /// Makes the %RotationMatrix `R_AB` associated with rotating a frame B
  /// relative to a frame A by an angle `theta` about unit vector `Az = Bz`.
  /// @param[in] theta radian measure of rotation angle about Az.
  /// @note `R_AB` relates two frames A and B having unit vectors Ax, Ay, Az and
  /// Bx, By, Bz.  Initially, `Bx = Ax`, `By = Ay`, `Bz = Az`, then B undergoes
  /// a right-handed rotation relative to A by an angle `theta` about `Az = Bz`.
  /// ```
  ///        ⎡ cos(theta)  -sin(theta)   0 ⎤
  /// R_AB = ⎢ sin(theta)   cos(theta)   0 ⎥
  ///        ⎣         0            0    1 ⎦
  /// ```
  static RotationMatrix<T> MakeZRotation(const T& theta) {
    Matrix3<T> R;
    using std::sin;
    using std::cos;
    const T c = cos(theta), s = sin(theta);
    // clang-format off
    R << c, -s,  0,
         s,  c,  0,
         0,  0,  1;
    // clang-format on
    return RotationMatrix(R);
  }

  /// Creates a %RotationMatrix templatized on a scalar type U from a
  /// %RotationMatrix templatized on scalar type T.  For example,
  /// ```
  /// RotationMatrix<double> source = RotationMatrix<double>::Identity();
  /// RotationMatrix<AutoDiffXd> foo = source.cast<AutoDiffXd>();
  /// ```
  /// @tparam U Scalar type on which the returned %RotationMatrix is templated.
  /// @note `RotationMatrix<From>::cast<To>()` creates a new
  /// `RotationMatrix<To>` from a `RotationMatrix<From>` but only if
  /// type `To` is constructible from type `From`.
  /// This cast method works in accordance with Eigen's cast method for Eigen's
  /// %Matrix3 that underlies this %RotationMatrix.  For example, Eigen
  /// currently allows cast from type double to AutoDiffXd, but not vice-versa.
  template <typename U>
  RotationMatrix<U> cast() const {
    const Matrix3<U> m = R_AB_.template cast<U>();
    return RotationMatrix<U>(m, true);
  }

  /// Sets `this` %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws std::logic_error in debug builds if R fails IsValid(R).
  void SetOrThrowIfNotValid(const Matrix3<T>& R) {
    ThrowIfNotValid(R);
    SetUnchecked(R);
  }

  /// Returns the 3x3 identity %RotationMatrix.
  // @internal This method's name was chosen to mimic Eigen's Identity().
  static const RotationMatrix<T>& Identity() {
    static const never_destroyed<RotationMatrix<T>> kIdentity;
    return kIdentity.access();
  }

  /// Returns `R_BA = R_AB⁻¹`, the inverse (transpose) of this %RotationMatrix.
  /// @note For a valid rotation matrix `R_BA = R_AB⁻¹ = R_ABᵀ`.
  // @internal This method's name was chosen to mimic Eigen's inverse().
  RotationMatrix<T> inverse() const {
    return RotationMatrix<T>(R_AB_.transpose());
  }

  /// Returns the Matrix3 underlying a %RotationMatrix.
  const Matrix3<T>& matrix() const { return R_AB_; }

  /// In-place multiply of `this` rotation matrix `R_AB` by `other` rotation
  /// matrix `R_BC`.  On return, `this` is set to equal `R_AB * R_BC`.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns `this` rotation matrix which has been multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T>& operator*=(const RotationMatrix<T>& other) {
    SetUnchecked(matrix() * other.matrix());
    return *this;
  }

  /// Calculates `this` rotation matrix `R_AB` multiplied by `other` rotation
  /// matrix `R_BC`, returning the composition `R_AB * R_BC`.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns rotation matrix that results from `this` multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    return RotationMatrix<T>(matrix() * other.matrix(), true);
  }

  /// Calculates `this` rotation matrix R multiplied by an arbitrary Vector3.
  /// @param[in] v 3x1 vector that post-multiplies `this`.
  /// @returns 3x1 vector that results from `R * v`.
  Vector3<T> operator*(const Vector3<T>& v) const {
    return Vector3<T>(matrix() * v);
  }

  /// Returns how close the matrix R is to to being a 3x3 orthonormal matrix by
  /// computing `‖R ⋅ Rᵀ - I‖∞` (i.e., the maximum absolute value of the
  /// difference between the elements of R ⋅ Rᵀ and the 3x3 identity matrix).
  /// @param[in] R matrix being checked for orthonormality.
  /// @returns `‖R ⋅ Rᵀ - I‖∞`
  static T GetMeasureOfOrthonormality(const Matrix3<T>& R) {
    const Matrix3<T> m = R * R.transpose();
    return GetMaximumAbsoluteDifference(m, Matrix3<T>::Identity());
  }

  /// Tests if a generic Matrix3 has orthonormal vectors to within the threshold
  /// specified by `tolerance`.
  /// @param[in] R an allegedly orthonormal rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference between R * Rᵀ
  /// and the identity matrix I, i.e., checks if `‖R ⋅ Rᵀ - I‖∞ <= tolerance`.
  /// @returns `true` if R is an orthonormal matrix.
  static bool IsOrthonormal(const Matrix3<T>& R, double tolerance) {
    return GetMeasureOfOrthonormality(R) <= tolerance;
  }

  /// Tests if a generic Matrix3 seems to be a proper orthonormal rotation
  /// matrix to within the threshold specified by `tolerance`.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference of `R * Rᵀ`
  /// and the identity matrix I (i.e., checks if `‖R ⋅ Rᵀ - I‖∞ <= tolerance`).
  /// @returns `true` if R is a valid rotation matrix.
  static bool IsValid(const Matrix3<T>& R, double tolerance) {
    return IsOrthonormal(R, tolerance) && R.determinant() > 0;
  }

  /// Tests if a generic Matrix3 is a proper orthonormal rotation matrix to
  /// within the threshold of get_internal_tolerance_for_orthonormality().
  /// @param[in] R an allegedly valid rotation matrix.
  /// @returns `true` if R is a valid rotation matrix.
  static bool IsValid(const Matrix3<T>& R) {
    return IsValid(R, get_internal_tolerance_for_orthonormality());
  }

  /// Tests if `this` rotation matrix R is a proper orthonormal rotation matrix
  /// to within the threshold of get_internal_tolerance_for_orthonormality().
  /// @returns `true` if `this` is a valid rotation matrix.
  bool IsValid() const { return IsValid(matrix()); }

  /// Returns `true` if `this` is exactly equal to the identity matrix.
  bool IsExactlyIdentity() const { return matrix() == Matrix3<T>::Identity(); }

  /// Returns true if `this` is equal to the identity matrix to within the
  /// threshold of get_internal_tolerance_for_orthonormality().
  bool IsIdentityToInternalTolerance() const {
    return IsNearlyEqualTo(matrix(), Matrix3<T>::Identity(),
                           get_internal_tolerance_for_orthonormality());
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if `‖this - other‖∞ <= tolerance`.
  bool IsNearlyEqualTo(const RotationMatrix<T>& other, double tolerance) const {
    return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are exactly the same.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @returns true if each element of `this` is exactly equal to the
  /// corresponding element in `other`.
  bool IsExactlyEqualTo(const RotationMatrix<T>& other) const {
    return matrix() == other.matrix();
  }

  /// Computes the infinity norm of `this` - `other` (i.e., the maximum absolute
  /// value of the difference between the elements of `this` and `other`).
  /// @param[in] other %RotationMatrix to subtract from `this`.
  /// @returns `‖this - other‖∞`
  T GetMaximumAbsoluteDifference(const RotationMatrix<T>& other) const {
    return GetMaximumAbsoluteDifference(matrix(), other.matrix());
  }

  /// Given an approximate rotation matrix M, finds the %RotationMatrix R
  /// closest to M.  Closeness is measured with a matrix-2 norm (or equivalently
  /// with a Frobenius norm).  Hence, this method creates a %RotationMatrix R
  /// from a 3x3 matrix M by minimizing `‖R - M‖₂` (the matrix-2 norm of (R-M))
  /// subject to `R * Rᵀ = I`, where I is the 3x3 identity matrix.  For this
  /// problem, closeness can also be measured by forming the orthonormal matrix
  /// R whose elements minimize the double-summation `∑ᵢ ∑ⱼ (R(i,j) - M(i,j))²`
  /// where `i = 1:3, j = 1:3`, subject to `R * Rᵀ = I`.  The square-root of
  /// this double-summation is called the Frobenius norm.
  /// @param[in] M a 3x3 matrix.
  /// @param[out] quality_factor.  The quality of M as a rotation matrix.
  /// `quality_factor` = 1 is perfect (M = R). `quality_factor` = 1.25 means
  /// that when M multiplies a unit vector (magnitude 1), a vector of magnitude
  /// as large as 1.25 may result.  `quality_factor` = 0.8 means that when M
  /// multiplies a unit vector, a vector of magnitude as small as 0.8 may
  /// result.  `quality_factor` = 0 means M is singular, so at least one of the
  /// bases related by matrix M does not span 3D space (when M multiples a unit
  /// vector, a vector of magnitude as small as 0 may result).
  /// @returns proper orthonormal matrix R that is closest to M.
  /// @throws std::logic_error if R fails IsValid(R).
  /// @note William Kahan (UC Berkeley) and Hongkai Dai (Toyota Research
  /// Institute) proved that for this problem, the same R that minimizes the
  /// Frobenius norm also minimizes the matrix-2 norm (a.k.a an induced-2 norm),
  /// which is defined [Dahleh, Section 4.2] as the column matrix u which
  /// maximizes `‖(R - M) u‖ / ‖u‖`, where `u ≠ 0`.  Since the matrix-2 norm of
  /// any matrix A is equal to the maximum singular value of A, minimizing the
  /// matrix-2 norm of (R - M) is equivalent to minimizing the maximum singular
  /// value of (R - M).
  ///
  /// - [Dahleh] "Lectures on Dynamic Systems and Controls: Electrical
  /// Engineering and Computer Science, Massachusetts Institute of Technology"
  /// https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-241j-dynamic-systems-and-control-spring-2011/readings/MIT6_241JS11_chap04.pdf
  // @internal This function is not generated for symbolic Expression.
  // @internal This function's name is referenced in Doxygen documentation.
  template <typename S = T>
  static typename std::enable_if<is_numeric<S>::value, RotationMatrix<S>>::type
  ProjectToRotationMatrix(const Matrix3<S>& M, T* quality_factor = NULL) {
    const Matrix3<S> M_orthonormalized =
        ProjectMatrix3ToOrthonormalMatrix3(M, quality_factor);
    ThrowIfNotValid(M_orthonormalized);
    return RotationMatrix<S>(M_orthonormalized, true);
  }

  /// Returns an internal tolerance that checks rotation matrix orthonormality.
  /// @returns internal tolerance (small multiplier of double-precision epsilon)
  /// used to check whether or not a rotation matrix is orthonormal.
  /// @note The tolerance is chosen by developers to ensure a reasonably
  /// valid (orthonormal) rotation matrix.
  /// @note To orthonormalize a 3x3 matrix, use ProjectToRotationMatrix().
  static double get_internal_tolerance_for_orthonormality() {
    return kInternalToleranceForOrthonormality_;
  }

  /// Returns a quaternion q that represents `this` %RotationMatrix.  Since the
  /// quaternion `q` and `-q` represent the same %RotationMatrix, the quaternion
  /// returned by this method chooses the quaternion with q(0) >= 0.
  // @internal This implementation is adapted from simbody at
  // https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
  Eigen::Quaternion<T> ToQuaternion() const { return ToQuaternion(R_AB_); }

  /// Returns a unit quaternion q associated with the 3x3 matrix M.  Since the
  /// quaternion `q` and `-q` represent the same %RotationMatrix, the quaternion
  /// returned by this method chooses the quaternion with q(0) >= 0.
  /// @param[in] M 3x3 matrix to be made into a quaternion.
  /// @returns a unit quaternion q.
  /// @throws std::logic_error in debug builds if the quaternion `q`
  /// returned by this method cannot construct a valid %RotationMatrix.
  /// For example, if `M` contains NaNs, `q` will not be a valid quaternion.
  // @internal This implementation is adapted from simbody at
  // https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
  static Eigen::Quaternion<T> ToQuaternion(
      const Eigen::Ref<const Matrix3<T>>& M) {
    T w, x, y, z;  // Elements of the quaternion, w relates to cos(theta/2).

    const T trace = M.trace();
    if (trace >= M(0, 0) && trace >= M(1, 1) && trace >= M(2, 2)) {
      // This branch occurs if the trace is larger than any diagonal element.
      w = T(1) + trace;
      x = M(2, 1) - M(1, 2);
      y = M(0, 2) - M(2, 0);
      z = M(1, 0) - M(0, 1);
    } else if (M(0, 0) >= M(1, 1) && M(0, 0) >= M(2, 2)) {
      // This branch occurs if M(0,0) is largest among the diagonal elements.
      w = M(2, 1) - M(1, 2);
      x = T(1) - (trace - 2 * M(0, 0));
      y = M(0, 1) + M(1, 0);
      z = M(0, 2) + M(2, 0);
    } else if (M(1, 1) >= M(2, 2)) {
      // This branch occurs if M(1,1) is largest among the diagonal elements.
      w = M(0, 2) - M(2, 0);
      x = M(0, 1) + M(1, 0);
      y = T(1) - (trace - 2 * M(1, 1));
      z = M(1, 2) + M(2, 1);
    } else {
      // This branch occurs if M(2,2) is largest among the diagonal elements.
      w = M(1, 0) - M(0, 1);
      x = M(0, 2) + M(2, 0);
      y = M(1, 2) + M(2, 1);
      z = T(1) - (trace - 2 * M(2, 2));
    }

    // Create a quantity q (which is not yet a quaternion).
    // Note: Eigen's Quaternion constructor does not normalize.
    Eigen::Quaternion<T> q(w, x, y, z);

    // Since the quaternions q and -q correspond to the same rotation matrix,
    // choose a "canonical" quaternion with q(0) > 0.
    const T canonical_factor = (w < 0) ? T(-1) : T(1);

    // The quantity q calculated thus far in this algorithm is not a quaternion
    // with magnitude 1.  It differs from a quaternion in that all elements of q
    // are scaled by the same factor (the value of this factor depends on which
    // branch of the if/else-statements was used). To return a valid quaternion,
    // q must be normalized so q(0)^2 + q(1)^2 + q(2)^2 + q(3)^2 = 1.
    const T scale = canonical_factor / q.norm();
    q.coeffs() *= scale;

    DRAKE_ASSERT_VOID(ThrowIfNotValid(QuaternionToRotationMatrix(q, T(2))));
    return q;
  }

  /// Utility method to return the Vector4 associated with ToQuaterion().
  /// @see ToQuaternion().
  Vector4<T> ToQuaternionAsVector4() const {
    return ToQuaternionAsVector4(R_AB_);
  }

  /// Utility method to return the Vector4 associated with ToQuaterion(M).
  /// @param[in] M 3x3 matrix to be made into a quaternion.
  /// @see ToQuaternion().
  static Vector4<T> ToQuaternionAsVector4(const Matrix3<T>& M)  {
    const Eigen::Quaternion<T> q = ToQuaternion(M);
    return Vector4<T>(q.w(), q.x(), q.y(), q.z());
  }

  /// Returns an AngleAxis `theta_lambda` containing an angle `theta` and unit
  /// vector (axis direction) `lambda` that represents `this` %RotationMatrix.
  /// @note The orientation and %RotationMatrix associated with `theta * lambda`
  /// is identical to that of `(-theta) * (-lambda)`.  The AngleAxis returned by
  /// this method chooses to have `0 <= theta <= pi`.
  /// @returns an AngleAxis with `0 <= theta <= pi` and a unit vector `lambda`.
  Eigen::AngleAxis<T> ToAngleAxis() const {
    const Eigen::AngleAxis<T> theta_lambda(this->matrix());
    return theta_lambda;
  }

 private:
  // Make RotationMatrix<U> templatized on any typename U be a friend of a
  // %RotationMatrix templatized on any other typename T.
  // This is needed for the method RotationMatrix<T>::cast<U>() to be able to
  // use the necessary private constructor.
  template <typename U>
  friend class RotationMatrix;

  // Declares the allowable tolerance (small multiplier of double-precision
  // epsilon) used to check whether or not a rotation matrix is orthonormal.
  static constexpr double kInternalToleranceForOrthonormality_{
      128 * std::numeric_limits<double>::epsilon() };

  // Constructs a %RotationMatrix from a Matrix3.  No check is performed to test
  // whether or not the parameter R is a valid rotation matrix.
  // @param[in] R an allegedly valid rotation matrix.
  // @note The second parameter is just a dummy to distinguish this constructor
  // from one of the public constructors.
  RotationMatrix(const Matrix3<T>& R, bool) : R_AB_(R) {}

  // Sets `this` %RotationMatrix from a Matrix3.  No check is performed to
  // test whether or not the parameter R is a valid rotation matrix.
  // @param[in] R an allegedly valid rotation matrix.
  void SetUnchecked(const Matrix3<T>& R) { R_AB_ = R; }

  // Computes the infinity norm of R - `other` (i.e., the maximum absolute
  // value of the difference between the elements of R and `other`).
  // @param[in] R matrix from which `other` is subtracted.
  // @param[in] other matrix to subtract from R.
  // @returns `‖R - other‖∞`
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
  // @returns `true` if `‖R - `other`‖∞ <= tolerance`.
  static bool IsNearlyEqualTo(const Matrix3<T>& R, const Matrix3<T>& other,
                              double tolerance) {
    const T R_max_difference = GetMaximumAbsoluteDifference(R, other);
    return R_max_difference <= tolerance;
  }

  // Throws an exception if R is not a valid %RotationMatrix.
  // @param[in] R an allegedly valid rotation matrix.
  static void ThrowIfNotValid(const Matrix3<T>& R);

  // Given an approximate rotation matrix M, finds the orthonormal matrix R
  // closest to M.  Closeness is measured with a matrix-2 norm (or equivalently
  // with a Frobenius norm).  Hence, this method creates an orthonormal matrix R
  // from a 3x3 matrix M by minimizing `‖R - M‖₂` (the matrix-2 norm of (R-M))
  // subject to `R * Rᵀ = I`, where I is the 3x3 identity matrix.  For this
  // problem, closeness can also be measured by forming the orthonormal matrix R
  // whose elements minimize the double-summation `∑ᵢ ∑ⱼ (R(i,j) - M(i,j))²`
  // where `i = 1:3, j = 1:3`, subject to `R * Rᵀ = I`.  The square-root of
  // this double-summation is called the Frobenius norm.
  // @param[in] M a 3x3 matrix.
  // @param[out] quality_factor.  The quality of M as a rotation matrix.
  // `quality_factor` = 1 is perfect (M = R). `quality_factor` = 1.25 means
  // that when M multiplies a unit vector (magnitude 1), a vector of magnitude
  // as large as 1.25 may result.  `quality_factor` = -1 means M relates two
  // perfectly orthonormal bases, but one is right-handed whereas the other is
  // left-handed (M is a "reflection").  `quality_factor` = -0.8 means M
  // relates a right-handed basis to a left-handed basis and when M multiplies
  // a unit vector, a vector of magnitude as small as 0.8 may result.
  // `quality_factor` = 0 means M is singular, so at least one of the bases
  // related by matrix M does not span 3D space (when M multiples a unit vector,
  // a vector of magnitude as small as 0 may result).
  // @return orthonormal matrix R that is closest to M (note det(R) may be -1).
  // @note The SVD part of this algorithm can be derived as a modification of
  // section 3.2 in http://haralick.org/conferences/pose_estimation.pdf.
  // @note William Kahan (UC Berkeley) and Hongkai Dai (Toyota Research
  // Institute) proved that for this problem, the same R that minimizes the
  // Frobenius norm also minimizes the matrix-2 norm (a.k.a an induced-2 norm),
  // which is defined [Dahleh, Section 4.2] as the column matrix u which
  // maximizes `‖(R - M) u‖ / ‖u‖`, where `u ≠ 0`.  Since the matrix-2 norm of
  // any matrix A is equal to the maximum singular value of A, minimizing the
  // matrix-2 norm of (R - M) is equivalent to minimizing the maximum singular
  // value of (R - M).
  //
  // - [Dahleh] "Lectures on Dynamic Systems and Controls: Electrical
  // Engineering and Computer Science, Massachusetts Institute of Technology"
  // https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-241j-dynamic-systems-and-control-spring-2011/readings/MIT6_241JS11_chap04.pdf
  template <typename Derived>
  static Matrix3<typename Derived::Scalar> ProjectMatrix3ToOrthonormalMatrix3(
      const Eigen::MatrixBase<Derived>& M, T* quality_factor) {
    DRAKE_DEMAND(M.rows() == 3 && M.cols() == 3);
    const auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (quality_factor != nullptr) {
      // Singular values are always non-negative and sorted in decreasing order.
      const auto singular_values = svd.singularValues();
      const T s_max = singular_values(0);  // maximum singular value.
      const T s_min = singular_values(2);  // minimum singular value.
      const T s_f = (s_max != 0.0 && s_min < 1.0/s_max) ? s_min : s_max;
      const T det = M.determinant();
      const double sign_det = (det > 0.0) ? 1 : ((det < 0.0) ? -1 : 0);
      *quality_factor = s_f * sign_det;
    }
    return svd.matrixU() * svd.matrixV().transpose();
  }

  // Constructs a 3x3 rotation matrix from a Quaternion.
  // @param[in] quaternion a quaternion which may or may not have unit length.
  // @param[in] two_over_norm_squared is supplied by the calling method and is
  // usually pre-computed as `2 / quaternion.squaredNorm()`.  If `quaternion`
  // has already been normalized [`quaternion.norm() = 1`] or there is a reason
  // (unlikely) that the calling method determines that normalization is
  // unwanted, the calling method should just past `two_over_norm_squared = 2`.
  // @internal The cost of Eigen's quaternion.toRotationMatrix() is 12 adds and
  // 12 multiplies.  This function also costs 12 adds and 12 multiplies, but
  // has a provision for an efficient algorithm for always calculating an
  // orthogonal rotation matrix (whereas Eigen's algorithm does not).
  static Matrix3<T> QuaternionToRotationMatrix(
      const Eigen::Quaternion<T>& quaternion, const T& two_over_norm_squared) {
    Matrix3<T> m;

    const T w = quaternion.w();
    const T x = quaternion.x();
    const T y = quaternion.y();
    const T z = quaternion.z();
    const T sx  = two_over_norm_squared * x;  // scaled x-value.
    const T sy  = two_over_norm_squared * y;  // scaled y-value.
    const T sz  = two_over_norm_squared * z;  // scaled z-value.
    const T swx = sx * w;
    const T swy = sy * w;
    const T swz = sz * w;
    const T sxx = sx * x;
    const T sxy = sy * x;
    const T sxz = sz * x;
    const T syy = sy * y;
    const T syz = sz * y;
    const T szz = sz * z;

    m.coeffRef(0, 0) = T(1) - syy - szz;
    m.coeffRef(0, 1) = sxy - swz;
    m.coeffRef(0, 2) = sxz + swy;
    m.coeffRef(1, 0) = sxy + swz;
    m.coeffRef(1, 1) = T(1) - sxx - szz;
    m.coeffRef(1, 2) = syz - swx;
    m.coeffRef(2, 0) = sxz - swy;
    m.coeffRef(2, 1) = syz + swx;
    m.coeffRef(2, 2) = T(1) - sxx - syy;

    return m;
  }

  // Stores the underlying rotation matrix relating two frames (e.g. A and B).
  // The default initialization is the identity matrix.
  Matrix3<T> R_AB_{Matrix3<T>::Identity()};
};

// TODO(mitiguy) Delete this code after:
// * All call sites removed, and
// * code has subsequently been marked deprecated in favor of
//   RotationMatrix(RollPitchYaw(rpy)). as per issue #8323.
template <typename Derived>
Matrix3<typename Derived::Scalar> rpy2rotmat(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  using Scalar = typename Derived::Scalar;
  const RollPitchYaw<Scalar> roll_pitch_yaw(rpy(0), rpy(1), rpy(2));
  const RotationMatrix<Scalar> R(roll_pitch_yaw);
  return R.matrix();
}

}  // namespace math
}  // namespace drake


// TODO(mitiguy): Remove the following #include statement when the included file
// has been deleted.  See the included file for file deletion conditions.
// Note: This #include intentionally appears at the end of rotation_matrix.h as
// a temporary stop-gap measure to support backward compatability.  It is
// understood that including this file here is not style-guide compliant.
// clang-format off
#define DRAKE_MATH_ROTATION_MATRIX_DEPRECATED_HEADER_IS_ENABLED
#include "drake/math/rotation_matrix_deprecated.h"
#undef DRAKE_MATH_ROTATION_MATRIX_DEPRECATED_HEADER_IS_ENABLED
// clang-format on
