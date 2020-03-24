#pragma once

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace math {

/// This class represents a 3x3 rotation matrix between two arbitrary frames
/// A and B and helps ensure users create valid rotation matrices.  This class
/// relates right-handed orthogonal unit vectors Ax, Ay, Az fixed in frame A
/// to right-handed orthogonal unit vectors Bx, By, Bz fixed in frame B.
/// The monogram notation for the rotation matrix relating A to B is `R_AB`.
/// An example that gives context to this rotation matrix is `v_A = R_AB * v_B`,
/// where `v_B` denotes an arbitrary vector v expressed in terms of Bx, By, Bz
/// and `v_A` denotes vector v expressed in terms of Ax, Ay, Az.
/// See @ref multibody_quantities for monogram notation for dynamics.
/// See @ref orientation_discussion "a discussion on rotation matrices".
///
/// @note This class does not store the frames associated with a rotation matrix
/// nor does it enforce strict proper usage of this class with vectors.
///
/// @note When assertions are enabled, several methods in this class
/// do a validity check and throw an exception (std::logic_error) if the
/// rotation matrix is invalid.  When assertions are disabled,
/// many of these validity checks are skipped (which helps improve speed).
/// In addition, these validity tests are only performed for scalar types for
/// which drake::scalar_predicate<T>::is_bool is `true`. For instance, validity
/// checks are not performed when T is symbolic::Expression.
///
/// @authors Paul Mitiguy (2018) Original author.
/// @authors Drake team (see https://drake.mit.edu/credits).
///
/// @tparam_default_scalar
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix)

  /// Constructs a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() : R_AB_(Matrix3<T>::Identity()) {}

  /// Constructs a %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws std::logic_error in debug builds if R fails IsValid(R).
  explicit RotationMatrix(const Matrix3<T>& R) { set(R); }

  /// Constructs a %RotationMatrix from an Eigen::Quaternion.
  /// @param[in] quaternion a non-zero, finite quaternion which may or may not
  /// have unit length [i.e., `quaternion.norm()` does not have to be 1].
  /// @throws std::logic_error in debug builds if the rotation matrix
  /// R that is built from `quaternion` fails IsValid(R).  For example, an
  /// exception is thrown if `quaternion` is zero or contains a NaN or infinity.
  /// @note This method has the effect of normalizing its `quaternion` argument,
  /// without the inefficiency of the square-root associated with normalization.
  explicit RotationMatrix(const Eigen::Quaternion<T>& quaternion) {
    // TODO(mitiguy) Although this method is fairly efficient, consider adding
    // an optional second argument if `quaternion` is known to be normalized
    // apriori or for some reason the calling site does not want `quaternion`
    // normalized.
    // Cost for various way to create a rotation matrix from a quaternion.
    // Eigen quaternion.toRotationMatrix() = 12 multiplies, 12 adds.
    // Drake  QuaternionToRotationMatrix() = 12 multiplies, 12 adds.
    // Extra cost for two_over_norm_squared =  4 multiplies,  3 adds, 1 divide.
    // Extra cost if normalized = 4 multiplies, 3 adds, 1 sqrt, 1 divide.
    const T two_over_norm_squared = T(2) / quaternion.squaredNorm();
    set(QuaternionToRotationMatrix(quaternion, two_over_norm_squared));
  }

  // @internal In general, the %RotationMatrix constructed by passing a non-unit
  // `lambda` to this method is different than the %RotationMatrix produced by
  // converting `lambda` to an un-normalized quaternion and calling the
  // %RotationMatrix constructor (above) with that un-normalized quaternion.
  /// Constructs a %RotationMatrix from an Eigen::AngleAxis.
  /// @param[in] theta_lambda an Eigen::AngleAxis whose associated axis (vector
  /// direction herein called `lambda`) is non-zero and finite, but which may or
  /// may not have unit length [i.e., `lambda.norm()` does not have to be 1].
  /// @throws std::logic_error in debug builds if the rotation matrix
  /// R that is built from `theta_lambda` fails IsValid(R).  For example, an
  /// exception is thrown if `lambda` is zero or contains a NaN or infinity.
  explicit RotationMatrix(const Eigen::AngleAxis<T>& theta_lambda) {
    // TODO(mitiguy) Consider adding an optional second argument if `lambda` is
    // known to be normalized apriori or calling site does not want
    // normalization.
    const Vector3<T>& lambda = theta_lambda.axis();
    const T norm = lambda.norm();
    const T& theta = theta_lambda.angle();
    set(Eigen::AngleAxis<T>(theta, lambda / norm).toRotationMatrix());
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
  /// @note In this discussion, A is the Space frame and D is the Body frame.
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
  /// @note This method constructs a RotationMatrix from a RollPitchYaw.
  /// Vice-versa, there are high-accuracy RollPitchYaw constructor/methods that
  /// form a RollPitchYaw from a rotation matrix.
  explicit RotationMatrix(const RollPitchYaw<T>& rpy) {
    // TODO(@mitiguy) Add publically viewable documentation on how Sherm and
    // Goldstein like to visualize/conceptualize rotation sequences.
    const T& r = rpy.roll_angle();
    const T& p = rpy.pitch_angle();
    const T& y = rpy.yaw_angle();
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
    SetFromOrthonormalRows(Vector3<T>(Rxx, Rxy, Rxz),
                           Vector3<T>(Ryx, Ryy, Ryz),
                           Vector3<T>(Rzx, Rzy, Rzz));
  }

  /// (Advanced) Makes the %RotationMatrix `R_AB` from right-handed orthogonal
  /// unit vectors `Bx`, `By`, `Bz` so the columns of `R_AB` are `[Bx, By, Bz]`.
  /// @param[in] Bx first unit vector in right-handed orthogonal set.
  /// @param[in] By second unit vector in right-handed orthogonal set.
  /// @param[in] Bz third unit vector in right-handed orthogonal set.
  /// @throws std::logic_error in debug builds if `R_AB` fails IsValid(R_AB).
  /// @note In release builds, the caller can subsequently test if `R_AB` is,
  /// in fact, a valid %RotationMatrix by calling `R_AB.IsValid()`.
  /// @note The rotation matrix `R_AB` relates two sets of right-handed
  /// orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz.
  /// The rows of `R_AB` are Ax, Ay, Az expressed in frame B (i.e.,`Ax_B`,
  /// `Ay_B`, `Az_B`).  The columns of `R_AB` are Bx, By, Bz expressed in
  /// frame A (i.e., `Bx_A`, `By_A`, `Bz_A`).
  static RotationMatrix<T> MakeFromOrthonormalColumns(
      const Vector3<T>& Bx, const Vector3<T>& By, const Vector3<T>& Bz) {
    RotationMatrix<T> R(DoNotInitializeMemberFields{});
    R.SetFromOrthonormalColumns(Bx, By, Bz);
    return R;
  }

  /// (Advanced) Makes the %RotationMatrix `R_AB` from right-handed orthogonal
  /// unit vectors `Ax`, `Ay`, `Az` so the rows of `R_AB` are `[Ax, Ay, Az]`.
  /// @param[in] Ax first unit vector in right-handed orthogonal set.
  /// @param[in] Ay second unit vector in right-handed orthogonal set.
  /// @param[in] Az third unit vector in right-handed orthogonal set.
  /// @throws std::logic_error in debug builds if `R_AB` fails IsValid(R_AB).
  /// @note In release builds, the caller can subsequently test if `R_AB` is,
  /// in fact, a valid %RotationMatrix by calling `R_AB.IsValid()`.
  /// @note The rotation matrix `R_AB` relates two sets of right-handed
  /// orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz.
  /// The rows of `R_AB` are Ax, Ay, Az expressed in frame B (i.e.,`Ax_B`,
  /// `Ay_B`, `Az_B`).  The columns of `R_AB` are Bx, By, Bz expressed in
  /// frame A (i.e., `Bx_A`, `By_A`, `Bz_A`).
  static RotationMatrix<T> MakeFromOrthonormalRows(
      const Vector3<T>& Ax, const Vector3<T>& Ay, const Vector3<T>& Az) {
    RotationMatrix<T> R(DoNotInitializeMemberFields{});
    R.SetFromOrthonormalRows(Ax, Ay, Az);
    return R;
  }

  /// Makes the %RotationMatrix `R_AB` associated with rotating a frame B
  /// relative to a frame A by an angle `theta` about unit vector `Ax = Bx`.
  /// @param[in] theta radian measure of rotation angle about Ax.
  /// @note Orientation is same as Eigen::AngleAxis<T>(theta, Vector3d::UnitX().
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
  /// @note Orientation is same as Eigen::AngleAxis<T>(theta, Vector3d::UnitY().
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
  /// @note Orientation is same as Eigen::AngleAxis<T>(theta, Vector3d::UnitZ().
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
    // TODO(Mitiguy) Make the RotationMatrix::cast() method more robust.  It is
    // currently limited by Eigen's cast() for the matrix underlying this class.
    // Consider the following logic to improve casts (and address issue #11785).
    // 1. If relevant, use Eigen's underlying cast method.
    // 2. Strip derivative data when casting from `<AutoDiffXd>` to `<double>`.
    // 3. Call ExtractDoubleOrThrow() when casting from `<symbolic::Expression>`
    //    to `<double>`.
    // 4. The current RotationMatrix::cast() method incurs overhead due to its
    //    underlying call to a RotationMatrix constructor. Perhaps create
    //    specialized code to return a reference if casting to the same type,
    //    e.g., casting from `<double>` to `<double>' should be inexpensive.
    const Matrix3<U> m = R_AB_.template cast<U>();
    return RotationMatrix<U>(m, true);
  }

  /// Sets `this` %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws std::logic_error in debug builds if R fails IsValid(R).
  void set(const Matrix3<T>& R) {
    DRAKE_ASSERT_VOID(ThrowIfNotValid(R));
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

  /// Returns `R_BA = R_AB⁻¹`, the transpose of this %RotationMatrix.
  /// @note For a valid rotation matrix `R_BA = R_AB⁻¹ = R_ABᵀ`.
  // @internal This method's name was chosen to mimic Eigen's transpose().
  RotationMatrix<T> transpose() const {
    return RotationMatrix<T>(R_AB_.transpose());
  }

  /// Returns the Matrix3 underlying a %RotationMatrix.
  /// @see col(), row()
  const Matrix3<T>& matrix() const { return R_AB_; }

  /// Returns `this` rotation matrix's iᵗʰ row (i = 0, 1, 2).
  /// For `this` rotation matrix R_AB (which relates right-handed
  /// sets of orthogonal unit vectors Ax, Ay, Az to Bx, By, Bz),
  /// - row(0) returns Ax_B (Ax expressed in terms of Bx, By, Bz).
  /// - row(1) returns Ay_B (Ay expressed in terms of Bx, By, Bz).
  /// - row(2) returns Az_B (Az expressed in terms of Bx, By, Bz).
  /// @param[in] index requested row index (0 <= index <= 2).
  /// @see col(), matrix()
  /// @throws In debug builds, asserts (0 <= index <= 2).
  /// @note For efficiency and consistency with Eigen, this method returns
  /// the same quantity returned by Eigen's row() operator.
  /// The returned quantity can be assigned in various ways, e.g., as
  /// `const auto& Az_B = row(2);` or `RowVector3<T> Az_B = row(2);`
  const Eigen::Block<const Matrix3<T>, 1, 3, false> row(int index) const {
    // The returned value from this method mimics Eigen's row() method which was
    // found in  Eigen/src/plugins/BlockMethods.h.  The Eigen Matrix3 R_AB_ that
    // underlies this class is a column major matrix.  To return a row,
    // InnerPanel = false is passed as the last template parameter above.
    DRAKE_ASSERT(0 <= index && index <= 2);
    return R_AB_.row(index);
  }

  /// Returns `this` rotation matrix's iᵗʰ column (i = 0, 1, 2).
  /// For `this` rotation matrix R_AB (which relates right-handed
  /// sets of orthogonal unit vectors Ax, Ay, Az to Bx, By, Bz),
  /// - col(0) returns Bx_A (Bx expressed in terms of Ax, Ay, Az).
  /// - col(1) returns By_A (By expressed in terms of Ax, Ay, Az).
  /// - col(2) returns Bz_A (Bz expressed in terms of Ax, Ay, Az).
  /// @param[in] index requested column index (0 <= index <= 2).
  /// @see row(), matrix()
  /// @throws In debug builds, asserts (0 <= index <= 2).
  /// @note For efficiency and consistency with Eigen, this method returns
  /// the same quantity returned by Eigen's col() operator.
  /// The returned quantity can be assigned in various ways, e.g., as
  /// `const auto& Bz_A = col(2);` or `Vector3<T> Bz_A = col(2);`
  const Eigen::Block<const Matrix3<T>, 3, 1, true> col(int index) const {
    // The returned value from this method mimics Eigen's col() method which was
    // found in  Eigen/src/plugins/BlockMethods.h.  The Eigen Matrix3 R_AB_ that
    // underlies this class is a column major matrix.  To return a column,
    // InnerPanel = true is passed as the last template parameter above.
    DRAKE_ASSERT(0 <= index && index <= 2);
    return R_AB_.col(index);
  }

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

  /// Calculates `this` rotation matrix `R_AB` multiplied by an arbitrary
  /// Vector3 expressed in the B frame.
  /// @param[in] v_B 3x1 vector that post-multiplies `this`.
  /// @returns 3x1 vector `v_A = R_AB * v_B`.
  Vector3<T> operator*(const Vector3<T>& v_B) const {
    return Vector3<T>(matrix() * v_B);
  }

  /// Multiplies `this` %RotationMatrix `R_AB` by the n vectors `v1`, ... `vn`,
  /// where each vector has 3 elements and is expressed in frame B.
  /// @param[in] v_B `3 x n` matrix whose n columns are regarded as arbitrary
  /// vectors `v1`, ... `vn` expressed in frame B.
  /// @retval v_A `3 x n` matrix whose n columns are vectors `v1`, ... `vn`
  /// expressed in frame A.
  /// @code{.cc}
  /// const RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  /// const RotationMatrix<double> R_AB(rpy);
  /// Eigen::Matrix<double, 3, 2> v_B;
  /// v_B.col(0) = Vector3d(4, 5, 6);
  /// v_B.col(1) = Vector3d(9, 8, 7);
  /// const Eigen::Matrix<double, 3, 2> v_A = R_AB * v_B;
  /// @endcode
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime>
  operator*(const Eigen::MatrixBase<Derived>& v_B) const {
    if (v_B.rows() != 3) {
      throw std::logic_error(
          "Error: Inner dimension for matrix multiplication is not 3.");
    }
    // Express vectors in terms of frame A as v_A = R_AB * v_B.
    return matrix() * v_B;
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
  static boolean<T> IsOrthonormal(const Matrix3<T>& R, double tolerance) {
    return GetMeasureOfOrthonormality(R) <= tolerance;
  }

  /// Tests if a generic Matrix3 seems to be a proper orthonormal rotation
  /// matrix to within the threshold specified by `tolerance`.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @param[in] tolerance maximum allowable absolute difference of `R * Rᵀ`
  /// and the identity matrix I (i.e., checks if `‖R ⋅ Rᵀ - I‖∞ <= tolerance`).
  /// @returns `true` if R is a valid rotation matrix.
  static boolean<T> IsValid(const Matrix3<T>& R, double tolerance) {
    return IsOrthonormal(R, tolerance) && R.determinant() > 0;
  }

  /// Tests if a generic Matrix3 is a proper orthonormal rotation matrix to
  /// within the threshold of get_internal_tolerance_for_orthonormality().
  /// @param[in] R an allegedly valid rotation matrix.
  /// @returns `true` if R is a valid rotation matrix.
  static boolean<T> IsValid(const Matrix3<T>& R) {
    return IsValid(R, get_internal_tolerance_for_orthonormality());
  }

  /// Tests if `this` rotation matrix R is a proper orthonormal rotation matrix
  /// to within the threshold of get_internal_tolerance_for_orthonormality().
  /// @returns `true` if `this` is a valid rotation matrix.
  boolean<T> IsValid() const { return IsValid(matrix()); }

  /// Returns `true` if `this` is exactly equal to the identity matrix.
  boolean<T> IsExactlyIdentity() const {
    return matrix() == Matrix3<T>::Identity();
  }

  /// Returns true if `this` is equal to the identity matrix to within the
  /// threshold of get_internal_tolerance_for_orthonormality().
  boolean<T> IsIdentityToInternalTolerance() const {
    return IsNearlyEqualTo(matrix(), Matrix3<T>::Identity(),
                           get_internal_tolerance_for_orthonormality());
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if `‖this - other‖∞ <= tolerance`.
  boolean<T> IsNearlyEqualTo(const RotationMatrix<T>& other,
                             double tolerance) const {
    return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are exactly the same.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @returns true if each element of `this` is exactly equal to the
  /// corresponding element in `other`.
  boolean<T> IsExactlyEqualTo(const RotationMatrix<T>& other) const {
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
  static RotationMatrix<T>
  ProjectToRotationMatrix(const Matrix3<T>& M, T* quality_factor = nullptr) {
    const Matrix3<T> M_orthonormalized =
        ProjectMatrix3ToOrthonormalMatrix3(M, quality_factor);
    ThrowIfNotValid(M_orthonormalized);
    return RotationMatrix<T>(M_orthonormalized, true);
  }

  /// Returns an internal tolerance that checks rotation matrix orthonormality.
  /// @returns internal tolerance (small multiplier of double-precision epsilon)
  /// used to check whether or not a rotation matrix is orthonormal.
  /// @note The tolerance is chosen by developers to ensure a reasonably
  /// valid (orthonormal) rotation matrix.
  /// @note To orthonormalize a 3x3 matrix, use ProjectToRotationMatrix().
  static double get_internal_tolerance_for_orthonormality() {
    return kInternalToleranceForOrthonormality;
  }

  /// Returns a quaternion q that represents `this` %RotationMatrix.  Since the
  /// quaternion `q` and `-q` represent the same %RotationMatrix, this method
  /// chooses to return a canonical quaternion, i.e., with q(0) >= 0.
  /// @note There is a constructor in the RollPitchYaw class that converts
  /// a rotation matrix to roll-pitch-yaw angles.
  Eigen::Quaternion<T> ToQuaternion() const { return ToQuaternion(R_AB_); }

  /// Returns a unit quaternion q associated with the 3x3 matrix M.  Since the
  /// quaternion `q` and `-q` represent the same %RotationMatrix, this method
  /// chooses to return a canonical quaternion, i.e., with q(0) >= 0.
  /// @param[in] M 3x3 matrix to be made into a quaternion.
  /// @returns a unit quaternion q in canonical form, i.e., with q(0) >= 0.
  /// @throws std::logic_error in debug builds if the quaternion `q`
  /// returned by this method cannot construct a valid %RotationMatrix.
  /// For example, if `M` contains NaNs, `q` will not be a valid quaternion.
  static Eigen::Quaternion<T> ToQuaternion(
      const Eigen::Ref<const Matrix3<T>>& M) {
    Eigen::Quaternion<T> q = RotationMatrixToUnnormalizedQuaternion(M);

    // Since the quaternions q and -q correspond to the same rotation matrix,
    // choose to return a canonical quaternion, i.e., with q(0) >= 0.
    const T canonical_factor = if_then_else(q.w() < 0, T(-1), T(1));

    // The quantity q calculated thus far in this algorithm is not a quaternion
    // with magnitude 1.  It differs from a quaternion in that all elements of
    // q are scaled by the same factor. To return a valid quaternion, q must be
    // normalized so q(0)^2 + q(1)^2 + q(2)^2 + q(3)^2 = 1.
    const T scale = canonical_factor / q.norm();
    q.coeffs() *= scale;

    DRAKE_ASSERT_VOID(ThrowIfNotValid(QuaternionToRotationMatrix(q, T(2))));
    return q;
  }

  /// Utility method to return the Vector4 associated with ToQuaternion().
  /// @see ToQuaternion().
  Vector4<T> ToQuaternionAsVector4() const {
    return ToQuaternionAsVector4(R_AB_);
  }

  /// Utility method to return the Vector4 associated with ToQuaternion(M).
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
  static constexpr double kInternalToleranceForOrthonormality{
      128 * std::numeric_limits<double>::epsilon() };

  // Constructs a RotationMatrix without initializing the underlying 3x3 matrix.
  struct DoNotInitializeMemberFields{};
  explicit RotationMatrix(DoNotInitializeMemberFields) {}

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

  // Sets `this` %RotationMatrix `R_AB` from right-handed orthogonal unit
  // vectors `Bx`, `By`, `Bz` so that the columns of `this` are `[Bx, By, Bz]`.
  // @param[in] Bx first unit vector in right-handed orthogonal basis.
  // @param[in] By second unit vector in right-handed orthogonal basis.
  // @param[in] Bz third unit vector in right-handed orthogonal basis.
  // @throws std::logic_error in debug builds if `R_AB` fails IsValid(R_AB).
  // @note The rotation matrix `R_AB` relates two sets of right-handed
  // orthogonal unit vectors, namely `Ax`, `Ay`, `Az` and `Bx`, `By`, `Bz`.
  // The rows of `R_AB` are `Ax`, `Ay`, `Az` whereas the
  // columns of `R_AB` are `Bx`, `By`, `Bz`.
  void SetFromOrthonormalColumns(const Vector3<T>& Bx,
                                 const Vector3<T>& By,
                                 const Vector3<T>& Bz) {
    R_AB_.col(0) = Bx;
    R_AB_.col(1) = By;
    R_AB_.col(2) = Bz;
    DRAKE_ASSERT_VOID(ThrowIfNotValid(R_AB_));
  }

  // Sets `this` %RotationMatrix `R_AB` from right-handed orthogonal unit
  // vectors `Ax`, `Ay`, `Az` so that the rows of `this` are `[Ax, Ay, Az]`.
  // @param[in] Ax first unit vector in right-handed orthogonal basis.
  // @param[in] Ay second unit vector in right-handed orthogonal basis.
  // @param[in] Az third unit vector in right-handed orthogonal basis.
  // @throws std::logic_error in debug builds if `R_AB` fails R_AB.IsValid().
  // @see SetFromOrthonormalColumns() for additional notes.
  void SetFromOrthonormalRows(const Vector3<T>& Ax,
                              const Vector3<T>& Ay,
                              const Vector3<T>& Az) {
    R_AB_.row(0) = Ax;
    R_AB_.row(1) = Ay;
    R_AB_.row(2) = Az;
    DRAKE_ASSERT_VOID(ThrowIfNotValid(R_AB_));
  }

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
  static boolean<T> IsNearlyEqualTo(const Matrix3<T>& R,
                                    const Matrix3<T>& other,
                                    double tolerance) {
    const T R_max_difference = GetMaximumAbsoluteDifference(R, other);
    return R_max_difference <= tolerance;
  }

  // Throws an exception if R is not a valid %RotationMatrix.
  // @param[in] R an allegedly valid rotation matrix.
  // @note If the underlying scalar type T is non-numeric (symbolic), no
  // validity check is made and no assertion is thrown.
  template <typename S = T>
  static typename std::enable_if_t<scalar_predicate<S>::is_bool>
  ThrowIfNotValid(const Matrix3<S>& R);

  template <typename S = T>
  static typename std::enable_if_t<!scalar_predicate<S>::is_bool>
  ThrowIfNotValid(const Matrix3<S>&) {}

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

  // This is a helper method for RotationMatrix::ToQuaternion that returns a
  // Quaternion that is neither sign-canonicalized nor magnitude-normalized.
  //
  // This method is used for scalar types where scalar_predicate<T>::is_bool is
  // true (e.g., T = `double` and T = `AutoDiffXd`).  For types where is_bool
  // is false (e.g., T = `symbolic::Expression`), the alternative specialization
  // below is used instead.  We have two implementations so that this method is
  // as fast and compact as possible when `T = double`.
  //
  // N.B. Keep the math in this method in sync with the other specialization,
  // immediately below.
  template <typename S = T>
  static std::enable_if_t<scalar_predicate<S>::is_bool, Eigen::Quaternion<S>>
  RotationMatrixToUnnormalizedQuaternion(
      const Eigen::Ref<const Matrix3<S>>& M) {
    // This implementation is adapted from simbody at
    // https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
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
    // Create a quantity q (which is not yet a unit quaternion).
    // Note: Eigen's Quaternion constructor does not normalize.
    return Eigen::Quaternion<T>(w, x, y, z);
  }

  // Refer to the same-named method above for comments and details about the
  // algorithm being used, the meaning of the branches, etc.  This method is
  // identical except that the if-elseif chain is replaced with a symbolic-
  // conditional formulation.
  //
  // N.B. Keep the math in this method in sync with the other specialization,
  // immediately above.
  template <typename S = T>
  static std::enable_if_t<!scalar_predicate<S>::is_bool, Eigen::Quaternion<S>>
  RotationMatrixToUnnormalizedQuaternion(
      const Eigen::Ref<const Matrix3<S>>& M) {
    const T M00 = M(0, 0); const T M01 = M(0, 1); const T M02 = M(0, 2);
    const T M10 = M(1, 0); const T M11 = M(1, 1); const T M12 = M(1, 2);
    const T M20 = M(2, 0); const T M21 = M(2, 1); const T M22 = M(2, 2);
    const T trace = M00 + M11 + M22;
    auto if_then_else_vec4 = [](
        const boolean<T>& f_cond,
        const Vector4<T>& e_then,
        const Vector4<T>& e_else) -> Vector4<T> {
      return Vector4<T>(
          if_then_else(f_cond, e_then[0], e_else[0]),
          if_then_else(f_cond, e_then[1], e_else[1]),
          if_then_else(f_cond, e_then[2], e_else[2]),
          if_then_else(f_cond, e_then[3], e_else[3]));
    };
    const Vector4<T> wxyz =
        if_then_else_vec4(trace >= M00 && trace >= M11 && trace >= M22, {
          T(1) + trace,
          M21 - M12,
          M02 - M20,
          M10 - M01,
        }, if_then_else_vec4(M00 >= M11 && M00 >= M22, {
          M21 - M12,
          T(1) - (trace - 2 * M00),
          M01 + M10,
          M02 + M20,
        }, if_then_else_vec4(M11 >= M22, {
          M02 - M20,
          M01 + M10,
          T(1) - (trace - 2 * M11),
          M12 + M21,
        }, /* else */ {
          M10 - M01,
          M02 + M20,
          M12 + M21,
          T(1) - (trace - 2 * M22),
        })));
    return Eigen::Quaternion<T>(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  }

  // Constructs a 3x3 rotation matrix from a Quaternion.
  // @param[in] quaternion a quaternion which may or may not have unit length.
  // @param[in] two_over_norm_squared is supplied by the calling method and is
  // usually pre-computed as `2 / quaternion.squaredNorm()`.  If `quaternion`
  // has already been normalized [`quaternion.norm() = 1`] or there is a reason
  // (unlikely) that the calling method determines that normalization is
  // unwanted, the calling method should just past `two_over_norm_squared = 2`.
  // @internal The cost of Eigen's quaternion.toRotationMatrix() is 12 adds and
  // 12 multiplies.  This method also costs 12 adds and 12 multiplies, but
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
  // For speed, `R_AB_` is uninitialized (public constructors set its value).
  Matrix3<T> R_AB_;
};

/// Abbreviation (alias/typedef) for a RotationMatrix double scalar type.
/// @relates RotationMatrix
using RotationMatrixd = RotationMatrix<double>;

/// Projects an approximate 3 x 3 rotation matrix M onto an orthonormal matrix R
/// so that R is a rotation matrix associated with a angle-axis rotation by an
/// angle θ about a vector direction `axis`, with `angle_lb <= θ <= angle_ub`.
/// @tparam Derived A 3 x 3 matrix
/// @param[in] M the matrix to be projected.
/// @param[in] axis vector direction associated with angle-axis rotation for R.
///            axis can be a non-unit vector, but cannot be the zero vector.
/// @param[in] angle_lb the lower bound of the rotation angle θ.
/// @param[in] angle_ub the upper bound of the rotation angle θ.
/// @return Rotation angle θ of the projected matrix, angle_lb <= θ <= angle_ub
/// @throws std::runtime_error if M is not a 3 x 3 matrix or if
///         axis is the zero vector or if angle_lb > angle_ub.
/// @note This method is useful for reconstructing a rotation matrix for a
/// revolute joint with joint limits.
/// @note This can be formulated as an optimization problem
/// <pre>
///   min_θ trace((R - M)ᵀ*(R - M))
///   subject to R = I + sinθ * A + (1 - cosθ) * A²   (1)
///              angle_lb <= θ <= angle_ub
/// </pre>
/// where A is the cross product matrix of a = axis / axis.norm() = [a₁, a₂, a₃]
/// <pre>
/// A = [ 0  -a₃  a₂]
///     [ a₃  0  -a₁]
///     [-a₂  a₁  0 ]
/// </pre>
/// Equation (1) is the Rodriguez Formula that computes the rotation matrix R
/// from the angle-axis rotation with angle θ and vector direction `axis`.
/// For details, see http://mathworld.wolfram.com/RodriguesRotationFormula.html
/// The objective function can be simplified as
/// <pre>
///    max_θ trace(Rᵀ * M + Mᵀ * R)
/// </pre>
/// By substituting the matrix `R` with the angle-axis representation, the
/// optimization problem is formulated as
/// <pre>
///    max_θ sinθ * trace(Aᵀ*M) - cosθ * trace(Mᵀ * A²)
///    subject to angle_lb <= θ <= angle_ub
/// </pre>
/// By introducing α = atan2(-trace(Mᵀ * A²), trace(Aᵀ*M)), we can compute the
/// optimal θ as
/// <pre>
///    θ = π/2 + 2kπ - α, if angle_lb <= π/2 + 2kπ - α <= angle_ub, k ∈ integers
/// else
///    θ = angle_lb, if sin(angle_lb + α) >= sin(angle_ub + α)
///    θ = angle_ub, if sin(angle_lb + α) <  sin(angle_ub + α)
/// </pre>
/// @see GlobalInverseKinematics for an usage of this function.
template <typename Derived>
double ProjectMatToRotMatWithAxis(const Eigen::MatrixBase<Derived>& M,
                                  const Eigen::Vector3d& axis,
                                  const double angle_lb,
                                  const double angle_ub) {
  using Scalar = typename Derived::Scalar;
  if (M.rows() != 3 || M.cols() != 3) {
    throw std::runtime_error("The input matrix should be of size 3 x 3.");
  }
  if (angle_ub < angle_lb) {
    throw std::runtime_error(
        "The angle upper bound should be no smaller than the angle lower "
            "bound.");
  }
  const double axis_norm = axis.norm();
  if (axis_norm == 0) {
    throw std::runtime_error("The axis argument cannot be the zero vector.");
  }
  const Vector3<Scalar> a = axis / axis_norm;
  Eigen::Matrix3d A;
  // clang-format off
  A <<    0,  -a(2),   a(1),
       a(2),      0,  -a(0),
      -a(1),   a(0),      0;
  // clang-format on
  const Scalar alpha =
      atan2(-(M.transpose() * A * A).trace(), (A.transpose() * M).trace());
  Scalar theta{};
  // The bounds on θ + α is [angle_lb + α, angle_ub + α].
  if (std::isinf(angle_lb) && std::isinf(angle_ub)) {
    theta = M_PI_2 - alpha;
  } else if (std::isinf(angle_ub)) {
    // First if the angle upper bound is inf, start from the angle_lb, and
    // find the angle θ, such that θ + α = 0.5π + 2kπ
    const int k = ceil((angle_lb + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else if (std::isinf(angle_lb)) {
    // If the angle lower bound is inf, start from the angle_ub, and find the
    // angle θ, such that θ + α = 0.5π + 2kπ
    const int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else {
    // Now neither angle_lb nor angle_ub is inf. Check if there exists an
    // integer k, such that 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
    const int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    const double max_sin_angle = M_PI_2 + 2 * k * M_PI;
    if (max_sin_angle >= angle_lb + alpha) {
      // 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
      theta = max_sin_angle - alpha;
    } else {
      // Now the maximal is at the boundary, either θ = angle_lb or angle_ub
      if (sin(angle_lb + alpha) >= sin(angle_ub + alpha)) {
        theta = angle_lb;
      } else {
        theta = angle_ub;
      }
    }
  }
  return theta;
}

// @internal Initially, this code was in rotation_matrix.cc.  After
// RotationMatrix was instantiated on symbolic expression, there was a linker
// error that arose, but only during release builds and when tests in
// rotation_matrix_test.cc used symbolic expressions.  I (Paul) spent a fair
// amount of time trying to understand this problem (with Sherm & Sean).
template <typename T>
template <typename S>
typename std::enable_if_t<scalar_predicate<S>::is_bool>
RotationMatrix<T>::ThrowIfNotValid(const Matrix3<S>& R) {
  if (!R.allFinite()) {
    throw std::logic_error(
        "Error: Rotation matrix contains an element that is infinity or "
            "NaN.");
  }
  // If the matrix is not-orthogonal, try to give a detailed message.
  // This is particularly important if matrix is very-near orthogonal.
  if (!IsOrthonormal(R, get_internal_tolerance_for_orthonormality())) {
    const T measure_of_orthonormality = GetMeasureOfOrthonormality(R);
    const double measure = ExtractDoubleOrThrow(measure_of_orthonormality);
    std::string message = fmt::format(
        "Error: Rotation matrix is not orthonormal.\n"
        "  Measure of orthonormality error: {:G}  (near-zero is good).\n"
        "  To calculate the proper orthonormal rotation matrix closest to"
        " the alleged rotation matrix, use the SVD (expensive) method"
        " RotationMatrix::ProjectToRotationMatrix(), or for a less expensive"
        " (but not necessarily closest) rotation matrix, use the constructor"
        " RotationMatrix<T>(ToQuaternion(your_Matrix3)).  Alternately, if"
        " using quaternions, ensure the quaternion is normalized.", measure);
    throw std::logic_error(message);
  }
  if (R.determinant() < 0) {
    throw std::logic_error("Error: Rotation matrix determinant is negative. "
                               "It is possible a basis is left-handed");
  }
}

}  // namespace math
}  // namespace drake
