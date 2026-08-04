#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/unused.h"
#include "drake/math/fast_pose_composition_functions.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/unit_vector.h"

namespace drake {
namespace math {

namespace internal {
// This is used to select a non-initializing constructor for use by
// RigidTransform.
struct DoNotInitializeMemberFields {};
}  // namespace internal

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
/// @note When assertions are enabled, several methods in this class perform a
/// validity check and throw std::exception if the rotation matrix is invalid.
/// When assertions are disabled, many of these validity checks are skipped
/// (which helps improve speed). These validity tests are only performed for
/// scalar types for which drake::scalar_predicate<T>::is_bool is `true`. For
/// instance, validity checks are not performed when T is symbolic::Expression.
///
/// @authors Paul Mitiguy (2018) Original author.
/// @authors Drake team (see https://drake.mit.edu/credits).
///
/// @tparam_default_scalar
template <typename T>
class RotationMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrix);

  /// Constructs a 3x3 identity %RotationMatrix -- which corresponds to
  /// aligning two frames (so that unit vectors Ax = Bx, Ay = By, Az = Bz).
  RotationMatrix() : R_AB_(Matrix3<T>::Identity()) {}

  /// Constructs a %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws std::exception in debug builds if R fails IsValid(R).
  explicit RotationMatrix(const Matrix3<T>& R) { set(R); }

  /// Constructs a %RotationMatrix from an Eigen::Quaternion.
  /// @param[in] quaternion a non-zero, finite quaternion which may or may not
  /// have unit length [i.e., `quaternion.norm()` does not have to be 1].
  /// @throws std::exception in debug builds if the rotation matrix
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
    const T two_over_norm_squared = 2.0 / quaternion.squaredNorm();
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
  /// @throws std::exception in debug builds if the rotation matrix
  /// R that is built from `theta_lambda` fails IsValid(R).  For example, an
  /// exception is thrown if `lambda` is zero or contains a NaN or infinity.
  explicit RotationMatrix(const Eigen::AngleAxis<T>& theta_lambda) {
    const T& theta = theta_lambda.angle();
    const Vector3<T> unit_lambda = theta_lambda.axis().normalized();
    const T x = unit_lambda.x();
    const T y = unit_lambda.y();
    const T z = unit_lambda.z();
    const T s = sin(theta);
    const T c = cos(theta);
    const T t = 1 - c;     // 1 - cos(θ)
    const T sx = s * x;    // sin(θ) x
    const T sy = s * y;    // sin(θ) y
    const T sz = s * z;    // sin(θ) z
    const T tx = t * x;    // (1 - cos(θ)) x
    const T ty = t * y;    // (1 - cos(θ)) y
    const T tz = t * z;    // (1 - cos(θ)) z
    const T txx = tx * x;  // (1 - cos(θ)) x²
    const T tyy = ty * y;  // (1 - cos(θ)) y²
    const T tzz = tz * z;  // (1 - cos(θ)) z²
    const T txy = tx * y;  // (1 - cos(θ)) x y
    const T txz = tx * z;  // (1 - cos(θ)) x z
    const T tyz = ty * z;  // (1 - cos(θ)) y z
    Matrix3<T> R;
    R(0, 0) = txx + c;   // (1 - cos(θ)) x² + cos(θ)
    R(1, 1) = tyy + c;   // (1 - cos(θ)) y² + cos(θ)
    R(2, 2) = tzz + c;   // (1 - cos(θ)) z² + cos(θ)
    R(0, 1) = txy - sz;  // (1 - cos(θ)) x y - sin(θ) z
    R(1, 0) = txy + sz;  // (1 - cos(θ)) x y + sin(θ) z
    R(0, 2) = txz + sy;  // (1 - cos(θ)) x z + sin(θ) y
    R(2, 0) = txz - sy;  // (1 - cos(θ)) x z - sin(θ) y
    R(1, 2) = tyz - sx;  // (1 - cos(θ)) y z - sin(θ) x
    R(2, 1) = tyz + sx;  // (1 - cos(θ)) y z + sin(θ) x
    set(R);
  }

  /// Constructs a %RotationMatrix from an %RollPitchYaw.  In other words,
  /// makes the %RotationMatrix for a Space-fixed (extrinsic) X-Y-Z rotation by
  /// "roll-pitch-yaw" angles `[r, p, y]`, which is equivalent to a Body-fixed
  /// (intrinsic) Z-Y-X rotation by "yaw-pitch-roll" angles `[y, p, r]`.
  /// @param[in] rpy a %RollPitchYaw which is a Space-fixed (extrinsic) X-Y-Z
  /// rotation with "roll-pitch-yaw" angles `[r, p, y]` or equivalently a Body-
  /// fixed (intrinsic) Z-Y-X rotation with "yaw-pitch-roll" angles `[y, p, r]`.
  /// @retval R_AD, rotation matrix relating frame A to frame D.
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
  explicit RotationMatrix(const RollPitchYaw<T>& rpy);

  /// (Advanced) Makes a %RotationMatrix from a Matrix3. No check is performed
  /// to test whether or not the parameter R is a valid rotation matrix.
  static RotationMatrix<T> MakeUnchecked(const Matrix3<T>& R) {
    RotationMatrix<T> result(internal::DoNotInitializeMemberFields{});
    result.R_AB_ = R;
    return result;
  }

  /// (Advanced) Makes the %RotationMatrix `R_AB` from right-handed orthogonal
  /// unit vectors `Bx`, `By`, `Bz` so the columns of `R_AB` are `[Bx, By, Bz]`.
  /// @param[in] Bx first unit vector in right-handed orthogonal set.
  /// @param[in] By second unit vector in right-handed orthogonal set.
  /// @param[in] Bz third unit vector in right-handed orthogonal set.
  /// @throws std::exception in debug builds if `R_AB` fails IsValid(R_AB).
  /// @note In release builds, the caller can subsequently test if `R_AB` is,
  /// in fact, a valid %RotationMatrix by calling `R_AB.IsValid()`.
  /// @note The rotation matrix `R_AB` relates two sets of right-handed
  /// orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz.
  /// The rows of `R_AB` are Ax, Ay, Az expressed in frame B (i.e.,`Ax_B`,
  /// `Ay_B`, `Az_B`).  The columns of `R_AB` are Bx, By, Bz expressed in
  /// frame A (i.e., `Bx_A`, `By_A`, `Bz_A`).
  static RotationMatrix<T> MakeFromOrthonormalColumns(const Vector3<T>& Bx,
                                                      const Vector3<T>& By,
                                                      const Vector3<T>& Bz) {
    RotationMatrix<T> R(internal::DoNotInitializeMemberFields{});
    R.SetFromOrthonormalColumns(Bx, By, Bz);
    return R;
  }

  /// (Advanced) Makes the %RotationMatrix `R_AB` from right-handed orthogonal
  /// unit vectors `Ax`, `Ay`, `Az` so the rows of `R_AB` are `[Ax, Ay, Az]`.
  /// @param[in] Ax first unit vector in right-handed orthogonal set.
  /// @param[in] Ay second unit vector in right-handed orthogonal set.
  /// @param[in] Az third unit vector in right-handed orthogonal set.
  /// @throws std::exception in debug builds if `R_AB` fails IsValid(R_AB).
  /// @note In release builds, the caller can subsequently test if `R_AB` is,
  /// in fact, a valid %RotationMatrix by calling `R_AB.IsValid()`.
  /// @note The rotation matrix `R_AB` relates two sets of right-handed
  /// orthogonal unit vectors, namely Ax, Ay, Az and Bx, By, Bz.
  /// The rows of `R_AB` are Ax, Ay, Az expressed in frame B (i.e.,`Ax_B`,
  /// `Ay_B`, `Az_B`).  The columns of `R_AB` are Bx, By, Bz expressed in
  /// frame A (i.e., `Bx_A`, `By_A`, `Bz_A`).
  static RotationMatrix<T> MakeFromOrthonormalRows(const Vector3<T>& Ax,
                                                   const Vector3<T>& Ay,
                                                   const Vector3<T>& Az) {
    RotationMatrix<T> R(internal::DoNotInitializeMemberFields{});
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
  static RotationMatrix<T> MakeXRotation(const T& theta);

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
  static RotationMatrix<T> MakeYRotation(const T& theta);

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
  static RotationMatrix<T> MakeZRotation(const T& theta);

  /// Creates a 3D right-handed orthonormal basis B from a given vector b_A,
  /// returned as a rotation matrix R_AB. It consists of orthogonal unit vectors
  /// u_A, v_A, w_A where u_A is the normalized b_A in the axis_index column of
  /// R_AB and v_A has one element which is zero.  If an element of b_A is zero,
  /// then one element of w_A is 1 and the other two elements are 0.
  /// @param[in] b_A vector expressed in frame A that when normalized as
  /// u_A = b_A.normalized() represents Bx, By, or Bz (depending on axis_index).
  /// @param[in] axis_index The index ∈ {0, 1, 2} of the unit vector associated
  ///  with u_A, 0 means u_A is Bx, 1 means u_A is By, and 2 means u_A is Bz.
  /// @pre axis_index is 0 or 1 or 2.
  /// @throws std::exception if b_A cannot be made into a unit vector because
  ///   b_A contains a NaN or infinity or |b_A| < 1.0E-10.
  /// @see MakeFromOneUnitVector() if b_A is known to already be unit length.
  /// @retval R_AB the rotation matrix with properties as described above.
  static RotationMatrix<T> MakeFromOneVector(const Vector3<T>& b_A,
                                             int axis_index) {
    const Vector3<T> u_A = math::internal::NormalizeOrThrow(b_A, __func__);
    return MakeFromOneUnitVector(u_A, axis_index);
  }

  /// (Advanced) Creates a right-handed orthonormal basis B from a given
  /// unit vector u_A, returned as a rotation matrix R_AB.
  /// @param[in] u_A unit vector which is expressed in frame A and is either
  ///  Bx or By or Bz (depending on the value of axis_index).
  /// @param[in] axis_index The index ∈ {0, 1, 2} of the unit vector associated
  ///  with u_A, 0 means u_A is Bx, 1 means u_A is By, and 2 means u_A is Bz.
  /// @pre axis_index is 0 or 1 or 2.
  /// @throws std::exception if u_A is not a unit vector, i.e., |u_A| is not
  /// within a tolerance of 4ε ≈ 8.88E-16 to 1.0.
  /// @note This method is designed for speed and does not normalize u_A to
  ///  ensure it is a unit vector. Alternatively, consider MakeFromOneVector().
  /// @retval R_AB the rotation matrix whose properties are described in
  /// MakeFromOneVector().
  static RotationMatrix<T> MakeFromOneUnitVector(const Vector3<T>& u_A,
                                                 int axis_index);

  /// Creates a 3D right-handed orthonormal basis B from a given unit vector
  /// u_A, returned as a rotation matrix R_AB. It consists of orthogonal unit
  /// vectors [Bx, By, Bz] where Bz is u_A.
  /// The angle-axis representation of the resulting rotation is the one with
  /// the minimum rotation angle that rotates A to B. When u_A is not parallel
  /// or antiparallel to [0, 0, 1], such rotation is unique.
  /// @param[in] u_A unit vector expressed in frame A that represents Bz.
  /// @throws std::exception if u_A is not a unit vector.
  /// @retval R_AB the rotation matrix with properties as described above.
  static RotationMatrix<T> MakeClosestRotationToIdentityFromUnitZ(
      const Vector3<T>& u_A) {
    internal::ThrowIfNotUnitVector(u_A, __func__);
    const Vector3<T>& Bz = u_A;
    const Vector3<T> Az = Vector3<T>(0, 0, 1);
    // The rotation axis of the Axis-Angle representation of the resulting
    // rotation.
    const Vector3<T> axis = Az.cross(Bz);
    const T axis_norm = axis.norm();
    const Vector3<T> normalized_axis =
        axis_norm < 1e-10 ? Vector3<T>(1, 0, 0) : axis / axis_norm;
    using std::atan2;
    const T angle = atan2(axis_norm, Az.dot(Bz));
    return RotationMatrix<T>(Eigen::AngleAxis<T>(angle, normalized_axis));
  }

  /// @name      (Internal use only) methods for axial rotations
  /// @anchor axial_rotation_def
  ///
  /// (Internal use only) Axial rotations are %RotationMatrix objects for which
  /// we know that the represented rotation is about one of the basis vectors
  /// (coordinate axes) x, y, or z. These are essentially 2d rotations and only
  /// four of the nine elements are "active", by which we mean that they change
  /// with the rotation angle θ. The other five elements are "inactive", meaning
  /// that they have known values and never change during a simulation. For best
  /// performance, the algorithms in this section are permitted to _assume_
  /// without looking that the inactive elements have their required values.
  /// However, those elements are still required to have the expected values,
  /// which are the values they would have in an identity rotation (that is, 1
  /// on the diagonal and 0 off-diagonal). (Even when T is symbolic::Expression,
  /// we insist that the inactive elements have the correct numerical values and
  /// don't require an Environment for evaluation.) This ensures that general
  /// purpose (non-specialized) code can still use the resulting rotation
  /// matrices.
  ///
  /// With _general_ rotation matrices (nine active elements), re-expressing a
  /// vector takes 15 floating point operations, composing two rotations
  /// takes 45, and updating requires writing all nine elements. The methods in
  /// this section take advantage of the axial structure to reduce the number
  /// of operations required. To do that, they are templatized by the axis
  /// number 0, 1, or 2 (+x, +y, or +z axis, resp.). The number of operations
  /// required is documented with each method.
  ///
  /// Notation: we denote axial rotations by preceding the usual "R" with a
  /// lower case "a" for axial, e.g. aR_BC is a rotation matrix R_BC but for
  /// which we have foreknowledge of its axial structure. In cases where we
  /// know the particular axis we'll replace the "a" with "x", "y", or "z".
  ///
  /// @warning We depend on the caller's promise that axial rotation matrix
  /// arguments are indeed axial; for performance reasons we do not verify that
  /// in Release builds although we may verify in Debug builds.
  ///
  /// @note There are no Python bindings for these methods since there is
  /// nothing to be gained by using them in Python. Use the general equivalent
  /// %RotationMatrix methods instead.
  ///@{

  /// (Internal use only) Creates an axial rotation aR_AB consisting of a
  /// rotation of `theta` radians about x, y, or z. Of the 9 entries in the
  /// rotation matrix, only 4 are active; the rest will be set to 0 or 1. This
  /// structure can be exploited for efficient updating and operating with this
  /// rotation matrix.
  /// @param[in] theta the rotation angle.
  /// @retval aR_BC the axial rotation (also known as R_BC(theta)).
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @see @ref axial_rotation_def "Axial rotations".
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  static RotationMatrix<T> MakeAxialRotation(const T& theta) {
    if constexpr (axis == 0) return MakeXRotation(theta);
    if constexpr (axis == 1) return MakeYRotation(theta);
    if constexpr (axis == 2) return MakeZRotation(theta);
  }

  /// (Internal use only) Given an axial rotation about a coordinate axis (x, y,
  /// or z), uses it to efficiently re-express a vector. This takes only 6
  /// floating point operations.
  /// @param[in] aR_BC An axial rotation about the indicated axis.
  /// @param[in] v_C A vector expressed in frame C to be re-expressed in
  ///   frame B.
  /// @retval v_B The input vector re-expressed in frame B.
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @pre aR_BC is an @ref axial_rotation_def "axial rotation matrix" about
  ///   the given `axis`.
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  static Vector3<T> ApplyAxialRotation(const RotationMatrix<T>& aR_BC,
                                       const Vector3<T>& v_C) {
    // See comments for MakeFromXRotation() etc. to see that this is correct.
    DRAKE_ASSERT_VOID(aR_BC.IsAxialRotationOrThrow<axis>());
    const Matrix3<T>& M = aR_BC.matrix();
    constexpr int x = axis, y = (axis + 1) % 3, z = (axis + 2) % 3;
    const T& c = M(y, y);  // cosine
    const T& s = M(z, y);  // sine
    Vector3<T> v_B;
    v_B(x) = v_C(x);  // No effect along the rotation axis.
    v_B(y) = c * v_C(y) - s * v_C(z);
    v_B(z) = c * v_C(z) + s * v_C(y);
    return v_B;
  }

  /// (Internal use only) Given a new rotation angle θ, updates the axial
  /// rotation aR_BC to represent the new rotation angle. We expect that aR_BC
  /// was already such an axial rotation about the given x, y, or z axis. Only
  /// the 4 active elements are modified; the other 5 remain unchanged.
  /// (However, execution time is likely to be dominated by the time to
  /// calculate sine and cosine.)
  /// @param[in] theta the new rotation angle in radians.
  /// @param[in,out] aR_BC the axial rotation matrix to be updated.
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @see the overloaded signature if you already have sin(θ) & cos(θ).
  /// @pre aR_BC is an @ref axial_rotation_def "axial rotation matrix" about
  ///   the given `axis`.
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  static void UpdateAxialRotation(const T& theta, RotationMatrix<T>* aR_BC) {
    using std::cos, std::sin;
    DRAKE_ASSERT(aR_BC != nullptr);
    UpdateAxialRotation<axis>(sin(theta), cos(theta), &*aR_BC);
  }

  /// (Internal use only) Given sin(θ) and cos(θ), where θ is a new rotation
  /// angle, updates the axial rotation aR_BC to represent the new rotation
  /// angle. We expect that aR_BC was already such a rotation (about the given
  /// x, y, or z axis). Only the 4 active elements are modified; the other 5
  /// remain unchanged.
  /// @param[in] sin_theta sin(θ), where θ is the new rotation angle.
  /// @param[in] cos_theta cos(θ), where θ is the new rotation angle.
  /// @param[in,out] aR_BC the axial rotation matrix to be updated.
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @see the overloaded signature if you just have the angle θ.
  /// @pre aR_BC is an @ref axial_rotation_def "axial rotation matrix" about
  ///   the given `axis`.
  /// @pre `sin_theta` and `cos_theta` are sine and cosine of the same angle.
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  static void UpdateAxialRotation(const T& sin_theta, const T& cos_theta,
                                  RotationMatrix<T>* aR_BC) {
    DRAKE_ASSERT(aR_BC != nullptr);
    DRAKE_ASSERT_VOID(aR_BC->IsAxialRotationOrThrow<axis>());
    DRAKE_ASSERT_VOID(SinCosConsistencyOrThrow(sin_theta, cos_theta));
    Matrix3<T>& M = aR_BC->mutable_matrix();
    constexpr int y = (axis + 1) % 3, z = (axis + 2) % 3;
    M(y, y) = M(z, z) = cos_theta;
    M(z, y) = sin_theta;
    M(y, z) = -sin_theta;  // only 1 flop
  }

  /// (Internal use only) With `this` a general rotation R_AB, and given an
  /// axial rotation aR_BC, efficiently forms R_AC = R_AB * aR_BC. This requires
  /// only 18 floating point operations.
  /// @param[in] aR_BC An axial rotation about the indicated axis.
  /// @param[out] R_AC The result, which will be a general rotation matrix.
  ///   Must not overlap with `this` in memory.
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @pre aR_BC is an @ref axial_rotation_def "axial rotation matrix" about
  ///   the given `axis`.
  /// @pre R_AC does not overlap with `this` in memory.
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  void PostMultiplyByAxialRotation(const RotationMatrix<T>& aR_BC,
                                   RotationMatrix<T>* R_AC) const {
    // If this is inlined it should be considerably faster than the
    // (non-inlined) SIMD case.
    DRAKE_ASSERT(R_AC != nullptr && R_AC != this);
    DRAKE_ASSERT_VOID(aR_BC.IsAxialRotationOrThrow<axis>());
    const Matrix3<T>& M_BC = aR_BC.matrix();
    constexpr int x = axis, y = (axis + 1) % 3, z = (axis + 2) % 3;
    const T& c = M_BC(y, y);  // cosine
    const T& s = M_BC(z, y);  // sine
    Matrix3<T>& M_AC = R_AC->mutable_matrix();
    M_AC.col(x) = R_AB_.col(x);  // No effect on the rotation axis.
    M_AC.col(y) = c * R_AB_.col(y) + s * R_AB_.col(z);
    M_AC.col(z) = c * R_AB_.col(z) - s * R_AB_.col(y);
  }

  /// (Internal use only) With `this` a general rotation R_BC, and given an
  /// axial rotation aR_AB, efficiently forms R_AC = aR_AB * R_BC. This requires
  /// only 18 floating point operations.
  /// @param[in] aR_AB An axial rotation about the indicated axis.
  /// @param[out] R_AC The result, which will be a general rotation matrix. Must
  ///   not overlap with `this` in memory.
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @pre aR_AB is an @ref axial_rotation_def "axial rotation matrix" about
  ///   the given `axis`.
  /// @pre R_AC does not overlap with `this` in memory.
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  void PreMultiplyByAxialRotation(const RotationMatrix<T>& aR_AB,
                                  RotationMatrix<T>* R_AC) const {
    // If this is inlined it should be considerably faster than the
    // (non-inlined) SIMD case.
    DRAKE_ASSERT(R_AC != nullptr && R_AC != this);
    DRAKE_ASSERT_VOID(aR_AB.IsAxialRotationOrThrow<axis>());
    const RotationMatrix<T>& R_BC = *this;
    const Matrix3<T>& M_AB = aR_AB.matrix();
    constexpr int x = axis, y = (axis + 1) % 3, z = (axis + 2) % 3;
    const T& c = M_AB(y, y);  // cosine
    const T& s = M_AB(z, y);  // sine
    Matrix3<T>& M_AC = R_AC->mutable_matrix();
    M_AC.row(x) = R_BC.row(x);  // No effect on the rotation axis.
    M_AC.row(y) = c * R_BC.row(y) - s * R_BC.row(z);
    M_AC.row(z) = c * R_BC.row(z) + s * R_BC.row(y);
  }

  /// (Internal use only) Throws an exception if `this` %RotationMatrix is not
  /// an axial rotation about the indicated axis.
  /// See @ref axial_rotation_def "axial rotation matrix" for the conditions
  /// that must be satisfied for a rotation matrix to be "axial". In addition,
  /// for numerical types T we check here that the active elements are
  /// reasonable: there should be two equal cos(θ) entries, ±sin(θ) entries, and
  /// sin²+cos²==1. (Numerical tests are done to a fairly loose tolerance of 16ε
  /// to avoid false negatives.)
  /// @tparam axis 0, 1, or 2 corresponding to +x, +y, or +z rotation axis.
  /// @note This is intended for Debug-mode checks that the other methods in
  ///   this section are being used properly.
  template <int axis>
#ifndef DRAKE_DOXYGEN_CXX
    requires(0 <= axis && axis <= 2)
#endif
  void IsAxialRotationOrThrow() const;
  ///@}

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
  RotationMatrix<U> cast() const
    requires is_default_scalar<U>
  {  // NOLINT(whitespace/braces)
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
    //    e.g., casting from `<double>` to `<double>` should be inexpensive.
    return RotationMatrix<U>::MakeUnchecked(R_AB_.template cast<U>());
  }

  /// Sets `this` %RotationMatrix from a Matrix3.
  /// @param[in] R an allegedly valid rotation matrix.
  /// @throws std::exception in debug builds if R fails IsValid(R).
  void set(const Matrix3<T>& R) {
    DRAKE_ASSERT_VOID(ThrowIfNotValid(R));
    R_AB_ = R;
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
  /// @throws In Debug builds, asserts (0 <= index <= 2).
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
  /// @throws In Debug builds, asserts (0 <= index <= 2).
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
    if constexpr (std::is_same_v<T, double>) {
      internal::ComposeRR(*this, other, this);
    } else {
      // The result of matrix multiplication is not checked with
      // ThrowIfNotValid() because the overhead would make this highly-used
      // function very expensive. However, both arguments to this function and
      // its result should be valid rotation matrices unless earlier validity
      // checks are by-passed, e.g., with RotationMatrix::MakeUnchecked().
      R_AB_ = matrix() * other.matrix();
    }
    return *this;
  }

  /// Calculates `this` rotation matrix `R_AB` multiplied by `other` rotation
  /// matrix `R_BC`, returning the composition `R_AB * R_BC`.
  /// @param[in] other %RotationMatrix that post-multiplies `this`.
  /// @returns rotation matrix that results from `this` multiplied by `other`.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> operator*(const RotationMatrix<T>& other) const {
    RotationMatrix<T> R_AC(internal::DoNotInitializeMemberFields{});
    if constexpr (std::is_same_v<T, double>) {
      internal::ComposeRR(*this, other, &R_AC);
    } else {
      R_AC.R_AB_ = matrix() * other.matrix();
    }
    return R_AC;
  }

  /// Calculates the product of `this` inverted and another %RotationMatrix.
  /// If you consider `this` to be the rotation matrix R_AB, and `other` to be
  /// R_AC, then this method returns R_BC = R_AB⁻¹ * R_AC. For T==double, this
  /// method can be _much_ faster than inverting first and then performing the
  /// composition because it can take advantage of the orthogonality of
  /// rotation matrices. On some platforms it can use SIMD instructions for
  /// further speedups.
  /// @param[in] other %RotationMatrix that post-multiplies `this` inverted.
  /// @retval R_BC where R_BC = this⁻¹ * other.
  /// @note It is possible (albeit improbable) to create an invalid rotation
  /// matrix by accumulating round-off error with a large number of multiplies.
  RotationMatrix<T> InvertAndCompose(const RotationMatrix<T>& other) const {
    const RotationMatrix<T>& R_AC = other;  // Nicer name.
    RotationMatrix<T> R_BC(internal::DoNotInitializeMemberFields{});
    if constexpr (std::is_same_v<T, double>) {
      internal::ComposeRinvR(*this, R_AC, &R_BC);
    } else {
      const RotationMatrix<T> R_BA = inverse();
      R_BC = R_BA * R_AC;
    }
    return R_BC;
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

  /// Returns how close the matrix R is to being a 3x3 orthonormal matrix by
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
  /// @see IsNearlyIdentity().
  boolean<T> IsExactlyIdentity() const {
    return matrix() == Matrix3<T>::Identity();
  }

  /// Returns true if `this` is within tolerance of the identity RigidTransform.
  /// @param[in] tolerance non-negative number that is generally the default
  /// value, namely RotationMatrix::get_internal_tolerance_for_orthonormality().
  /// @see IsExactlyIdentity().
  boolean<T> IsNearlyIdentity(
      double tolerance = get_internal_tolerance_for_orthonormality()) const {
    return IsNearlyEqualTo(matrix(), Matrix3<T>::Identity(), tolerance);
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are the same to within a specified `tolerance`.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @param[in] tolerance maximum allowable absolute difference between the
  /// matrix elements in `this` and `other`.
  /// @returns `true` if `‖this - other‖∞ <= tolerance`.
  /// @see IsExactlyEqualTo().
  boolean<T> IsNearlyEqualTo(const RotationMatrix<T>& other,
                             double tolerance) const {
    return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
  }

  /// Compares each element of `this` to the corresponding element of `other`
  /// to check if they are exactly the same.
  /// @param[in] other %RotationMatrix to compare to `this`.
  /// @returns true if each element of `this` is exactly equal to the
  /// corresponding element in `other`.
  /// @see IsNearlyEqualTo().
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
  /// @param[out] quality_factor The quality of M as a rotation matrix.
  /// `quality_factor` = 1 is perfect (M = R). `quality_factor` = 1.25 means
  /// that when M multiplies a unit vector (magnitude 1), a vector of magnitude
  /// as large as 1.25 may result.  `quality_factor` = 0.8 means that when M
  /// multiplies a unit vector, a vector of magnitude as small as 0.8 may
  /// result.  `quality_factor` = 0 means M is singular, so at least one of the
  /// bases related by matrix M does not span 3D space (when M multiples a unit
  /// vector, a vector of magnitude as small as 0 may result).
  /// @returns proper orthonormal matrix R that is closest to M.
  /// @throws std::exception if R fails IsValid(R).
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
  static RotationMatrix<T> ProjectToRotationMatrix(
      const Matrix3<T>& M, T* quality_factor = nullptr) {
    const Matrix3<T> M_orthonormalized =
        ProjectMatrix3ToOrthonormalMatrix3(M, quality_factor);
    ThrowIfNotValid(M_orthonormalized);
    return RotationMatrix<T>::MakeUnchecked(M_orthonormalized);
  }

  /// Returns an internal tolerance that checks rotation matrix orthonormality.
  /// @returns internal tolerance (small multiplier of double-precision epsilon)
  /// used to check whether or not a rotation matrix is orthonormal.
  /// @note The tolerance is chosen by developers to ensure a reasonably
  /// valid (orthonormal) rotation matrix.
  /// @note To orthonormalize a 3x3 matrix, use ProjectToRotationMatrix().
  static constexpr double get_internal_tolerance_for_orthonormality() {
    return kInternalToleranceForOrthonormality;
  }

  /// Returns a RollPitchYaw that represents `this` %RotationMatrix, with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= π`.
  /// @note This new high-accuracy algorithm avoids numerical round-off issues
  /// encountered by some algorithms when pitch is within 1E-6 of π/2 or -π/2.
  RollPitchYaw<T> ToRollPitchYaw() const { return RollPitchYaw(*this); }

  /// Returns a quaternion q that represents `this` %RotationMatrix.  Since the
  /// quaternion `q` and `-q` represent the same %RotationMatrix, this method
  /// chooses to return a canonical quaternion, i.e., with q(0) >= 0.
  Eigen::Quaternion<T> ToQuaternion() const { return ToQuaternion(R_AB_); }

  /// Returns a unit quaternion q associated with the 3x3 matrix M.  Since the
  /// quaternion `q` and `-q` represent the same %RotationMatrix, this method
  /// chooses to return a canonical quaternion, i.e., with q(0) >= 0.
  /// @param[in] M 3x3 matrix to be made into a quaternion.
  /// @returns a unit quaternion q in canonical form, i.e., with q(0) >= 0.
  /// @throws std::exception in debug builds if the quaternion `q`
  /// returned by this method cannot construct a valid %RotationMatrix.
  /// For example, if `M` contains NaNs, `q` will not be a valid quaternion.
  static Eigen::Quaternion<T> ToQuaternion(
      const Eigen::Ref<const Matrix3<T>>& M);

  /// Utility method to return the Vector4 associated with ToQuaternion().
  /// @see ToQuaternion().
  Vector4<T> ToQuaternionAsVector4() const {
    return ToQuaternionAsVector4(R_AB_);
  }

  /// Utility method to return the Vector4 associated with ToQuaternion(M).
  /// @param[in] M 3x3 matrix to be made into a quaternion.
  /// @see ToQuaternion().
  static Vector4<T> ToQuaternionAsVector4(const Matrix3<T>& M) {
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

  /// (Internal use only) Constructs a RotationMatrix without initializing the
  /// underlying 3x3 matrix. For use by RigidTransform and RotationMatrix only.
  explicit RotationMatrix(internal::DoNotInitializeMemberFields) {}

  /// Implements the @ref hash_append concept.
  /// @pre T implements the hash_append concept.
  template <class HashAlgorithm>
  // NOLINTNEXTLINE(runtime/references) Per hash_append convention.
  friend void hash_append(HashAlgorithm& hasher,
                          const RotationMatrix& R) noexcept {
    const T* begin = R.R_AB_.data();
    const T* end = R.R_AB_.data() + R.R_AB_.size();
    using drake::hash_append_range;
    hash_append_range(hasher, begin, end);
  }

 private:
  // Make RotationMatrix<U> templatized on any typename U be a friend of a
  // %RotationMatrix templatized on any other typename T.
  // This is needed for the method RotationMatrix<T>::cast<U>() to be able to
  // use the necessary private constructor.
  template <typename U>
  friend class RotationMatrix;

  // Returns the mutable Matrix3 underlying a RotationMatrix.
  Matrix3<T>& mutable_matrix() { return R_AB_; }

  // Declares the allowable tolerance (small multiplier of double-precision
  // epsilon) used to check whether or not a rotation matrix is orthonormal.
  static constexpr double kInternalToleranceForOrthonormality{
      128 * std::numeric_limits<double>::epsilon()};

  // Given values that are allegedly sin(θ) and cos(θ) for some θ, checks that
  // they satisfy the condition sin²(θ) + cos²(θ) = 1 (within some fairly
  // relaxed tolerance) and throws an exception if not. Due to this function's
  // relatively high computational cost, it is intended to be called only
  // in Debug mode (using DRAKE_ASSERT_VOID()). (It is likely that the
  // arguments were precomputed to avoid their recalculation in some
  // performance-critical situation.) This function does nothing if T is
  // symbolic.
  static void SinCosConsistencyOrThrow(const T& sin_theta, const T& cos_theta);

  // Sets `this` %RotationMatrix `R_AB` from right-handed orthogonal unit
  // vectors `Bx`, `By`, `Bz` so that the columns of `this` are `[Bx, By, Bz]`.
  // @param[in] Bx first unit vector in right-handed orthogonal basis.
  // @param[in] By second unit vector in right-handed orthogonal basis.
  // @param[in] Bz third unit vector in right-handed orthogonal basis.
  // @throws std::exception in debug builds if `R_AB` fails IsValid(R_AB).
  // @note The rotation matrix `R_AB` relates two sets of right-handed
  // orthogonal unit vectors, namely `Ax`, `Ay`, `Az` and `Bx`, `By`, `Bz`.
  // The rows of `R_AB` are `Ax`, `Ay`, `Az` whereas the
  // columns of `R_AB` are `Bx`, `By`, `Bz`.
  void SetFromOrthonormalColumns(const Vector3<T>& Bx, const Vector3<T>& By,
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
  // @throws std::exception in debug builds if `R_AB` fails R_AB.IsValid().
  // @see SetFromOrthonormalColumns() for additional notes.
  void SetFromOrthonormalRows(const Vector3<T>& Ax, const Vector3<T>& Ay,
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
                                    const Matrix3<T>& other, double tolerance) {
    const T R_max_difference = GetMaximumAbsoluteDifference(R, other);
    return R_max_difference <= tolerance;
  }

  // Throws an exception if R is not a valid %RotationMatrix.
  // @param[in] R an allegedly valid rotation matrix.
  // @note If the underlying scalar type T is non-numeric (symbolic), no
  // validity check is made and no exception is thrown.
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
  static Matrix3<T> ProjectMatrix3ToOrthonormalMatrix3(const Matrix3<T>& M,
                                                       T* quality_factor);

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
      w = 1.0 + trace;
      x = M(2, 1) - M(1, 2);
      y = M(0, 2) - M(2, 0);
      z = M(1, 0) - M(0, 1);
    } else if (M(0, 0) >= M(1, 1) && M(0, 0) >= M(2, 2)) {
      // This branch occurs if M(0,0) is largest among the diagonal elements.
      w = M(2, 1) - M(1, 2);
      x = 1.0 - (trace - 2 * M(0, 0));
      y = M(0, 1) + M(1, 0);
      z = M(0, 2) + M(2, 0);
    } else if (M(1, 1) >= M(2, 2)) {
      // This branch occurs if M(1,1) is largest among the diagonal elements.
      w = M(0, 2) - M(2, 0);
      x = M(0, 1) + M(1, 0);
      y = 1.0 - (trace - 2 * M(1, 1));
      z = M(1, 2) + M(2, 1);
    } else {
      // This branch occurs if M(2,2) is largest among the diagonal elements.
      w = M(1, 0) - M(0, 1);
      x = M(0, 2) + M(2, 0);
      y = M(1, 2) + M(2, 1);
      z = 1.0 - (trace - 2 * M(2, 2));
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
    // clang-format off
    const T M00 = M(0, 0); const T M01 = M(0, 1); const T M02 = M(0, 2);
    const T M10 = M(1, 0); const T M11 = M(1, 1); const T M12 = M(1, 2);
    const T M20 = M(2, 0); const T M21 = M(2, 1); const T M22 = M(2, 2);
    const T trace = M00 + M11 + M22;
    const Vector4<T> wxyz =
        if_then_else(trace >= M00 && trace >= M11 && trace >= M22, Vector4<T>{
          1.0 + trace,
          M21 - M12,
          M02 - M20,
          M10 - M01,
        }, if_then_else(M00 >= M11 && M00 >= M22, Vector4<T>{
          M21 - M12,
          1.0 - (trace - 2 * M00),
          M01 + M10,
          M02 + M20,
        }, if_then_else(M11 >= M22, Vector4<T>{
          M02 - M20,
          M01 + M10,
          1.0 - (trace - 2 * M11),
          M12 + M21,
        }, /* else */ Vector4<T>{
          M10 - M01,
          M02 + M20,
          M12 + M21,
          1.0 - (trace - 2 * M22),
        })));
    // clang-format on
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
  // @throws std::exception if all the elements of quaternion are zero.
  // Throws std::exception in debug builds if any of the elements in quaternion
  // are infinity or NaN.
  static Matrix3<T> QuaternionToRotationMatrix(
      const Eigen::Quaternion<T>& quaternion, const T& two_over_norm_squared);

  // Stores the underlying rotation matrix relating two frames (e.g. A and B).
  // For speed, `R_AB_` is uninitialized (public constructors set its value).
  // The elements are stored in column-major order, per Eigen's default,
  // see https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html.
  Matrix3<T> R_AB_;
};

// To enable low-level optimizations we insist that RotationMatrix<double> is
// packed into 9 consecutive doubles, with no extra alignment padding.
static_assert(
    sizeof(RotationMatrix<double>) == 9 * sizeof(double),
    "Low-level optimizations depend on RotationMatrix<double> being stored as "
    "9 sequential doubles in memory, with no extra memory alignment padding.");

/// Abbreviation (alias/typedef) for a RotationMatrix double scalar type.
/// @relates RotationMatrix
using RotationMatrixd = RotationMatrix<double>;

/// Projects an approximate 3 x 3 rotation matrix M onto an orthonormal matrix R
/// so that R is a rotation matrix associated with a angle-axis rotation by an
/// angle θ about a vector direction `axis`, with `angle_lb <= θ <= angle_ub`.
/// @param[in] M the matrix to be projected.
/// @param[in] axis vector direction associated with angle-axis rotation for R.
///            axis can be a non-unit vector, but cannot be the zero vector.
/// @param[in] angle_lb the lower bound of the rotation angle θ.
/// @param[in] angle_ub the upper bound of the rotation angle θ.
/// @return Rotation angle θ of the projected matrix, angle_lb <= θ <= angle_ub
/// @throws std::exception if axis is the zero vector or
///         if angle_lb > angle_ub.
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
double ProjectMatToRotMatWithAxis(const Eigen::Matrix3d& M,
                                  const Eigen::Vector3d& axis, double angle_lb,
                                  double angle_ub);

}  // namespace math
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RotationMatrix);
