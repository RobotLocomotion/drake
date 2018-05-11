#include "drake/math/rotation_matrix.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/default_scalars.h"

namespace drake {
namespace math {
//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T>::RotationMatrix(const Matrix3<T>& R) : R_AB_() {
#ifdef DRAKE_ASSERT_IS_ARMED
  SetOrThrowIfNotValid(R);
#else
  SetUnchecked(R);
#endif
}

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T>::RotationMatrix(const Matrix3<T>& R, bool) : R_AB_(R) {}

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T>::RotationMatrix(const Eigen::Quaternion<T>& quaternion) {
  // Cost for various way to create a rotation matrix from a quaternion.
  // Eigen quaternion.toRotationMatrix() = 12 multiplies, 12 adds.
  // Drake  QuaternionToRotationMatrix() = 12 multiplies, 12 adds.
  // Extra cost for two_over_norm_squared =  4 multiplies,  3 adds, 1 divide.
  // Extra cost if normalized = 4 multiplies, 3 adds, 1 sqrt, 1 divide.
  const T two_over_norm_squared = T(2) / quaternion.squaredNorm();
  R_AB_ = QuaternionToRotationMatrix(quaternion, two_over_norm_squared);
  DRAKE_ASSERT_VOID(ThrowIfNotValid(R_AB_));
}

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T>::RotationMatrix(const Eigen::AngleAxis<T>& theta_lambda) {
  const Vector3<T>& lambda = theta_lambda.axis();
  const T norm = lambda.norm();
  const T& theta = theta_lambda.angle();
  R_AB_ = Eigen::AngleAxis<T>(theta, lambda / norm).toRotationMatrix();
  DRAKE_ASSERT_VOID(ThrowIfNotValid(R_AB_));
}

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T>::RotationMatrix(const RollPitchYaw<T>& rpy) {
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

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T> RotationMatrix<T>::MakeXRotation(const T& theta) {
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

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T> RotationMatrix<T>::MakeYRotation(const T& theta) {
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

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T> RotationMatrix<T>::MakeZRotation(const T& theta) {
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

//------------------------------------------------------------------------------
template <typename T>
const RotationMatrix<T>& RotationMatrix<T>::Identity() {
  static const never_destroyed<RotationMatrix<T>> kIdentity;
  return kIdentity.access();
}

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T> RotationMatrix<T>::inverse() const {
  return RotationMatrix<T>(R_AB_.transpose());
}

//------------------------------------------------------------------------------
template <typename T>
const Matrix3<T>& RotationMatrix<T>::matrix() const { return R_AB_; }

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T> RotationMatrix<T>::operator*(
    const RotationMatrix<T>& other) const {
  return RotationMatrix<T>(matrix() * other.matrix(), true);
}

//------------------------------------------------------------------------------
template <typename T>
RotationMatrix<T>& RotationMatrix<T>::operator*=(
    const RotationMatrix<T>& other) {
  SetUnchecked(matrix() * other.matrix());
  return *this;
}

//------------------------------------------------------------------------------
template <typename T>
Vector3<T> RotationMatrix<T>::operator*(const Vector3<T>& v) const {
  return Vector3<T>(matrix() * v);
}

//------------------------------------------------------------------------------
template <typename T>
T RotationMatrix<T>::GetMeasureOfOrthonormality(const Matrix3<T>& R) {
  const Matrix3<T> m = R * R.transpose();
  return GetMaximumAbsoluteDifference(m, Matrix3<T>::Identity());
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsOrthonormal(const Matrix3<T>& R, double tolerance) {
  return GetMeasureOfOrthonormality(R) <= tolerance;
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsValid(const Matrix3<T>& R, double tolerance) {
  return IsOrthonormal(R, tolerance) && R.determinant() > 0;
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsValid(const Matrix3<T>& R) {
  return IsValid(R, get_internal_tolerance_for_orthonormality());
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsValid() const { return IsValid(matrix()); }

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsExactlyIdentity() const {
  return matrix() == Matrix3<T>::Identity();
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsIdentityToInternalTolerance() const {
  return IsNearlyEqualTo(matrix(), Matrix3<T>::Identity(),
                         get_internal_tolerance_for_orthonormality());
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsNearlyEqualTo(const RotationMatrix<T>& other,
                                        double tolerance) const {
  return IsNearlyEqualTo(matrix(), other.matrix(), tolerance);
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsExactlyEqualTo(const RotationMatrix<T>& other) const {
  return matrix() == other.matrix();
}

//------------------------------------------------------------------------------
template <typename T>
T RotationMatrix<T>::GetMaximumAbsoluteDifference(
    const RotationMatrix<T>& other) const {
  return GetMaximumAbsoluteDifference(matrix(), other.matrix());
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::Quaternion<T> RotationMatrix<T>::ToQuaternion() const {
  return ToQuaternion(R_AB_);
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::Quaternion<T> RotationMatrix<T>::ToQuaternion(
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

//------------------------------------------------------------------------------
template <typename T>
Vector4<T> RotationMatrix<T>::ToQuaternionAsVector4() const {
  return ToQuaternionAsVector4(R_AB_);
}

//------------------------------------------------------------------------------
template <typename T>
Vector4<T> RotationMatrix<T>::ToQuaternionAsVector4(const Matrix3<T>& M)  {
  const Eigen::Quaternion<T> q = ToQuaternion(M);
  return Vector4<T>(q.w(), q.x(), q.y(), q.z());
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::AngleAxis<T> RotationMatrix<T>::ToAngleAxis() const {
  const Eigen::AngleAxis<T> theta_lambda(this->matrix());
  return theta_lambda;
}

//------------------------------------------------------------------------------
template <typename T>
void RotationMatrix<T>::SetUnchecked(const Matrix3<T>& R) { R_AB_ = R; }

//------------------------------------------------------------------------------
template <typename T>
T RotationMatrix<T>::GetMaximumAbsoluteDifference(const Matrix3<T>& R,
                                                  const Matrix3<T>& other) {
  const Matrix3<T> R_difference = R - other;
  return R_difference.template lpNorm<Eigen::Infinity>();
}

//------------------------------------------------------------------------------
template <typename T>
bool RotationMatrix<T>::IsNearlyEqualTo(const Matrix3<T>& R,
                                    const Matrix3<T>& other, double tolerance) {
  const T R_max_difference = GetMaximumAbsoluteDifference(R, other);
  return R_max_difference <= tolerance;
}

//------------------------------------------------------------------------------
template <typename T>
Matrix3<T> RotationMatrix<T>::QuaternionToRotationMatrix(
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

//------------------------------------------------------------------------------
template <typename T>
void RotationMatrix<T>::SetOrThrowIfNotValid(const Matrix3<T>& R) {
  ThrowIfNotValid(R);
  SetUnchecked(R);
}

//------------------------------------------------------------------------------
template <typename T>
void RotationMatrix<T>::ThrowIfNotValid(const Matrix3<T>& R) {
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
        "Error: Rotation matrix is not orthonormal."
        "  Measure of orthonormality error: {:G}  (near-zero is good)."
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

// Explicitly instantiate on non-symbolic scalar types.
// TODO(Mitiguy) Ensure this class handles RotationMatrix<symbolic::Expression>.
// To enable symbolic expressions, remove _NONSYMBOLIC in next line.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::RotationMatrix)
