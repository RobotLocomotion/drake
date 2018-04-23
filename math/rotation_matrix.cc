#include "drake/math/rotation_matrix.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/default_scalars.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace math {

template<typename T>
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

template<typename T>
RotationMatrix<T>::RotationMatrix(const RollPitchYaw<T>& rpy) {
  const T& r = rpy.get_roll_angle();
  const T& p = rpy.get_pitch_angle();
  const T& y = rpy.get_yaw_angle();
  using std::sin;
  using std::cos;
  const T c0 = cos(r), c1 = cos(p), c2 = cos(y);
  const T s0 = sin(r), s1 = sin(p), s2 = sin(y);
  const T Rxx = c2 * c1;
  const T Rxy = c2 * s1 * s0 - s2 * c0;
  const T Rxz = c2 * s1 * c0 + s2 * s0;
  const T Ryx = s2 * c1;
  const T Ryy = s2 * s1 * s0 + c2 * c0;
  const T Ryz = s2 * s1 * c0 - c2 * s0;
  const T Rzx = -s1;
  const T Rzy = c1 * s0;
  const T Rzz = c1 * c0;
  R_AB_.row(0) << Rxx, Rxy, Rxz;
  R_AB_.row(1) << Ryx, Ryy, Ryz;
  R_AB_.row(2) << Rzx, Rzy, Rzz;
}

}  // namespace math
}  // namespace drake

// Explicitly instantiate on non-symbolic scalar types.
// TODO(Mitiguy) Ensure this class handles RotationMatrix<symbolic::Expression>.
// To enable symbolic expressions, remove _NONSYMBOLIC in next line.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::RotationMatrix)
