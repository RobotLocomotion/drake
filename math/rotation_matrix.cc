#include "drake/math/rotation_matrix.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/default_scalars.h"

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

}  // namespace math
}  // namespace drake

// Explicitly instantiate on non-symbolic scalar types.
// TODO(Mitiguy) Ensure this class handles RotationMatrix<symbolic::Expression>.
// To enable symbolic expressions, remove _NONSYMBOLIC in next line.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::RotationMatrix)
