#include "drake/math/rotation_matrix.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/default_scalars.h"

namespace drake {
namespace math {

template<typename T>
template <typename S>
typename std::enable_if<is_numeric<S>::value>::type
RotationMatrix<T>::ThrowIfNotValid(const Matrix3<T>& R) {
  if (!R.allFinite()) {
    throw std::logic_error(
        "Error: Rotation matrix contains an element that is infinity or "
            "NaN.");
  }
  // If the matrix is not-orthogonal, try to give a detailed message.
  // This is particularly important if matrix is very-near orthogonal.
  if (!IsOrthonormal(R, get_internal_tolerance_for_orthonormality()).value()) {
    const T measure_of_orthonormality = GetMeasureOfOrthonormality(R);
    const double measure = ExtractDoubleOrThrow(measure_of_orthonormality);
    std::string message = fmt::format(
        "Error: Rotation matrix is not orthonormal."
        "  Measure of orthonormality error: {:G}"
        " (near-zero is good).  To orthonormalize a 3x3 matrix,"
        " use RotationMatrix::ProjectToRotationMatrix(), or if"
        " you are using quaternions, ensure you normalize them.", measure);
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
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RotationMatrix)
