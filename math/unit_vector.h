#pragma once

#include <limits>
#include <string>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
namespace internal {

// Tolerance for ‖unit_vector‖ is 3 bits (= 8ε ≈ 1.78E-15) of 1.0, where
// 3 bits = 2³ε = 8ε and ε = std::numeric_limits<double>::epsilon().
// @note The use of 3 bits was determined empirically, is well within the
// tolerance achieved by normalizing a vast range of non-zero vectors, and
// seems to provide a valid RotationMatrix() (see RotationMatrix::IsValid()).
constexpr double kTolerance_unit_vector_norm =
    8 * std::numeric_limits<double>::epsilon();

// Throws unless ‖unit_vector‖ is within kTolerance_unit_vector_norm of 1.0.
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @param[in] function_name name of the function that is to appear in the
// exception message (if an exception is thrown).
// @throws std::exception if ‖unit_vector‖ is not within tolerance of 1.0.
// @retval ‖unit_vector‖² which is exactly 1.0 for a perfect unit_vector.
// @note: When type T is symbolic::Expression, this function is a no-op that
// does not throw an exception and returns 1.0.
// @note Frequently ‖unit_vector‖ is exactly 1.0 (e.g., when unit_vector is an
// x or y or z axis direction). If the retval ‖unit_vector‖² is not exactly 1.0,
// the calling function should consider normalizing unit_vector even when it
// passes this test so ‖unit_vector‖ is within ε ≈ 2.22E-16 of 1.0, e.g., as
// @code{.cpp}
//  using std::sqrt;
//  const T mag_squared =
//      math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
//  if (mag_squared != 1.0) unit_vector /= sqrt(mag_squared);
// @endcode
template <typename T>
T ThrowIfNotUnitVector(const Vector3<T>& unit_vector,
                       std::string_view function_name);

// If ‖unit_vector‖ is not within kTolerance_unit_vector_norm of 1.0,
// writes a warning to the log file (only writes one warning per process).
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @param[in] function_name name of the function that appears in the message
// written to the log file (if ‖unit_vector‖ is not within tolerance of 1.0).
// @retval ‖unit_vector‖² which is exactly 1.0 for a perfect unit_vector.
// @note: When type T is symbolic::Expression, this function is a no-op that
// does not write to the log file and returns 1.0.
// @note Frequently ‖unit_vector‖ is exactly 1.0 (e.g., when unit_vector is an
// x or y or z axis direction). If the retval ‖unit_vector‖² is not exactly 1.0,
// the calling function should consider normalizing unit_vector even when it
// passes this test so ‖unit_vector‖ is within ε ≈ 2.22E-16 of 1.0, e.g., as
// @code{.cpp}
//  using std::sqrt;
//  const T mag_squared =
//      math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
//  if (mag_squared != 1.0) unit_vector /= sqrt(mag_squared);
// @endcode
// TODO(2023-12-01) Change calls to this function to ThrowIfNotUnitVector().
template <typename T>
T WarnIfNotUnitVector(const Vector3<T>& unit_vector,
                      std::string_view function_name);

}  // namespace internal
}  // namespace math
}  // namespace drake
