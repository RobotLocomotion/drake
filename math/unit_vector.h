#pragma once

#include <string>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
namespace internal {

// Default tolerance for ‖unit_vector‖ is 1E-14 (≈ 5.5 bits) of 1.0.
// Note: 1E-14 ≈ 2^5.5 * std::numeric_limits<double>::epsilon();
constexpr double kToleranceUnitVectorNorm = 1.0E-14;

// Throws unless ‖unit_vector‖ is within tolerance_unit_vector_norm of 1.0.
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @param[in] function_name name of the function that is to appear in the
// exception message (if an exception is thrown).
// @param[in] tolerance_unit_vector_norm allowable tolerance for ‖unit_vector‖
// from 1.0. The default value is kToleranceUnitVectorNorm.
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
T ThrowIfNotUnitVector(
    const Vector3<T>& unit_vector, std::string_view function_name,
    double tolerance_unit_vector_norm = kToleranceUnitVectorNorm);

// If ‖unit_vector‖ is not within kToleranceUnitVectorNorm of 1.0,
// writes a warning to the log file (only writes one warning per process).
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @param[in] function_name name of the function that appears in the message
// written to the log file (if a warning message needs to be written).
// @note: This function is similar to ThrowIfNotUnitVector(), except all calls
// to WarnIfNotUnitVector() use the default tolerance kToleranceUnitVectorNorm.
// See ThrowIfNotUnitVector() for more information.
// TODO(2023-12-01) Change all calls to WarnIfNotUnitVector() to:
//  ThrowIfNotUnitVector(unit_vector, function_name). If this function does not
//  have utility past that date, consider also deleting this function.
//  Alternatively, consider updating this function to address Jeremy's concerns:
//  "This function is not suitable to be used from multiple call sites.  It only
//  warns once per process, instead of once per date or once per class or once
//  per function. Once per process is not sufficient to give users enough
//  feedback when they have non-unit vectors reaching multiple points of entry."
template <typename T>
T WarnIfNotUnitVector(const Vector3<T>& unit_vector,
                      std::string_view function_name);

}  // namespace internal
}  // namespace math
}  // namespace drake
