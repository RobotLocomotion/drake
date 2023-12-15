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
// @note: When type T is symbolic::Expression, this function is a no-op that
// does not throw an exception.
template <typename T>
void ThrowIfNotUnitVector(
    const Vector3<T>& unit_vector, std::string_view function_name,
    double tolerance_unit_vector_norm = kToleranceUnitVectorNorm);

// Returns the unit vector in the direction of v or throws an exception if v
// cannot be "safely" normalized.
// @param[in] v The vector to normalize.
// @param[in] function_name name of the function that is to appear in the
// exception message (if an exception is thrown).
// @throws std::exception if v contains nonfinite numbers (NaN or infinity)
// or ‖v‖ < 1E-10.
// @note: No exception is thrown when type T is symbolic::Expression.
// TODO(Mitiguy) Consider evolving towards a consistent policy of ‖v‖ ≈ 1.0
//  instead of ‖v‖ ≥ 1E-10, somewhat similar to ThrowIfNotUnitVector().
template <typename T>
Vector3<T> NormalizeOrThrow(const Vector3<T>& v,
                            std::string_view function_name);

}  // namespace internal
}  // namespace math
}  // namespace drake
