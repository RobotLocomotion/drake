#pragma once

#include <string>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Throws unless ‖unit_vector‖ is within 2 bits (= 4ε ≈ 8.88E-16) of 1.0, where
/// 2 bits = 2²ε = 4ε and ε = std::numeric_limits<double>::epsilon().
/// @param[in] unit_vector a vector which is allegedly a unit vector.
/// @param[in] function_name name of the function that is to appear in the
/// exception message (if an exception is thrown).
/// @throws std::exception if ‖unit_vector‖ is not within 4ε of 1.0.
/// @retval ‖unit_vector‖² which is exactly 1.0 for a perfect unit_vector.
/// @note: When type T is symbolic::Expression, this function is a no-op that
/// does not throw an exception and returns 1.0.
/// @note Frequently ‖unit_vector‖ is exactly 1.0 (e.g., when unit_vector is an
/// x or y or z coordinate axis direction). If not, the calling function should
/// consider normalizing unit_vector even if it passes this test so that
/// ‖unit_vector‖ is less than ε ≈ 2.22E-16 of 1.0, e.g., as
/// @code{.cpp}
///  using std::sqrt;
///  const T mag_squared = math::ThrowIfNotUnitVector(unit_vector, __func__);
///  const Vector3<T> uvec =
///      (mag_squared == 1.0) ? unit_vector : unit_vector / sqrt(mag_squared);
/// @endcode
template <typename T>
T ThrowIfNotUnitVector(const Vector3<T>& unit_vector,
                       std::string_view function_name);

/// If ‖unit_vector‖ is not within 2 bits (= 4ε ≈ 8.88E-16) of 1.0,
/// writes a warning to the log file (only writes one warning per process).
/// Note: 2 bits = 2²ε = 4ε and ε = std::numeric_limits<double>::epsilon().
/// @param[in] unit_vector a vector which is allegedly a unit vector.
/// @param[in] function_name name of the function that appears in the message
/// written to the log file (if ‖unit_vector‖ is not within tolerance of 1.0).
/// @retval ‖unit_vector‖² which is exactly 1.0 for a perfect unit_vector.
/// @note: When type T is symbolic::Expression, this function is a no-op that
/// does not write to the log file and returns 1.0.
/// @note Frequently ‖unit_vector‖ is exactly 1.0 (e.g., when unit_vector is an
/// x or y or z coordinate axis direction). If not, the calling function should
/// consider normalizing unit_vector even if it passes this test so that
/// ‖unit_vector‖ is less than ε ≈ 2.22E-16 of 1.0, e.g., as
/// @code{.cpp}
///  using std::sqrt;
///  const T mag_squared = math::ThrowIfNotUnitVector(unit_vector, __func__);
///  const Vector3<T> uvec =
///      (mag_squared == 1.0) ? unit_vector : unit_vector / sqrt(mag_squared);
/// @endcode
template <typename T>
T WarnIfNotUnitVector(const Vector3<T>& unit_vector,
                      std::string_view function_name);

}  // namespace math
}  // namespace drake
