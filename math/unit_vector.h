#pragma once

#include <string>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Throws unless ‖unit_vector‖ is within 1E-14 (≈ 5.5 bits) of 1.0.
/// Note: 1E-14 ≈ 2^5.5 * std::numeric_limits<double>::epsilon();
/// @param[in] unit_vector a vector which is allegedly a unit vector.
/// @param[in] function_name name of the function that is to appear in the
/// exception message (if an exception is thrown).
/// @throws std::exception if ‖unit_vector‖ is not within 1E-14 of 1.0.
/// @retval ‖unit_vector‖² which is exactly 1.0 for a perfect unit_vector.
/// @note: When type T is symbolic::Expression, this function is a no-op that
/// returns 1.
/// @note Although this function uses a tolerance of 1E-14 to determine if
/// unit_vector is acceptable, unit_vector can be normalized for more accurate
/// calculations (e.g., so ‖unit_vector‖ is within 1E-16 of 1.0).
/// Frequently ‖unit_vector‖ is exactly 1 (e.g., when unit_vector is an x or y
/// or z coordinate axis direction). If not, the calling function should
/// consider normalizing unit_vector even if it passes this test, e.g. as
/// @code{.cpp}
///  using std::sqrt;
///  const T mag_squared =
///      math::ThrowUnlessVectorIsMagnitudeOne(unit_vector, __func__);
///  const Vector3<T> uvec =
///      (mag_squared == 1.0) ? unit_vector : unit_vector / sqrt(mag_squared);
/// @endcode
template <typename T>
T ThrowUnlessVectorIsMagnitudeOne(const Vector3<T>& unit_vector,
                                  std::string_view function_name);

/// If ‖unit_vector‖ is not within 1E-14 (≈ 5.5 bits) of 1.0,
/// writes a warning to the log file (only writes one warning per process).
/// Note: 1E-14 ≈ 2^5.5 * std::numeric_limits<double>::epsilon();
/// @param[in] unit_vector a vector which is allegedly a unit vector.
/// @param[in] function_name name of the function that is to appear in the
/// warning message (if ‖unit_vector‖ is not within tolerance of 1.0).
/// @retval ‖unit_vector‖² which is exactly 1.0 for a perfect unit_vector.
/// @note: When type T is symbolic::Expression, this function is a no-op that
/// returns 1.
/// @note Although this function uses a tolerance of 1E-14 to determine if
/// unit_vector is acceptable, unit_vector can be normalized for more accurate
/// calculations (e.g., so ‖unit_vector‖ is within 1E-16 of 1.0).
/// Frequently ‖unit_vector‖ is exactly 1 (e.g., when unit_vector is an x or y
/// or z coordinate axis direction). If not, the calling function should
/// consider normalizing unit_vector even if it passes this test, e.g. as
/// @code{.cpp}
///  using std::sqrt;
///  const T mag_squared =
///      math::WarnUnlessVectorIsMagnitudeOne(unit_vector, __func__);
///  const Vector3<T> uvec =
///      (mag_squared == 1.0) ? unit_vector : unit_vector / sqrt(mag_squared);
/// @endcode
template <typename T>
T WarnUnlessVectorIsMagnitudeOne(const Vector3<T>& unit_vector,
                                 std::string_view function_name);

}  // namespace math
}  // namespace drake
