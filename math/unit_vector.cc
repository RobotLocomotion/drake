#include "drake/math/unit_vector.h"

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace math {
namespace internal {

namespace {
// Checks if ‖unit_vector‖ is within 2 bits of 1.0, where 2 bits is
// (2² = 4) * std::numeric_limits<double>::epsilon() ≈ 8.88E-16.
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @retval {‖unit_vector‖², is_ok_unit_vector} as a pair. The return value is
// {‖unit_vector‖², true} for ‖unit_vector‖ within 2 bits of 1, otherwise return
// {‖unit_vector‖², false}.
// @note: When type T is symbolic::Expression, this function is a no-op that
// returns {1.0, {}}.
// @note The use of 2 bits was determined empirically, is well within the
// tolerance achieved by normalizing a vast range of non-zero vectors, and
// seems to provide a valid RotationMatrix() (see RotationMatrix::IsValid()).
template<typename T>
std::pair<T, bool> IsUnitVector(const Vector3<T> &unit_vector) {
  if constexpr (scalar_predicate<T>::is_bool) {
    // A test that a unit vector's magnitude is within a very small ε of 1 is
    // |√(𝐯⋅𝐯) − 1| ≤ ε. To avoid an unnecessary square-root, notice that this
    // simple test is equivalent to  ±(√(𝐯⋅𝐯) - 1) ≤ ε  which means that both
    // √(𝐯⋅𝐯) - 1 ≤ ε  and √(𝐯⋅𝐯) - 1 ≥ -ε,  or
    // √(𝐯⋅𝐯) ≤ 1 + ε  and √(𝐯⋅𝐯) ≥ 1 - ε.  Squaring these two equations give
    // 𝐯⋅𝐯 ≤ (1 + ε)²  and 𝐯⋅𝐯 ≥ (1 - ε)².  Distributing the square results in
    // 𝐯⋅𝐯 ≤ 1 + 2 ε + ε²  and 𝐯⋅𝐯 ≥ 1 - 2 ε + ε². Since ε² ≪ 2 ε, this gives
    // 𝐯⋅𝐯 - 1 ≤ 2 ε   and 𝐯⋅𝐯 - 1 ≥ -2 ε  or  |𝐯⋅𝐯 − 1| ≤ 2 ε
    // -------------------------------------------------------------
    // Hence the following test that requires √ is replaced by an efficient one.
    // constexpr double kTolerance = 4 * std::numeric_limits<double>::epsilon();
    // is_ok_unit_vector = (abs(unit_vector.norm() - 1) <=  Tolerance;
    // -------------------------------------------------------------
    using std::abs;
    using std::isfinite;
    constexpr double kTolerance2 = 8 * std::numeric_limits<double>::epsilon();
    const T uvec_squared = unit_vector.squaredNorm();
    const bool is_ok_unit_vector = isfinite(uvec_squared) &&
        abs(uvec_squared - 1) <= kTolerance2;
    return {uvec_squared, is_ok_unit_vector};
  }
  return {1.0, true};
}

// Returns an error message that ‖unit_vector‖ ≠ 1.
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @param[in] function_name function name that appears in the error_message
// returned by this function.
// @retval error_message string that can be subsequently used as an exception
// message or to write to a log file.
// @note: This helper function only creates an error message. It does not verify
// ‖unit_vector‖ ≠ 1. This helper function is used by ThrowIfNotUnitVector().
template <typename T>
std::string ErrorMessageNotUnitVector(const Vector3<T>& unit_vector,
                                      std::string_view function_name) {
  if constexpr (scalar_predicate<T>::is_bool) {
    DRAKE_DEMAND(!function_name.empty());
    using std::abs;
    const T norm = unit_vector.norm();
    const T norm_diff = abs(1.0 - norm);
    constexpr double kTolerance = 4 * std::numeric_limits<double>::epsilon();
    const std::string error_message =
      fmt::format("{}(): The unit_vector argument {} is not a unit vector.\n"
                  "|unit_vector| = {}\n"
                  "||unit_vector| - 1| = {} is greater than {}.",
                  function_name, fmt_eigen(unit_vector.transpose()),
                  norm, norm_diff, kTolerance);
    return error_message;
  }
  return {};
}

}  // namespace

template <typename T>
T ThrowIfNotUnitVector(const Vector3<T>& unit_vector,
                       std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  auto [unit_vector_squaredNorm, is_ok_unit_vector] = IsUnitVector(unit_vector);
  if (!is_ok_unit_vector) {
    throw std::logic_error(
        ErrorMessageNotUnitVector(unit_vector, function_name));
  }
  return unit_vector_squaredNorm;
}

template <typename T>
T WarnIfNotUnitVector(const Vector3<T>& unit_vector,
    std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  auto [unit_vector_squaredNorm, is_ok_unit_vector] = IsUnitVector(unit_vector);
  if (!is_ok_unit_vector) {
    static const logging::Warn log_once(
        "{}\nImplicit normalization is deprecated; on or after 2023-12-01 this "
        "will become an exception.",
        ErrorMessageNotUnitVector(unit_vector, function_name));
  }
  return unit_vector_squaredNorm;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &ThrowIfNotUnitVector<T>,
    &WarnIfNotUnitVector<T>
))

}  // namespace internal
}  // namespace math
}  // namespace drake
