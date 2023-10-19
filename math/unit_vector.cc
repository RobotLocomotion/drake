#include "drake/math/unit_vector.h"

#include <cmath>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace math {
namespace internal {

namespace {

// Checks if ‖unit_vector‖ is within tolerance_unit_vector_norm of 1.0.
// @param[in] unit_vector a vector which is allegedly a unit vector.
// @retval {‖unit_vector‖², is_ok_unit_vector} as pair. If ‖unit_vector‖ is OK,
// returns {‖unit_vector‖², true}, otherwise returns {‖unit_vector‖², false}.
// @note: When type T is symbolic::Expression, this function is a no-op that
// returns {1.0, {true}}.
template <typename T>
std::pair<T, bool> IsUnitVector(const Vector3<T>& unit_vector,
                                const double tolerance_unit_vector_norm) {
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
    // Hence the following test with norm() that uses an extra √, e.g., as
    // sqrt(squaredNorm()), is replaced by one that only uses squaredNorm().
    // is_ok_unit_vector =
    //     (abs(unit_vector.norm() - 1) <=  tolerance_unit_vector_norm;
    // -------------------------------------------------------------
    using std::abs;
    using std::isfinite;
    const double tolerance2 = 2 * tolerance_unit_vector_norm;

    // In calculating ‖unit_vector‖² for AutoDiff type <T>, there is no need to
    // calculate derivatives. Use DiscardGradient() to skip that calculation.
    const double uvec_squared = DiscardGradient(unit_vector).squaredNorm();
    const bool is_ok_unit_vector =
        isfinite(uvec_squared) && abs(uvec_squared - 1) <= tolerance2;

    return {uvec_squared, is_ok_unit_vector};
  } else {
    unused(unit_vector, tolerance_unit_vector_norm);
    return {1.0, true};
  }
}

// Returns an error message that ‖unit_vector‖ ≠ 1.
// @param[in] bad_unit_vector a vector which is not a unit vector because
// ‖bad_unit_vector‖ ≠ 1, which may be due to NaN or infinity elements.
// @param[in] function_name function name that appears in the error_message
// returned by this function.
// @retval error_message string that can be subsequently used as an exception
// message or to write to a log file.
// @pre ‖bad_unit_vector‖ is not a valid unit vector.
// @note: This helper function only creates an error message. It does not verify
// ‖bad_unit_vector‖ ≠ 1.
template <typename T>
std::string ErrorMessageNotUnitVector(const Vector3<T>& bad_unit_vector,
                                      std::string_view function_name,
                                      const double tolerance_unit_vector_norm) {
  if constexpr (scalar_predicate<T>::is_bool) {
    DRAKE_DEMAND(!function_name.empty());
    using std::abs;
    const T norm = bad_unit_vector.norm();
    const T norm_diff = abs(1.0 - norm);
    const std::string error_message = fmt::format(
        "{}(): The unit_vector argument {} is not a unit vector.\n"
        "|unit_vector| = {}\n"
        "||unit_vector| - 1| = {} is greater than {}.",
        function_name, fmt_eigen(bad_unit_vector.transpose()), norm, norm_diff,
        tolerance_unit_vector_norm);
    return error_message;
  } else {
    unused(bad_unit_vector, function_name, tolerance_unit_vector_norm);
    return {};
  }
}

}  // namespace

template <typename T>
T ThrowIfNotUnitVector(const Vector3<T>& unit_vector,
                       std::string_view function_name,
                       const double tolerance_unit_vector_norm) {
  DRAKE_DEMAND(!function_name.empty());
  auto [unit_vector_squared_norm, is_ok_unit_vector] =
      IsUnitVector(unit_vector, tolerance_unit_vector_norm);
  if (!is_ok_unit_vector) {
    throw std::logic_error(ErrorMessageNotUnitVector(
        unit_vector, function_name, tolerance_unit_vector_norm));
  }
  return unit_vector_squared_norm;
}

template <typename T>
T WarnIfNotUnitVector(const Vector3<T>& unit_vector,
                      std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  auto [unit_vector_squared_norm, is_ok_unit_vector] =
      IsUnitVector(unit_vector, kToleranceUnitVectorNorm);
  if (!is_ok_unit_vector) {
    const std::string msg_not_unit_vector = ErrorMessageNotUnitVector(
        unit_vector, function_name, kToleranceUnitVectorNorm);
    static const drake::internal::WarnDeprecated warn_once(
        "2023-12-01", fmt::format("{} Implicit normalization is deprecated.",
                                  msg_not_unit_vector));
  }
  return unit_vector_squared_norm;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&ThrowIfNotUnitVector<T>, &WarnIfNotUnitVector<T>))

}  // namespace internal
}  // namespace math
}  // namespace drake
