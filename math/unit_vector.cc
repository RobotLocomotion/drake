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
// @param[in] tolerance_unit_vector_norm small positive real number that
// specifies the allowable tolerance for ‖unit_vector‖ from 1.0.
// @retval true if ‖unit_vector‖ is OK, otherwise false.
// @note: When type T is symbolic::Expression, this function is a no-op that
// returns true.
template <typename T>
bool IsUnitVector(const Vector3<T>& unit_vector,
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
    const double tolerance2 = 2 * tolerance_unit_vector_norm;
    const double uvec_squared = DiscardGradient(unit_vector).squaredNorm();
    const bool is_ok_unit_vector = std::isfinite(uvec_squared) &&
                                   std::abs(uvec_squared - 1.0) <= tolerance2;
    return is_ok_unit_vector;
  } else {
    unused(unit_vector, tolerance_unit_vector_norm);
    return true;
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
Vector3<T> NormalizeOrThrow(const Vector3<T>& v,
                            std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  const T norm = v.norm();
  if constexpr (scalar_predicate<T>::is_bool) {
    // Throw an exception if norm is non-finite (NaN or infinity) or too small.
    // The threshold for "too small" is a heuristic (rule of thumb) guided by an
    // expected small physical dimensions in a robotic systems. Numbers smaller
    // than this are probably user or developer errors.
    constexpr double kMinMagnitude = 1e-10;
    using std::isfinite;
    if (!(isfinite(norm) && norm >= kMinMagnitude)) {
      throw std::logic_error(fmt::format(
          "{}() cannot normalize the given vector v.\n"
          "   v = {}\n"
          " |v| = {}\n"
          " The measures must be finite and the vector must have a magnitude of"
          " at least {} to normalize. If you are confident that v's direction"
          " is meaningful, pass v.normalized() instead of v.",
          function_name, fmt_eigen(DiscardGradient(v).transpose()),
          ExtractDoubleOrThrow(norm), kMinMagnitude));
    }
  }
  return v / norm;
}

template <typename T>
void ThrowIfNotUnitVector(const Vector3<T>& unit_vector,
                          std::string_view function_name,
                          const double tolerance_unit_vector_norm) {
  DRAKE_DEMAND(!function_name.empty());
  if (!IsUnitVector(unit_vector, tolerance_unit_vector_norm)) {
    throw std::logic_error(ErrorMessageNotUnitVector(
        unit_vector, function_name, tolerance_unit_vector_norm));
  }
}

template <typename T>
bool WarnIfNotUnitVector(const Vector3<T>& unit_vector,
                         std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  const bool is_ok_unit_vector =
      IsUnitVector(unit_vector, kToleranceUnitVectorNorm);
  if (!is_ok_unit_vector) {
    const std::string msg_not_unit_vector = ErrorMessageNotUnitVector(
        unit_vector, function_name, kToleranceUnitVectorNorm);
    static const drake::internal::WarnDeprecated warn_once(
        "2023-12-01", fmt::format("{} Implicit normalization is deprecated.",
                                  msg_not_unit_vector));
  }
  return !is_ok_unit_vector;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&NormalizeOrThrow<T>, &ThrowIfNotUnitVector<T>, &WarnIfNotUnitVector<T>))

}  // namespace internal
}  // namespace math
}  // namespace drake
