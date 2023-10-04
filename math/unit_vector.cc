#include "drake/math/unit_vector.h"

#include <cmath>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace math {

// The implementation logic underlying ThrowUnlessVectorIsMagnitudeOne(), but
// instead of throwing, returns the exception message as an extra return value.
// When there are no errors, the error_message will be empty.
// @retval {‖unit_vector‖², error_message} as a pair.
template <typename T>
std::pair<T, std::string> CheckVectorIsMagnitudeOne(
    const Vector3<T>& unit_vector, std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  if constexpr (scalar_predicate<T>::is_bool) {
    using std::abs;
    // A test that a unit vector's magnitude is within a very small ε of 1 is
    // |√(𝐯⋅𝐯) − 1| ≤ ε. To avoid an unnecessary square-root, notice that this
    // simple test is equivalent to  ±(√(𝐯⋅𝐯) - 1) ≤ ε  which means that both
    // √(𝐯⋅𝐯) - 1 ≤ ε  and √(𝐯⋅𝐯) - 1 ≥ -ε,  or
    // √(𝐯⋅𝐯) ≤ 1 + ε  and √(𝐯⋅𝐯) ≥ 1 - ε.  Squaring these two equations give
    // 𝐯⋅𝐯 ≤ (1 + ε)²  and 𝐯⋅𝐯 ≥ (1 - ε)².  Distributing the square results in
    // 𝐯⋅𝐯 ≤ 1 + 2 ε + ε²  and 𝐯⋅𝐯 ≥ 1 - 2 ε + ε². Since ε² ≪ 2 ε, this gives
    // 𝐯⋅𝐯 - 1 ≤ 2 ε   and 𝐯⋅𝐯 - 1 ≥ -2 ε  or  |𝐯⋅𝐯 − 1| ≤ 2 ε
    // -------------------------------------------------------------
    // Hence the following simple test can be replaced by a more efficient one.
    // constexpr double kTolerance = 1E-14;
    // if (abs(unit_vector.norm() - 1) > kTolerance) {
    // -------------------------------------------------------------
    constexpr double kTolerance2 = 2E-14;
    const T uvec_squared = unit_vector.squaredNorm();
    if (abs(uvec_squared - 1) > kTolerance2) {
      const std::string error_message =
          fmt::format("{}(): The unit_vector argument {} is not a unit vector.",
                      function_name, fmt_eigen(unit_vector.transpose()));
      return {uvec_squared, error_message};
    }
    return {uvec_squared, {}};
  }
  return {1.0, {}};
}

template <typename T>
T ThrowUnlessVectorIsMagnitudeOne(const Vector3<T>& unit_vector,
                                  std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  auto [result, error_message] =
      CheckVectorIsMagnitudeOne(unit_vector, function_name);
  if (!error_message.empty()) {
    throw std::logic_error(error_message);
  }
  return result;
}

// Like ThrowUnlessVectorIsMagnitudeOne(), but warns (once per process) instead
// of throwing.
template <typename T>
T WarnUnlessVectorIsMagnitudeOne(const Vector3<T>& unit_vector,
                                 std::string_view function_name) {
  DRAKE_DEMAND(!function_name.empty());
  auto [result, error_message] =
      CheckVectorIsMagnitudeOne(unit_vector, function_name);
  if (!error_message.empty()) {
    static const logging::Warn log_once(
        "{} Implicit normalization is deprecated; on or after 2023-12-01 this "
        "will become an exception.",
        error_message);
  }
  return result;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &ThrowUnlessVectorIsMagnitudeOne<T>,
    &WarnUnlessVectorIsMagnitudeOne<T>
))

}  // namespace math
}  // namespace drake
