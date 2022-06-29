#include "drake/math/quaternion.h"

#include <string>

#include <fmt/format.h>

#include "drake/common/unused.h"

namespace drake {
namespace math {

template <typename T>
void ThrowIfAllElementsInQuaternionAreZero(
    const Eigen::Quaternion<T>& quaternion, const char* function_name) {
  if constexpr (scalar_predicate<T>::is_bool) {
    if (IsQuaternionZero(quaternion)) {
      std::string message = fmt::format("{}():"
        " All the elements in a quaternion are zero.", function_name);
      throw std::logic_error(message);
    }
  } else {
    unused(quaternion, function_name);
  }
}

template <typename T>
void ThrowIfAnyElementInQuaternionIsInfinityOrNaN(
    const Eigen::Quaternion<T>& quaternion, const char* function_name) {
  if constexpr (scalar_predicate<T>::is_bool) {
    if (!quaternion.coeffs().allFinite()) {
      std::string message = fmt::format("{}():"
        " Quaternion contains an element that is infinity or NaN.",
        function_name);
      throw std::logic_error(message);
    }
  } else {
    unused(quaternion, function_name);
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &ThrowIfAllElementsInQuaternionAreZero<T>,
    &ThrowIfAnyElementInQuaternionIsInfinityOrNaN<T>
))

}  // namespace math
}  // namespace drake

