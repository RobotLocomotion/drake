#include "drake/math/quaternion.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace math {

#if 0
template <typename T>
void ThrowIfQuaternionIsNotValid(const Eigen::Quaternion<T>& quaternion) {
  if constexpr (scalar_predicate<T>::is_bool) {
    if (!quaternion.coeffs().is_zero()) {
      throw std::logic_error(
          "Error: All the elements in a quaternion are zero.");
    }
    if (!quaternion.coeffs().allFinite()) {
      throw std::logic_error(
          "Error: Quaternion contains an element that is infinity or NaN.");
    }
  } else {
    unused(quaternion);
  }
}
#endif

}  // namespace math
}  // namespace drake

// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
//    class ::drake::math::Quaternion)
