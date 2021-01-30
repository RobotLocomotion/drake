#pragma once

#include <utility>

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {
/* Verifies that the given Young's modulus and Poisson ratio are valid. If so,
 calculates the Lamé parameters from the Young's modulus and the Poisson
 ratio. If not, throw.
 @tparam_nonsymbolic_scalar T.
 @throw std::exception if `youngs_modulus` is negative or if `poisson_ratio` is
 not in (-1, 0.5). */
template <typename T>
std::pair<T, T> CalcLameParameters(const T& youngs_modulus,
                                   const T& poisson_ratio) {
  if (youngs_modulus < 0.0) {
    throw std::logic_error("Young's modulus must be nonnegative.");
  }
  if (poisson_ratio >= 0.5 || poisson_ratio <= -1) {
    throw std::logic_error("Poisson ratio must be in (-1, 0.5).");
  }
  T mu = youngs_modulus / (2.0 * (1.0 + poisson_ratio));
  T lambda = youngs_modulus * poisson_ratio /
             ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
  return std::make_pair(lambda, mu);
}
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
