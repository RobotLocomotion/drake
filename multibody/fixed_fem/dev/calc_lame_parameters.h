#pragma once

#include <utility>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Verifies that the given Young's modulus and Poisson ratio are valid. If so,
 calculates the Lamé parameters from the Young's modulus and the Poisson
 ratio. If not, throw.
 @tparam_nonsymbolic_scalar T.
 @throw std::exception if `youngs_modulus` is negative or if `poisson_ratio` is
 not in (-1, 0.5). */
template <typename T>
std::pair<T, T> CalcLameParameters(const T& youngs_modulus,
                                   const T& poisson_ratio);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
