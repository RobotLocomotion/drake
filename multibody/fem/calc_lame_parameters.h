#pragma once

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
struct LameParameters {
  T lambda{};  // First Lamé parameter.
  T mu{};      // Second Lamé parameter.
};

/* Verifies that the given Young's modulus and Poisson's ratio are valid. If so,
 calculates the Lamé parameters from the Young's modulus and the Poisson ratio.
 If not, throw an exception. Note that a Poisson's ratio of -1 or 0.5 is _not_
 allowed.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd.
 @throw std::exception if `youngs_modulus` is negative or if `poissons_ratio` is
 not in (-1, 0.5). */
template <typename T>
LameParameters<T> CalcLameParameters(const T& youngs_modulus,
                                     const T& poissons_ratio);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
