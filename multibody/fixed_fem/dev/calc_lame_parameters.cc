#include "drake/multibody/fixed_fem/dev/calc_lame_parameters.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

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

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcLameParameters<T>))

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
