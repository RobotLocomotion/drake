#include "drake/multibody/fem/calc_lame_parameters.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
LameParameters<T> CalcLameParameters(const T& youngs_modulus,
                                     const T& poissons_ratio) {
  if (!(youngs_modulus >= 0.0)) {
    throw std::logic_error("Young's modulus must be nonnegative.");
  }
  if (!(poissons_ratio < 0.5 && poissons_ratio > -1)) {
    throw std::logic_error("Poisson's ratio must be in (-1, 0.5).");
  }
  const T mu = youngs_modulus / (2.0 * (1.0 + poissons_ratio));
  const T lambda = youngs_modulus * poissons_ratio /
                   ((1.0 + poissons_ratio) * (1.0 - 2.0 * poissons_ratio));
  return {lambda, mu};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcLameParameters<T>));
template LameParameters<float> CalcLameParameters<float>(const float&,
                                                         const float&);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
