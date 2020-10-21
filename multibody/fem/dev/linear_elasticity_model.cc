#include "drake/multibody/fem/dev/linear_elasticity_model.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
LinearElasticityModel<T>::LinearElasticityModel(const T& youngs_modulus,
                                                const T& poisson_ratio)
    : E_(youngs_modulus), nu_(poisson_ratio) {
  VerifyParameterValidity(E_, nu_);
  SetLameParameters(E_, nu_);
}

template <typename T>
void LinearElasticityModel<T>::DoCalcElasticEnergyDensity(
    const DeformationGradientCache<T>& cache, std::vector<T>* Psi) const {
  const LinearElasticityModelCache<T>& linear_cache =
      static_cast<const LinearElasticityModelCache<T>&>(cache);
  for (int i = 0; i < linear_cache.num_quads(); ++i) {
    const auto& strain = linear_cache.strain()[i];
    const auto& trace_strain = linear_cache.trace_strain()[i];
    (*Psi)[i] = mu_ * strain.squaredNorm() +
                0.5 * lambda_ * trace_strain * trace_strain;
  }
}

template <typename T>
void LinearElasticityModel<T>::DoCalcFirstPiolaStress(
    const DeformationGradientCache<T>& cache,
    std::vector<Matrix3<T>>* P) const {
  const LinearElasticityModelCache<T>& linear_cache =
      static_cast<const LinearElasticityModelCache<T>&>(cache);
  for (int i = 0; i < linear_cache.num_quads(); ++i) {
    const auto& strain = linear_cache.strain()[i];
    const auto& trace_strain = linear_cache.trace_strain()[i];
    (*P)[i] =
        2.0 * mu_ * strain + lambda_ * trace_strain * Matrix3<T>::Identity();
  }
}

template <typename T>
void LinearElasticityModel<T>::VerifyParameterValidity(
    const T& youngs_modulus, const T& poisson_ratio) const {
  if (youngs_modulus < 0.0) {
    throw std::logic_error("Young's modulus must be nonnegative.");
  }
  if (poisson_ratio >= 0.5 || poisson_ratio <= -1) {
    throw std::logic_error("Poisson ratio must be in (-1, 0.5).");
  }
}

template <typename T>
void LinearElasticityModel<T>::SetLameParameters(const T& youngs_modulus,
                                                 const T& poisson_ratio) {
  mu_ = youngs_modulus / (2.0 * (1.0 + poisson_ratio));
  lambda_ = youngs_modulus * poisson_ratio /
            ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::LinearElasticityModel);
