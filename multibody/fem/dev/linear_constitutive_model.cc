#include "drake/multibody/fem/dev/linear_constitutive_model.h"

#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
LinearConstitutiveModel<T>::LinearConstitutiveModel(const T& youngs_modulus,
                                                    const T& poisson_ratio)
    : E_(youngs_modulus), nu_(poisson_ratio) {
  VerifyParameterValidity(E_, nu_);
  SetLameParameters(E_, nu_);
  /* Recall that
        Pᵢⱼ = 2μ * εᵢⱼ + λ * εₐₐ * δᵢⱼ,
    So,
        ∂Pᵢⱼ/∂Fₖₗ = 2μ * ∂εᵢⱼ/∂Fₖₗ + λ * ∂εₐₐ/∂Fₖₗ * δᵢⱼ,
    Since
        ∂εᵢⱼ/∂Fₖₗ = 0.5 * δᵢₖ δⱼₗ  + 0.5 * δᵢₗ δₖⱼ.
    Plugging in, we get:
        ∂Pᵢⱼ/∂Fₖₗ = μ * (δᵢₖδⱼₗ + δᵢₗ δⱼₖ) +  λ * δₖₗ * δᵢⱼ.
    Keep in mind that the indices are laid out such that the ik-th entry in the
    jl-th block corresponds to the value dPᵢⱼ/dFₖₗ.  */
  // First term.
  dPdF_ = mu_ * Eigen::Matrix<T, 9, 9>::Identity();
  for (int k = 0; k < 3; ++k) {
    // Second term.
    for (int l = 0; l < 3; ++l) {
      const int i = l;
      const int j = k;
      dPdF_(3 * j + i, 3 * l + k) += mu_;
    }
    // Third term.
    for (int i = 0; i < 3; ++i) {
      const int l = k;
      const int j = i;
      dPdF_(3 * j + i, 3 * l + k) += lambda_;
    }
  }
}

template <typename T>
void LinearConstitutiveModel<T>::DoCalcElasticEnergyDensity(
    const DeformationGradientCacheEntry<T>& cache_entry,
    std::vector<T>* Psi) const {
  const LinearConstitutiveModelCacheEntry<T>& linear_cache_entry =
      static_cast<const LinearConstitutiveModelCacheEntry<T>&>(cache_entry);
  for (int i = 0; i < linear_cache_entry.num_quadrature_points(); ++i) {
    const auto& strain = linear_cache_entry.strain()[i];
    const auto& trace_strain = linear_cache_entry.trace_strain()[i];
    (*Psi)[i] = mu_ * strain.squaredNorm() +
                0.5 * lambda_ * trace_strain * trace_strain;
  }
}

template <typename T>
void LinearConstitutiveModel<T>::DoCalcFirstPiolaStress(
    const DeformationGradientCacheEntry<T>& cache_entry,
    std::vector<Matrix3<T>>* P) const {
  const LinearConstitutiveModelCacheEntry<T>& linear_cache_entry =
      static_cast<const LinearConstitutiveModelCacheEntry<T>&>(cache_entry);
  for (int i = 0; i < linear_cache_entry.num_quadrature_points(); ++i) {
    const auto& strain = linear_cache_entry.strain()[i];
    const auto& trace_strain = linear_cache_entry.trace_strain()[i];
    (*P)[i] =
        2.0 * mu_ * strain + lambda_ * trace_strain * Matrix3<T>::Identity();
  }
}

template <typename T>
void LinearConstitutiveModel<T>::DoCalcFirstPiolaStressDerivative(
    const DeformationGradientCacheEntry<T>& cache_entry,
    std::vector<Eigen::Matrix<T, 9, 9>>* dPdF) const {
  unused(cache_entry);
  std::fill(dPdF->begin(), dPdF->end(), dPdF_);
}

template <typename T>
std::unique_ptr<DeformationGradientCacheEntry<T>>
LinearConstitutiveModel<T>::DoMakeDeformationGradientCacheEntry(
    ElementIndex element_index, int num_quadrature_points) const {
  return std::make_unique<LinearConstitutiveModelCacheEntry<T>>(
      element_index, num_quadrature_points);
}

template <typename T>
void LinearConstitutiveModel<T>::VerifyParameterValidity(
    const T& youngs_modulus, const T& poisson_ratio) const {
  if (youngs_modulus < 0.0) {
    throw std::logic_error("Young's modulus must be nonnegative.");
  }
  if (poisson_ratio >= 0.5 || poisson_ratio <= -1) {
    throw std::logic_error("Poisson ratio must be in (-1, 0.5).");
  }
}

template <typename T>
void LinearConstitutiveModel<T>::SetLameParameters(const T& youngs_modulus,
                                                   const T& poisson_ratio) {
  mu_ = youngs_modulus / (2.0 * (1.0 + poisson_ratio));
  lambda_ = youngs_modulus * poisson_ratio /
            ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::LinearConstitutiveModel);
