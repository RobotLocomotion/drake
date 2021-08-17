#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

#include <array>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/multibody/fixed_fem/dev/calc_lame_parameters.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T, int num_locations>
LinearConstitutiveModel<T, num_locations>::LinearConstitutiveModel(
    const T& youngs_modulus, const T& poisson_ratio)
    : E_(youngs_modulus), nu_(poisson_ratio) {
  std::tie(lambda_, mu_) = CalcLameParameters(E_, nu_);
  /* Recall that
        Pᵢⱼ = 2μ * εᵢⱼ + λ * εₐₐ * δᵢⱼ,
    So,
        ∂Pᵢⱼ/∂Fₖₗ = 2μ * ∂εᵢⱼ/∂Fₖₗ + λ * ∂εₐₐ/∂Fₖₗ * δᵢⱼ,
    Since
        ∂εᵢⱼ/∂Fₖₗ = 0.5 * δᵢₖ δⱼₗ  + 0.5 * δᵢₗ δₖⱼ.
    Plugging in, we get:
        ∂Pᵢⱼ/∂Fₖₗ = μ * (δᵢₖδⱼₗ + δᵢₗ δⱼₖ) +  λ * δₖₗ * δᵢⱼ.
    Keep in mind that the indices are laid out such that the ik-th entry in
    the jl-th block corresponds to the value dPᵢⱼ/dFₖₗ.  */
  /* First term. */
  dPdF_ = mu_ * Eigen::Matrix<T, 9, 9>::Identity();
  for (int k = 0; k < 3; ++k) {
    /* Second term. */
    for (int l = 0; l < 3; ++l) {
      const int i = l;
      const int j = k;
      dPdF_(3 * j + i, 3 * l + k) += mu_;
    }
    /* Third term. */
    for (int i = 0; i < 3; ++i) {
      const int l = k;
      const int j = i;
      dPdF_(3 * j + i, 3 * l + k) += lambda_;
    }
  }
}

template <typename T, int num_locations>
void LinearConstitutiveModel<T, num_locations>::CalcElasticEnergyDensityImpl(
    const Data& data, std::array<T, num_locations>* Psi) const {
  for (int i = 0; i < num_locations; ++i) {
    const auto& strain = data.strain()[i];
    const auto& trace_strain = data.trace_strain()[i];
    (*Psi)[i] = mu_ * strain.squaredNorm() +
                0.5 * lambda_ * trace_strain * trace_strain;
  }
}

template <typename T, int num_locations>
void LinearConstitutiveModel<T, num_locations>::CalcFirstPiolaStressImpl(
    const Data& data, std::array<Matrix3<T>, num_locations>* P) const {
  for (int i = 0; i < num_locations; ++i) {
    const auto& strain = data.strain()[i];
    const auto& trace_strain = data.trace_strain()[i];
    (*P)[i] =
        2.0 * mu_ * strain + lambda_ * trace_strain * Matrix3<T>::Identity();
  }
}

template <typename T, int num_locations>
void LinearConstitutiveModel<T, num_locations>::
    CalcFirstPiolaStressDerivativeImpl(
        const Data&,
        std::array<Eigen::Matrix<T, 9, 9>, num_locations>* dPdF) const {
  dPdF->fill(dPdF_);
}

template class LinearConstitutiveModel<double, 1>;
template class LinearConstitutiveModel<AutoDiffXd, 1>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
