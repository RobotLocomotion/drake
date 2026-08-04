#include "drake/multibody/fem/linear_constitutive_model.h"

#include <array>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/calc_lame_parameters.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
LinearConstitutiveModel<T>::LinearConstitutiveModel(const T& youngs_modulus,
                                                    const T& poissons_ratio)
    : E_(youngs_modulus), nu_(poissons_ratio) {
  const LameParameters<T> lame_params = CalcLameParameters(E_, nu_);
  mu_ = lame_params.mu;
  lambda_ = lame_params.lambda;
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
  dPdF_.SetAsOuterProduct(Matrix3<T>::Identity(),
                          lambda_ * Matrix3<T>::Identity());
  dPdF_ +=
      math::internal::FourthOrderTensor<T>::MakeSymmetricIdentity(2.0 * mu_);
}

template <typename T>
void LinearConstitutiveModel<T>::CalcElasticEnergyDensityImpl(const Data& data,
                                                              T* Psi) const {
  const auto& strain = data.strain();
  const auto& trace_strain = data.trace_strain();
  (*Psi) =
      mu_ * strain.squaredNorm() + 0.5 * lambda_ * trace_strain * trace_strain;
}

template <typename T>
void LinearConstitutiveModel<T>::CalcFirstPiolaStressImpl(const Data& data,
                                                          Matrix3<T>* P) const {
  const auto& strain = data.strain();
  const auto& trace_strain = data.trace_strain();
  (*P) = 2.0 * mu_ * strain + lambda_ * trace_strain * Matrix3<T>::Identity();
}

template <typename T>
void LinearConstitutiveModel<T>::CalcFirstPiolaStressDerivativeImpl(
    const Data&, math::internal::FourthOrderTensor<T>* dPdF) const {
  *dPdF = dPdF_;
}

template <typename T>
void LinearConstitutiveModel<T>::CalcFilteredHessianImpl(
    const Data&, math::internal::FourthOrderTensor<T>* hessian) const {
  *hessian = dPdF_;
}

template class LinearConstitutiveModel<float>;
template class LinearConstitutiveModel<double>;
template class LinearConstitutiveModel<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
