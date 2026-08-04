#include "drake/multibody/fem/linear_corotated_model.h"

#include <array>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/calc_lame_parameters.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
LinearCorotatedModel<T>::LinearCorotatedModel(const T& youngs_modulus,
                                              const T& poissons_ratio)
    : E_(youngs_modulus), nu_(poissons_ratio) {
  const LameParameters<T> lame_params = CalcLameParameters(E_, nu_);
  mu_ = lame_params.mu;
  lambda_ = lame_params.lambda;
}

template <typename T>
void LinearCorotatedModel<T>::CalcElasticEnergyDensityImpl(const Data& data,
                                                           T* Psi) const {
  const Matrix3<T>& strain = data.strain();
  const T& trace_strain = data.trace_strain();
  (*Psi) =
      mu_ * strain.squaredNorm() + 0.5 * lambda_ * trace_strain * trace_strain;
}

/* First Piola stress P is the derivative of energy density
     ϕ =  μ * ε : ε + λ/2 * tr(ε)²
 with respect to deformation gradient F. In Einstein notation, we have
     εᵢⱼ = 0.5 RₖᵢFₖⱼ + 0.5 FₖᵢRₖⱼ - δᵢⱼ.
 Differentiating w.r.t. F gives:
     Pₐᵦ = 2μ * εᵢⱼ * ∂εᵢⱼ/∂Fₐᵦ + λ * εⱼⱼ ∂εᵢᵢ/∂Fₐᵦ.
 Differentiate ε w.r.t. F gives:
     ∂εᵢⱼ/∂Fₐᵦ = 0.5 Rₐᵢ δⱼᵦ  + 0.5 δᵢᵦ Rₐⱼ,
 plug it into the expression for P results in:
     2μ * Rₐᵢ*εᵢᵦ + λ * εⱼⱼ * Rₐᵦ,
 which simplifies to
     2μRε + λtr(ε)R. */
template <typename T>
void LinearCorotatedModel<T>::CalcFirstPiolaStressImpl(const Data& data,
                                                       Matrix3<T>* P) const {
  const Matrix3<T>& R0 = data.R0();
  const Matrix3<T>& strain = data.strain();
  const T& trace_strain = data.trace_strain();
  (*P) = 2.0 * mu_ * R0 * strain + lambda_ * trace_strain * R0;
}

/*
 Calculates ∂Pᵢⱼ/∂Fₐᵦ, where
     Pᵢⱼ = 2μ * Rᵢₖ * εₖⱼ + λ * εₖₖ * Rᵢⱼ,
 So,
     ∂Pᵢⱼ/∂Fₐᵦ = 2μ * Rᵢₖ * ∂εₖⱼ/∂Fₐᵦ + λ * ∂εₖₖ/∂Fₐᵦ * Rᵢⱼ,
 We use the calculation result from the calculation result documented in
 CalcFirstPiolaStressImpl that
     ∂εᵢⱼ/∂Fₐᵦ = 0.5 Rₐᵢ δⱼᵦ  + 0.5 δᵢᵦ Rₐⱼ.
 Plugging in, we get
    ∂Pᵢⱼ/∂Fₐᵦ = 2μ * Rᵢₖ * (0.5 Rₐₖ δⱼᵦ  + 0.5 δₖᵦ Rₐⱼ)
                + λ * (0.5 Rₐₖ δₖᵦ  + 0.5 δₖᵦ Rₐₖ) * Rᵢⱼ,
 which simplifies to:
     ∂Pᵢⱼ/∂Fₐᵦ = μ * (δₐᵢδⱼᵦ + Rᵢᵦ Rₐⱼ) +  λ * Rₐᵦ * Rᵢⱼ.
 Keep in mind that the indices are laid out as following:
                  β = 1       β = 2       β = 3
              -------------------------------------
              |           |           |           |
    j = 1     |   Aᵢ₁ₐ₁   |   Aᵢ₁ₐ₂   |   Aᵢ₁ₐ₃   |
              |           |           |           |
              -------------------------------------
              |           |           |           |
    j = 2     |   Aᵢ₂ₐ₁   |   Aᵢ₂ₐ₂   |   Aᵢ₂ₐ₃   |
              |           |           |           |
              -------------------------------------
              |           |           |           |
    j = 3     |   Aᵢ₃ₐ₁   |   Aᵢ₃ₐ₂   |   Aᵢ₃ₐ₃   |
              |           |           |           |
              -------------------------------------
*/
template <typename T>
void LinearCorotatedModel<T>::CalcFirstPiolaStressDerivativeImpl(
    const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const {
  const Matrix3<T>& R0 = data.R0();
  auto& local_dPdF = (*dPdF);
  /* Add in μ * δₐᵢδⱼᵦ. */
  local_dPdF = math::internal::FourthOrderTensor<T>::MakeMajorIdentity(mu_);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int alpha = 0; alpha < 3; ++alpha) {
        for (int beta = 0; beta < 3; ++beta) {
          /* Add in  μ *  Rᵢᵦ Rₐⱼ +   λ * Rₐᵦ * Rᵢⱼ. */
          local_dPdF.mutable_data()(3 * j + i, 3 * beta + alpha) +=
              mu_ * R0(i, beta) * R0(alpha, j) +
              lambda_ * R0(alpha, beta) * R0(i, j);
        }
      }
    }
  }
}

template <typename T>
void LinearCorotatedModel<T>::CalcFilteredHessianImpl(
    const Data& data, math::internal::FourthOrderTensor<T>* hessian) const {
  CalcFirstPiolaStressDerivativeImpl(data, hessian);
}

template class LinearCorotatedModel<float>;
template class LinearCorotatedModel<double>;
template class LinearCorotatedModel<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
