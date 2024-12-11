#include "drake/multibody/fem/corotated_model.h"

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
CorotatedModel<T>::CorotatedModel(const T& youngs_modulus,
                                  const T& poissons_ratio)
    : E_(youngs_modulus), nu_(poissons_ratio) {
  const LameParameters<T> lame_params = CalcLameParameters(E_, nu_);
  mu_ = lame_params.mu;
  lambda_ = lame_params.lambda;
}

template <typename T>
void CorotatedModel<T>::CalcElasticEnergyDensityImpl(const Data& data,
                                                     T* Psi) const {
  const T& Jm1 = data.Jm1();
  const Matrix3<T>& F = data.deformation_gradient();
  const Matrix3<T>& R = data.R();
  /* Note that ‖F − R‖² is equivalent to the ∑(σᵢ−1)² term used in
   [Stomakhin, 2012]. */
  (*Psi) = mu_ * (F - R).squaredNorm() + 0.5 * lambda_ * Jm1 * Jm1;
}

template <typename T>
void CorotatedModel<T>::CalcFirstPiolaStressImpl(const Data& data,
                                                 Matrix3<T>* P) const {
  const T& Jm1 = data.Jm1();
  const Matrix3<T>& F = data.deformation_gradient();
  const Matrix3<T>& R = data.R();
  const Matrix3<T>& JFinvT = data.JFinvT();
  (*P) = 2.0 * mu_ * (F - R) + lambda_ * Jm1 * JFinvT;
}

template <typename T>
void CorotatedModel<T>::CalcFirstPiolaStressDerivativeImpl(
    const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const {
  const T& Jm1 = data.Jm1();
  const Matrix3<T>& F = data.deformation_gradient();
  const Matrix3<T>& R = data.R();
  const Matrix3<T>& S = data.S();
  const Matrix3<T>& JFinvT = data.JFinvT();
  auto& local_dPdF = (*dPdF);
  local_dPdF.SetAsOuterProduct(lambda_ * JFinvT, JFinvT);
  /* The contribution from derivatives of F. */
  local_dPdF.mutable_data().diagonal().array() += 2.0 * mu_;
  /* The contribution from derivatives of R. */
  internal::AddScaledRotationalDerivative<T>(R, S, -2.0 * mu_, &local_dPdF);
  /* The contribution from derivatives of JFinvT. */
  internal::AddScaledCofactorMatrixDerivative<T>(F, lambda_ * Jm1, &local_dPdF);
}

template class CorotatedModel<float>;
template class CorotatedModel<double>;
template class CorotatedModel<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
