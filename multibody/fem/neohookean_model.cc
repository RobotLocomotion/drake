#include "drake/multibody/fem/neohookean_model.h"

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
NeoHookeanModel<T>::NeoHookeanModel(const T& youngs_modulus,
                                    const T& poissons_ratio)
    : E_(youngs_modulus), nu_(poissons_ratio) {
  const LameParameters<T> lame_params = CalcLameParameters(E_, nu_);
  mu_ = lame_params.mu;
  lambda_ = lame_params.lambda;
  mu_over_lambda_ = mu_ / lambda_;
}

template <typename T>
void NeoHookeanModel<T>::CalcElasticEnergyDensityImpl(const Data& data,
                                                      T* Psi) const {
  const T& Ic = data.Ic();
  const T& volume_correction = data.Jm1() - mu_over_lambda_;
  *Psi = 0.5 *
         (mu_ * (Ic - 3.0) + lambda_ * volume_correction * volume_correction);
}

template <typename T>
void NeoHookeanModel<T>::CalcFirstPiolaStressImpl(const Data& data,
                                                  Matrix3<T>* P) const {
  const Matrix3<T>& dJdF = data.dJdF();
  const T& volume_correction = data.Jm1() - mu_over_lambda_;
  const Matrix3<T>& F = data.deformation_gradient();
  (*P) = mu_ * F + lambda_ * volume_correction * dJdF;
}

template <typename T>
void NeoHookeanModel<T>::CalcFirstPiolaStressDerivativeImpl(
    const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const {
  const T& Jm1 = data.Jm1();
  const T& volume_correction = Jm1 - mu_over_lambda_;
  const Matrix3<T>& F = data.deformation_gradient();
  const Matrix3<T>& JFinvT = data.dJdF();
  auto& local_dPdF = (*dPdF);
  local_dPdF.SetAsOuterProduct(lambda_ * JFinvT, JFinvT);
  /* The contribution from derivatives of F. */
  local_dPdF.mutable_data().diagonal().array() += mu_;
  /* The contribution from derivatives of JFinvT. */
  internal::AddScaledCofactorMatrixDerivative<T>(F, lambda_ * volume_correction,
                                                 &local_dPdF);
}

template class NeoHookeanModel<float>;
template class NeoHookeanModel<double>;
template class NeoHookeanModel<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
