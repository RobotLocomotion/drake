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

template <typename T, int num_locations>
CorotatedModel<T, num_locations>::CorotatedModel(const T& youngs_modulus,
                                                 const T& poissons_ratio)
    : E_(youngs_modulus), nu_(poissons_ratio) {
  const LameParameters<T> lame_params = CalcLameParameters(E_, nu_);
  mu_ = lame_params.mu;
  lambda_ = lame_params.lambda;
}

template <typename T, int num_locations>
void CorotatedModel<T, num_locations>::CalcElasticEnergyDensityImpl(
    const Data& data, std::array<T, num_locations>* Psi) const {
  for (int i = 0; i < num_locations; ++i) {
    const T& Jm1 = data.Jm1()[i];
    const Matrix3<T>& F = data.deformation_gradient()[i];
    const Matrix3<T>& R = data.R()[i];
    /* Note that ‖F − R‖² is equivalent to the ∑(σᵢ−1)² term used in
     [Stomakhin, 2012]. */
    (*Psi)[i] = mu_ * (F - R).squaredNorm() + 0.5 * lambda_ * Jm1 * Jm1;
  }
}

template <typename T, int num_locations>
void CorotatedModel<T, num_locations>::CalcFirstPiolaStressImpl(
    const Data& data, std::array<Matrix3<T>, num_locations>* P) const {
  for (int i = 0; i < num_locations; ++i) {
    const T& Jm1 = data.Jm1()[i];
    const Matrix3<T>& F = data.deformation_gradient()[i];
    const Matrix3<T>& R = data.R()[i];
    const Matrix3<T>& JFinvT = data.JFinvT()[i];
    (*P)[i] = 2.0 * mu_ * (F - R) + lambda_ * Jm1 * JFinvT;
  }
}

template <typename T, int num_locations>
void CorotatedModel<T, num_locations>::CalcFirstPiolaStressDerivativeImpl(
    const Data& data,
    std::array<Eigen::Matrix<T, 9, 9>, num_locations>* dPdF) const {
  for (int i = 0; i < num_locations; ++i) {
    const T& Jm1 = data.Jm1()[i];
    const Matrix3<T>& F = data.deformation_gradient()[i];
    const Matrix3<T>& R = data.R()[i];
    const Matrix3<T>& S = data.S()[i];
    const Matrix3<T>& JFinvT = data.JFinvT()[i];
    const Vector<T, 3 * 3>& flat_JFinvT =
        Eigen::Map<const Vector<T, 3 * 3>>(JFinvT.data(), 3 * 3);
    auto& local_dPdF = (*dPdF)[i];
    /* The contribution from derivatives of Jm1. */
    local_dPdF.noalias() = lambda_ * flat_JFinvT * flat_JFinvT.transpose();
    /* The contribution from derivatives of F. */
    local_dPdF.diagonal().array() += 2.0 * mu_;
    /* The contribution from derivatives of R. */
    internal::AddScaledRotationalDerivative<T>(R, S, -2.0 * mu_, &local_dPdF);
    /* The contribution from derivatives of JFinvT. */
    internal::AddScaledCofactorMatrixDerivative<T>(F, lambda_ * Jm1,
                                                   &local_dPdF);
  }
}

template class CorotatedModel<double, 1>;
template class CorotatedModel<AutoDiffXd, 1>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
