#include "drake/multibody/fem/neohookean_model.h"

#include <array>
#include <numbers>
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
  /* Note that we drop the constant term from
   ΨE = (μ/2)(IC − 3) − μ(J − 1) + (λ/2)(J − 1)² as done in (6.9) in
   [Kim and Eberle, 2020]. */
  *Psi = 0.5 *
         (mu_ * (Ic - 3.0) + lambda_ * volume_correction * volume_correction);
}

template <typename T>
void NeoHookeanModel<T>::CalcFirstPiolaStressImpl(const Data& data,
                                                  Matrix3<T>* P) const {
  const Matrix3<T>& dJdF = data.dJdF();
  const T& volume_correction = data.Jm1() - mu_over_lambda_;
  const Matrix3<T>& F = data.deformation_gradient();
  /* P(F) = μF + λ(J−α)∂J/∂F  */
  (*P) = mu_ * F + lambda_ * volume_correction * dJdF;
}

template <typename T>
void NeoHookeanModel<T>::CalcFirstPiolaStressDerivativeImpl(
    const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const {
  /* We need to differentiate P(F) = μF + λ(J−α)∂J/∂F w.r.t. F. The
   derivatives are going to hit F, J and ∂J/∂F = JF⁻ᵀ respectively. */
  const T& Jm1 = data.Jm1();
  const T& volume_correction = Jm1 - mu_over_lambda_;
  const Matrix3<T>& F = data.deformation_gradient();
  const Matrix3<T>& JFinvT = data.dJdF();
  auto& dPdF_output = (*dPdF);
  /* The contribution from derivatives of J. */
  dPdF_output.SetAsOuterProduct(lambda_ * JFinvT, JFinvT);
  /* The contribution from derivatives of F. */
  dPdF_output.mutable_data().diagonal().array() += mu_;
  /* The contribution from derivatives of JF⁻ᵀ. */
  internal::AddScaledCofactorMatrixDerivative<T>(F, lambda_ * volume_correction,
                                                 &dPdF_output);
}

template <typename T>
void NeoHookeanModel<T>::CalcFilteredHessianImpl(
    const Data& data, math::internal::FourthOrderTensor<T>* hessian) const {
  /* The implementation follows the notes in section 7.3 and 7.4 in
  [Kim and Eberle, 2020]. The eigenvectors are spelled out in equations
  (7.10-7.13); the eigenvalues are spelled out in equations (7.34-7.39) and the
  two equations above 7.34. */
  using Matrix9T = Eigen::Matrix<T, 9, 9>;
  using Vector9T = Eigen::Matrix<T, 9, 1>;

  Vector9T eigenvalues;
  Matrix9T eigenvectors;

  const T& Jm1 = data.Jm1();
  const T J = Jm1 + 1.0;
  const Vector3<T>& sigma = data.sigma();
  const Matrix3<T>& U = data.U();
  const Matrix3<T>& V = data.V();

  /* Compute the twist and flip eigenvalues. */
  {
    /* Twist eigenvalues */
    eigenvalues.template segment<3>(0) = sigma;
    /* Flip eigenvalues */
    eigenvalues.template segment<3>(3) = -sigma;
    const T scale = lambda_ * data.Jm1() - mu_;
    eigenvalues.template segment<6>(0) *= scale;
    eigenvalues.template segment<6>(0).array() += mu_;
  }

  /* Compute the twist and flip eigenvectors. */
  BuildTwistAndFlipEigenvectors(U, V, &eigenvectors);

  /* Compute the remaining three eigenvalues and eigenvectors. */
  {
    Matrix3<T> A;
    const T s0s0 = sigma(0) * sigma(0);
    const T s1s1 = sigma(1) * sigma(1);
    const T s2s2 = sigma(2) * sigma(2);
    A(0, 0) = mu_ + lambda_ * s1s1 * s2s2;
    A(1, 1) = mu_ + lambda_ * s0s0 * s2s2;
    A(2, 2) = mu_ + lambda_ * s0s0 * s1s1;
    const T scale = lambda_ * (2.0 * J - 1.0) - mu_;
    A(0, 1) = scale * sigma(2);
    A(1, 0) = A(0, 1);
    A(0, 2) = scale * sigma(1);
    A(2, 0) = A(0, 2);
    A(1, 2) = scale * sigma(0);
    A(2, 1) = A(1, 2);

    const Eigen::SelfAdjointEigenSolver<Matrix3<T>> solver(A);
    eigenvalues.template segment<3>(6) = solver.eigenvalues();

    Eigen::Map<Matrix3<T>>(eigenvectors.data() + 54).noalias() =
        U * solver.eigenvectors().col(0).asDiagonal() * V.transpose();
    Eigen::Map<Matrix3<T>>(eigenvectors.data() + 63).noalias() =
        U * solver.eigenvectors().col(1).asDiagonal() * V.transpose();
    Eigen::Map<Matrix3<T>>(eigenvectors.data() + 72).noalias() =
        U * solver.eigenvectors().col(2).asDiagonal() * V.transpose();
  }

  /* An arbitrary small positive to guarantee that the resulting Hessian is
   positive semidefinite (even with numerical rounding). In practice, the
   assembled Hessian is always positive-definite (SPD) (due to the SPD mass
   matrix). */
  const T kTol = 1e-14;
  /* Clamp the eigenvalues. */
  for (int i = 0; i < 9; ++i) {
    if (eigenvalues(i) < kTol) {
      eigenvalues(i) = kTol;
    }
  }
  hessian->mutable_data().noalias() =
      eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

template <typename T>
void NeoHookeanModel<T>::BuildTwistAndFlipEigenvectors(
    const Matrix3<T>& U, const Matrix3<T>& V, Eigen::Matrix<T, 9, 9>* Q) const {
  const T scale = 1.0 / std::numbers::sqrt2;
  const Matrix3<T> sV = scale * V;
  /* The eigenvectors are spelled out in equations (7.10-7.13) in
  [Kim and Eberle, 2020]. */
  Matrix3<T> A;
  A << sV(0, 2) * U(0, 1), sV(1, 2) * U(0, 1), sV(2, 2) * U(0, 1),
      sV(0, 2) * U(1, 1), sV(1, 2) * U(1, 1), sV(2, 2) * U(1, 1),
      sV(0, 2) * U(2, 1), sV(1, 2) * U(2, 1), sV(2, 2) * U(2, 1);

  Matrix3<T> B;
  B << sV(0, 1) * U(0, 2), sV(1, 1) * U(0, 2), sV(2, 1) * U(0, 2),
      sV(0, 1) * U(1, 2), sV(1, 1) * U(1, 2), sV(2, 1) * U(1, 2),
      sV(0, 1) * U(2, 2), sV(1, 1) * U(2, 2), sV(2, 1) * U(2, 2);

  Matrix3<T> C;
  C << sV(0, 2) * U(0, 0), sV(1, 2) * U(0, 0), sV(2, 2) * U(0, 0),
      sV(0, 2) * U(1, 0), sV(1, 2) * U(1, 0), sV(2, 2) * U(1, 0),
      sV(0, 2) * U(2, 0), sV(1, 2) * U(2, 0), sV(2, 2) * U(2, 0);

  Matrix3<T> D;
  D << sV(0, 0) * U(0, 2), sV(1, 0) * U(0, 2), sV(2, 0) * U(0, 2),
      sV(0, 0) * U(1, 2), sV(1, 0) * U(1, 2), sV(2, 0) * U(1, 2),
      sV(0, 0) * U(2, 2), sV(1, 0) * U(2, 2), sV(2, 0) * U(2, 2);

  Matrix3<T> E;
  E << sV(0, 1) * U(0, 0), sV(1, 1) * U(0, 0), sV(2, 1) * U(0, 0),
      sV(0, 1) * U(1, 0), sV(1, 1) * U(1, 0), sV(2, 1) * U(1, 0),
      sV(0, 1) * U(2, 0), sV(1, 1) * U(2, 0), sV(2, 1) * U(2, 0);

  Matrix3<T> F;
  F << sV(0, 0) * U(0, 1), sV(1, 0) * U(0, 1), sV(2, 0) * U(0, 1),
      sV(0, 0) * U(1, 1), sV(1, 0) * U(1, 1), sV(2, 0) * U(1, 1),
      sV(0, 0) * U(2, 1), sV(1, 0) * U(2, 1), sV(2, 0) * U(2, 1);

  /* Twist eigenvectors */
  Eigen::Map<Matrix3<T>>(Q->data()) = B - A;
  Eigen::Map<Matrix3<T>>(Q->data() + 9) = D - C;
  Eigen::Map<Matrix3<T>>(Q->data() + 18) = F - E;

  /* Flip eigenvectors */
  Eigen::Map<Matrix3<T>>(Q->data() + 27) = A + B;
  Eigen::Map<Matrix3<T>>(Q->data() + 36) = C + D;
  Eigen::Map<Matrix3<T>>(Q->data() + 45) = E + F;
}

template class NeoHookeanModel<float>;
template class NeoHookeanModel<double>;
template class NeoHookeanModel<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
