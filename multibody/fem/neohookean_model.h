#pragma once

#include <array>

#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/neohookean_model_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Traits for NeoHookeanModel. */
template <typename T>
struct NeoHookeanModelTraits {
  using Scalar = T;
  using Data = NeoHookeanModelData<T>;
  static constexpr int is_linear = false;
};

/* Implements the stable Neo-Hookean hyperelastic constitutive model as
 described in [Smith et al., 2019]. The implementation references Section 7.3 in
 the course note [Kim and Eberle, 2020].
 @tparam T The scalar type, can be a double, float, or AutoDiffXd.

 [Smith et al., 2019] Smith, Breannan, Fernando De Goes, and Theodore Kim.
 "Stable Neo-Hookean flesh simulation." ACM Transactions on Graphics (TOG) 37.2
 (2018): 1-15.
 [Kim and Eberle, 2020] Kim, Theodore, and David Eberle. "Dynamic deformables:
 implementation and production practicalities." Acm siggraph 2020 courses. 2020.
 1-182. */
template <typename T>
class NeoHookeanModel final
    : public ConstitutiveModel<NeoHookeanModel<T>, NeoHookeanModelTraits<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NeoHookeanModel);

  using Traits = NeoHookeanModelTraits<T>;
  using Data = typename Traits::Data;

  /* Constructs a NeoHookeanModel constitutive model with the
   prescribed Young's modulus and Poisson's ratio.
   @param youngs_modulus  Young's modulus of the model, with units of N/m².
   @param poissons_ratio  Poisson's ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poissons_ratio < 0.5. */
  NeoHookeanModel(const T& youngs_modulus, const T& poissons_ratio);

  const T& youngs_modulus() const { return E_; }

  const T& poissons_ratio() const { return nu_; }

  /* Returns the shear modulus (Lamé's second parameter) which is given by
   `E/(2*(1+nu))` where `E` is the Young's modulus and `nu` is the Poisson's
   ratio. See `fem::internal::CalcLameParameters()`. */
  const T& shear_modulus() const { return mu_; }

  /* Returns the Lamé's first parameter which is given by
   `E*nu/((1+nu)*(1-2*nu))` where `E` is the Young's modulus and `nu` is the
   Poisson's ratio. See `fem::internal::CalcLameParameters()`. */
  const T& lame_first_parameter() const { return lambda_; }

 private:
  friend ConstitutiveModel<NeoHookeanModel<T>, NeoHookeanModelTraits<T>>;

  /* Shadows ConstitutiveModel::CalcElasticEnergyDensityImpl() as required by
   the CRTP base class. */
  void CalcElasticEnergyDensityImpl(const Data& data, T* Psi) const;

  /* Shadows ConstitutiveModel::CalcFirstPiolaStressImpl() as required by the
   CRTP base class. */
  void CalcFirstPiolaStressImpl(const Data& data, Matrix3<T>* P) const;

  /* Shadows ConstitutiveModel::CalcFirstPiolaStressDerivativeImpl() as required
   by the CRTP base class. */
  void CalcFirstPiolaStressDerivativeImpl(
      const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const;

  /* Shadows ConstitutiveModel::CalcFilteredHessian() in the base class to
   provide a more efficient implementation. */
  void CalcFilteredHessianImpl(
      const Data& data, math::internal::FourthOrderTensor<T>* hessian) const;

  /* Given the rotation matrices in the SVD of F, computes the "twist" and
   "flip" eigenvectors (3 of each) and write them into the first 6 columns of
   the eigenvector matrix Q described in [Kim and Eberle, 2020]. */
  void BuildTwistAndFlipEigenvectors(const Matrix3<T>& U, const Matrix3<T>& V,
                                     Eigen::Matrix<T, 9, 9>* Q) const;

  T E_;               // Young's modulus, N/m².
  T nu_;              // Poisson's ratio.
  T mu_;              // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;          // Lamé's first parameter, N/m².
  T mu_over_lambda_;  // mu/lambda, unitless.
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
