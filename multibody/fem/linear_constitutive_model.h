#pragma once

#include <array>

#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/linear_constitutive_model_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Traits for LinearConstitutiveModel. */
template <typename T>
struct LinearConstitutiveModelTraits {
  using Scalar = T;
  using Data = LinearConstitutiveModelData<T>;
  static constexpr int is_linear = true;
};

/* Implements the infinitesimal-strain linear elasticity constitutive model as
 described in Section 7.4 of [Gonzalez, 2008].
 @tparam T The scalar type, can be a double, float, or AutoDiffXd.

[Gonzalez, 2008] Gonzalez, Oscar, and Andrew M. Stuart. A first course in
continuum mechanics. Cambridge University Press, 2008. */
template <typename T>
class LinearConstitutiveModel final
    : public ConstitutiveModel<LinearConstitutiveModel<T>,
                               LinearConstitutiveModelTraits<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModel);

  using Base = ConstitutiveModel<LinearConstitutiveModel<T>,
                                 LinearConstitutiveModelTraits<T>>;
  using Data = typename Base::Data;

  /* Constructs a LinearConstitutiveModel constitutive model with the
   prescribed Young's modulus and Poisson's ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poissons_ratio Poisson's ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poissons_ratio < 0.5. */
  LinearConstitutiveModel(const T& youngs_modulus, const T& poissons_ratio);

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
  /* Allow base class friend access to the private CalcFooImpl functions. */
  friend Base;

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

  /* Shadows ConstitutiveModel::CalcFilteredHessianImpl() in the CRTP base
   class. */
  void CalcFilteredHessianImpl(
      const Data& data, math::internal::FourthOrderTensor<T>* hessian) const;

  T E_;       // Young's modulus, N/m².
  T nu_;      // Poisson's ratio.
  T mu_;      // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;  // Lamé's first parameter, N/m².
  math::internal::FourthOrderTensor<T>
      dPdF_;  // The First Piola stress derivative is constant and precomputed.
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
