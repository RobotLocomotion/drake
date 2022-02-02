#pragma once

#include <array>

#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/linear_constitutive_model_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Traits for LinearConstitutiveModel. */
template <typename T, int num_locations>
struct LinearConstitutiveModelTraits {
  using Scalar = T;
  using Data = LinearConstitutiveModelData<T, num_locations>;
};

/* Implements the infinitesimal-strain linear elasticity constitutive model as
 described in Section 7.4 of [Gonzalez, 2008].
 @tparam_nonsymbolic_scalar.
 @tparam num_locations Number of locations at which the constitutive
 relationship is evaluated. We currently only provide one instantiation of this
 template with `num_locations = 1`, but more instantiations can easily be added
 when needed.

[Gonzalez, 2008] Gonzalez, Oscar, and Andrew M. Stuart. A first course in
continuum mechanics. Cambridge University Press, 2008. */
template <typename T, int num_locations>
class LinearConstitutiveModel final
    : public ConstitutiveModel<
          LinearConstitutiveModel<T, num_locations>,
          LinearConstitutiveModelTraits<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModel)

  using Base =
      ConstitutiveModel<LinearConstitutiveModel<T, num_locations>,
                        LinearConstitutiveModelTraits<T, num_locations>>;
  using Data = typename Base::Data;

  /* Constructs a LinearConstitutiveModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poisson_ratio Poisson ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poisson_ratio < 0.5. */
  LinearConstitutiveModel(const T& youngs_modulus, const T& poisson_ratio);

  const T& youngs_modulus() const { return E_; }

  const T& poisson_ratio() const { return nu_; }

  /* Returns the shear modulus (Lame's second parameter) which is given by
   `E/(2*(1+nu))` where `E` is the Young's modulus and `nu` is the Poisson
   ratio. See `fem::internal::CalcLameParameters()`. */
  const T& shear_modulus() const { return mu_; }

  /* Returns the Lame's first parameter which is given by
   `E*nu/((1+nu)*(1-2*nu))` where `E` is the Young's modulus and `nu` is the
   Poisson ratio. See `fem::internal::CalcLameParameters()`. */
  const T& lame_first_parameter() const { return lambda_; }

 private:
  friend Base;

  /* Shadows ConstitutiveModel::CalcElasticEnergyDensityImpl() as required by
   the CRTP base class. */
  void CalcElasticEnergyDensityImpl(const Data& data,
                                    std::array<T, num_locations>* Psi) const;

  /* Shadows ConstitutiveModel::CalcFirstPiolaStressImpl() as required by the
   CRTP base class. */
  void CalcFirstPiolaStressImpl(const Data& data,
                                std::array<Matrix3<T>, num_locations>* P) const;

  /* Shadows ConstitutiveModel::CalcFirstPiolaStressDerivativeImpl() as required
   by the CRTP base class. */
  void CalcFirstPiolaStressDerivativeImpl(
      const Data& data,
      std::array<Eigen::Matrix<T, 9, 9>, num_locations>* dPdF) const;

  T E_;       // Young's modulus, N/m².
  T nu_;      // Poisson ratio.
  T mu_;      // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;  // Lamé's first parameter, N/m².
  Eigen::Matrix<T, 9, 9>
      dPdF_;  // The First Piola stress derivative is constant and precomputed.
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
