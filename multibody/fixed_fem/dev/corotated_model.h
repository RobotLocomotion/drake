#pragma once

#include <array>

#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/corotated_model_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Traits for CorotatedModel. */
template <typename T, int num_locations>
struct CorotatedModelTraits {
  using Scalar = T;
  using Data = CorotatedModelData<T, num_locations>;
};

/* Implements the fixed corotated hyperelastic constitutive model as
 described in [Stomakhin, 2012].
 @tparam_nonsymbolic_scalar.
 @tparam num_locations Number of locations at which the constitutive
 relationship is evaluated. We currently only provide one instantiation of this
 template with `num_locations = 1`, but more instantiations can easily be added
 when needed.

 [Stomakhin, 2012] Stomakhin, Alexey, et al. "Energetically consistent
 invertible elasticity." Proceedings of the 11th ACM SIGGRAPH/Eurographics
 conference on Computer Animation. 2012. */
template <typename T, int num_locations>
class CorotatedModel final
    : public ConstitutiveModel<CorotatedModel<T, num_locations>,
                               CorotatedModelTraits<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModel)

  using Base = ConstitutiveModel<CorotatedModel<T, num_locations>,
                                 CorotatedModelTraits<T, num_locations>>;
  using Data = typename Base::Data;

  /* Constructs a CorotatedModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus  Young's modulus of the model, with unit N/m²
   @param poisson_ratio   Poisson ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poisson_ratio < 0.5. */
  CorotatedModel(const T& youngs_modulus, const T& poisson_ratio);

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
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
