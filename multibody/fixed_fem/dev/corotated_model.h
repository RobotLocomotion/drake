#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"
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
 @tparam_nonsymbolic_scalar T.

 [Stomakhin, 2012] Stomakhin, Alexey, et al. "Energetically consistent
 invertible elasticity." Proceedings of the 11th ACM SIGGRAPH/Eurographics
 conference on Computer Animation. 2012. */
template <typename T, int num_locations>
class CorotatedModel final
    : public ConstitutiveModel<CorotatedModel<T, num_locations>,
                               CorotatedModelTraits<T, num_locations>> {
 public:
  using Traits = CorotatedModelTraits<T, num_locations>;
  using Data = typename Traits::Data;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModel)

  /* Constructs a CorotatedModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus  Young's modulus of the model, with unit N/m²
   @param poisson_ratio   Poisson ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poisson_ratio < 0.5. */
  CorotatedModel(const T& youngs_modulus, const T& poisson_ratio);

  const T& youngs_modulus() const { return E_; }

  const T& poisson_ratio() const { return nu_; }

  const T& shear_modulus() const { return mu_; }

  const T& lame_first_parameter() const { return lambda_; }

 private:
  using Base = ConstitutiveModel<CorotatedModel<T, num_locations>, Traits>;
  friend Base;

  /* Shadows ConstitutiveModel::DoCalcElasticEnergyDensity() as required by the
   CRTP base class. */
  void DoCalcElasticEnergyDensity(
      const CorotatedModelData<T, num_locations>& data,
      std::array<T, num_locations>* Psi) const;

  /* Shadows ConstitutiveModel::DoCalcFirstPiolaStress() as required by the CRTP
   base class. */
  void DoCalcFirstPiolaStress(const Data& data,
                              std::array<Matrix3<T>, num_locations>* P) const;

  /* Shadows ConstitutiveModel::DoCalcFirstPiolaStressDerivative() as required
   by the CRTP base class. */
  void DoCalcFirstPiolaStressDerivative(
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
