#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model_data.h"

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
 @tparam_nonsymbolic_scalar T.

[Gonzalez, 2008] Gonzalez, Oscar, and Andrew M. Stuart. A first course in
continuum mechanics. Cambridge University Press, 2008. */
template <typename T, int num_locations>
class LinearConstitutiveModel final
    : public ConstitutiveModel<
          LinearConstitutiveModel<T, num_locations>,
          LinearConstitutiveModelTraits<T, num_locations>> {
 public:
  using Traits = LinearConstitutiveModelTraits<T, num_locations>;
  using Data = typename Traits::Data;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModel)

  /* Constructs a LinearConstitutiveModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poisson_ratio Poisson ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poisson_ratio < 0.5. */
  LinearConstitutiveModel(const T& youngs_modulus, const T& poisson_ratio);

  const T& youngs_modulus() const { return E_; }

  const T& poisson_ratio() const { return nu_; }

  const T& shear_modulus() const { return mu_; }

  const T& lame_first_parameter() const { return lambda_; }

 private:
  using Base =
      ConstitutiveModel<LinearConstitutiveModel<T, num_locations>, Traits>;
  friend Base;

  /* Shadows ConstitutiveModel::DoCalcElasticEnergyDensity() as required by the
   CRTP base class. */
  void DoCalcElasticEnergyDensity(
      const LinearConstitutiveModelData<T, num_locations>& data,
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
  Eigen::Matrix<T, 9, 9>
      dPdF_;  // The First Piola stress derivative is constant and precomputed.
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
