#pragma once

#include <array>

#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/linear_corotated_model_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Traits for LinearCorotatedModel. */
template <typename T, int num_locations>
struct LinearCorotatedModelTraits {
  using Scalar = T;
  using Data = LinearCorotatedModelData<T, num_locations>;
};

/* Implements the linear corotated constitutive model as described in
 [Han et al., 2023].
 @tparam_nonsymbolic_scalar
 @tparam num_locations Number of locations at which the constitutive
 relationship is evaluated. We currently only provide one instantiation of this
 template with `num_locations = 1`, but more instantiations can easily be added
 when needed.

 [Han et al., 2023] Han, Xuchen, Joseph Masterjohn, and Alejandro Castro. "A
 Convex Formulation of Frictional Contact between Rigid and Deformable Bodies."
 arXiv preprint arXiv:2303.08912 (2023). */
template <typename T, int num_locations>
class LinearCorotatedModel final
    : public ConstitutiveModel<LinearCorotatedModel<T, num_locations>,
                               LinearCorotatedModelTraits<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearCorotatedModel)

  using Traits = LinearCorotatedModelTraits<T, num_locations>;
  using Data = typename Traits::Data;

  /* Constructs a LinearCorotatedModel constitutive model with the
   prescribed Young's modulus and Poisson's ratio.
   @param youngs_modulus  Young's modulus of the model, with units of N/m².
   @param poissons_ratio  Poisson's ratio of the model, unitless.
   @pre youngs_modulus >= 0.
   @pre -1 < poissons_ratio < 0.5. */
  LinearCorotatedModel(const T& youngs_modulus, const T& poissons_ratio);

  const T& youngs_modulus() const { return E_; }

  const T& poissons_ratio() const { return nu_; }

  /* Returns the shear modulus (Lame's second parameter) which is given by
   `E/(2*(1+nu))` where `E` is the Young's modulus and `nu` is the Poisson's
   ratio. See `fem::internal::CalcLameParameters()`. */
  const T& shear_modulus() const { return mu_; }

  /* Returns the Lame's first parameter which is given by
   `E*nu/((1+nu)*(1-2*nu))` where `E` is the Young's modulus and `nu` is the
   Poisson's ratio. See `fem::internal::CalcLameParameters()`. */
  const T& lame_first_parameter() const { return lambda_; }

 private:
  friend ConstitutiveModel<LinearCorotatedModel<T, num_locations>,
                           LinearCorotatedModelTraits<T, num_locations>>;

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
  T nu_;      // Poisson's ratio.
  T mu_;      // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;  // Lamé's first parameter, N/m².
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
