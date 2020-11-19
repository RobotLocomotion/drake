#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model.h"
#include "drake/multibody/fem/dev/linear_elasticity_model_cache_entry.h"

namespace drake {
namespace multibody {
namespace fem {
/** Implements the infinitesimal-strain linear elasticity constitutive model as
 described in Section 7.4 of [Gonzalez, 2008].
 @tparam_nonsymbolic_scalar T.

[Gonzalez, 2008] Gonzalez, Oscar, and Andrew M. Stuart. A first course in
continuum mechanics. Cambridge University Press, 2008. */
template <typename T>
class LinearElasticityModel final : public ConstitutiveModel<T> {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  LinearElasticityModel(LinearElasticityModel&&) = delete;
  LinearElasticityModel& operator=(LinearElasticityModel&&) = delete;
  LinearElasticityModel& operator=(const LinearElasticityModel&) = delete;
  /** @} */

  /** Constructs a %LinearElasticityModel constitutive model with the prescribed
   Young's modulus and Poisson ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poisson_ratio Poisson ratio of the model, unitless.
   @pre youngs_modulus must be non-negative.
   @pre poisson_ratio must be strictly greater than -1 and strictly smaller than
   0.5. */
  LinearElasticityModel(const T& youngs_modulus, const T& poisson_ratio);

  ~LinearElasticityModel() = default;

  T youngs_modulus() const { return E_; }

  T poisson_ratio() const { return nu_; }

  T shear_modulus() const { return mu_; }

  T lame_first_parameter() const { return lambda_; }

 private:
  /* Copy constructor to facilitate the `DoClone()` method. */
  LinearElasticityModel(const LinearElasticityModel&) = default;

  /* Creates an identical copy of the LinearElasticityModel object. */
  std::unique_ptr<ConstitutiveModel<T>> DoClone() const final {
    // Can't use std::make_unique because the copy constructor is protected.
    return std::unique_ptr<ConstitutiveModel<T>>(
        new LinearElasticityModel<T>(*this));
  }

  /* Calculates the energy density, in unit J/m³, given the model cache entry.
   */
  void DoCalcElasticEnergyDensity(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<T>* Psi) const final;

  /* Calculates the First Piola stress, in unit Pa, given the model cache entry.
   */
  void DoCalcFirstPiolaStress(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<Matrix3<T>>* P) const final;

  /* Creates a LinearElasticityModelCache that is compatible with this
   LinearElasticityModel. */
  std::unique_ptr<DeformationGradientCacheEntry<T>>
  DoMakeDeformationGradientCacheEntry(ElementIndex element_index,
                                      int num_quadrature_points) const final;

  /* Set the Lamé parameters from Young's modulus and Poisson ratio. It's
  important to keep the Lamé Parameters in sync with Young's modulus and
  Poisson ratio as most computations use Lame parameters. */
  void VerifyParameterValidity(const T& youngs_modulus,
                               const T& poisson_ratio) const;

  void SetLameParameters(const T& youngs_modulus, const T& poisson_ratio);

  T E_;       // Young's modulus, N/m².
  T nu_;      // Poisson ratio.
  T mu_;      // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;  // Lamé's first parameter, N/m².
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::LinearElasticityModel);
