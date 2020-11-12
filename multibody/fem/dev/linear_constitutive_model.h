#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model.h"
#include "drake/multibody/fem/dev/linear_constitutive_model_cache_entry.h"

namespace drake {
namespace multibody {
namespace fem {
/** Implements the infinitesimal-strain linear elasticity constitutive model as
 described in Section 7.4 of [Gonzalez, 2008].
 @tparam_nonsymbolic_scalar T.

[Gonzalez, 2008] Gonzalez, Oscar, and Andrew M. Stuart. A first course in
continuum mechanics. Cambridge University Press, 2008. */
template <typename T>
class LinearConstitutiveModel final : public ConstitutiveModel<T> {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  LinearConstitutiveModel(LinearConstitutiveModel&&) = delete;
  LinearConstitutiveModel& operator=(LinearConstitutiveModel&&) = delete;
  LinearConstitutiveModel& operator=(const LinearConstitutiveModel&) = delete;
  /** @} */

  /** Constructs a %LinearConstitutiveModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poisson_ratio Poisson ratio of the model, unitless.
   @pre youngs_modulus must be non-negative.
   @pre poisson_ratio must be strictly greater than -1 and strictly smaller than
   0.5. */
  LinearConstitutiveModel(const T& youngs_modulus, const T& poisson_ratio);

  ~LinearConstitutiveModel() = default;

  T youngs_modulus() const { return E_; }

  T poisson_ratio() const { return nu_; }

  T shear_modulus() const { return mu_; }

  T lame_first_parameter() const { return lambda_; }

 private:
  /* Copy constructor to facilitate the `DoClone()` method. */
  LinearConstitutiveModel(const LinearConstitutiveModel&) = default;

  /* Creates an identical copy of the LinearConstitutiveModel object. */
  std::unique_ptr<ConstitutiveModel<T>> DoClone() const final {
    // Can't use std::make_unique because the copy constructor is protected.
    return std::unique_ptr<ConstitutiveModel<T>>(
        new LinearConstitutiveModel<T>(*this));
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

  /* Calculates the derivative of First Piola stress with respect to the
   deformation gradient, given the model cache entry. */
  void DoCalcFirstPiolaStressDerivative(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<Eigen::Matrix<T, 9, 9>>* dPdF) const final;

  /* Creates a LinearElasticityModelCache that is compatible with this
   LinearConstitutiveModel. */
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
  Eigen::Matrix<T, 9, 9>
      dPdF_;  // The First Piola stress derivative is constant and precomputed.
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::LinearConstitutiveModel);
