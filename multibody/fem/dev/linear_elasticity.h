#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model.h"
#include "drake/multibody/fem/dev/constitutive_model_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** @anchor linear_elasticity
 Implements the linear elasticity constitutive model.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
template <typename T, int SpatialDim>
class LinearElasticity final : public ConstitutiveModel<T, SpatialDim> {
 public:
  using MatrixD = typename ConstitutiveModel<T, SpatialDim>::MatrixD;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearElasticity);

  LinearElasticity(T E, T nu);

  virtual ~LinearElasticity() = default;

  /** Calculates the energy density with the given model cache. */
  std::vector<T> CalcPsi(
      const ConstitutiveModelCache<T, SpatialDim>& cache) const final;

  /** Calculates the First Piola stress with the given model cache. */
  std::vector<MatrixD> CalcP(
      const ConstitutiveModelCache<T, SpatialDim>& cache) const final;

 private:
  /* Facilitates testing. */
  friend class LinearElasticityTest;

  /* Set the Lamé parameters from Young's modulus and Poisson ratio. It's
  important to keep the Lamé Parameters in sync with Young's modulus and
  Poisson ratio as most computations use Lame parameters. */
  void VerifyParameterValidity(T E, T nu) const;

  void SetLameParameters(T E, T nu);

  T E_;       // Young's modulus.
  T nu_;      // Poisson ratio.
  T mu_;      // Lamé's second parameter/Shear modulus.
  T lambda_;  // Lamé's first parameter.
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
