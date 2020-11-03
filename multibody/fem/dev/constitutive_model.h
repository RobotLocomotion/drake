#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** A constitutive model relates the strain to the stress of the material. It
 governs the material response under deformation. This constitutive relationship
 is defined through the potential energy, which increases with non-rigid
 deformation from the initial state.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ConstitutiveModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstitutiveModel);

  ConstitutiveModel() = default;

  virtual ~ConstitutiveModel() {}

  // TODO(xuchenhan-tri) Update the list of methods here as more methods are
  // introduced.
  // TODO(xuchenhan-tri) Clarify that FemElement is responsible for updating
  // the cached quantities once FemElement is introduced.
  /** @name "Calc" Methods
   "Calc" methods that calculate the energy density and stress given the cached
   quantities required for these calculations. The constitutive model expects
   that the input cached quantities are up-to-date.
   @warning Derived classes will static cast `cache` into derived cache classes
   that match the derived %ConstitutiveModel. Make sure the `cache` that is
   passed in matches the %ConstitutiveModel. */

  /** Calculates the energy density per unit reference volume, in unit J/m³,
   * given the model cache. */
  std::vector<T> CalcElasticEnergyDensity(
      const DeformationGradientCache<T>& cache) const {
    std::vector<T> Psi(cache.num_quads());
    CalcElasticEnergyDensity(cache, &Psi);
    return Psi;
  }

  /** Alternative signature for CalcElasticEnergyDensity that writes the result
   in the output argument. */
  void CalcElasticEnergyDensity(const DeformationGradientCache<T>& cache,
                                std::vector<T>* Psi) const {
    DRAKE_DEMAND(Psi != nullptr);
    DRAKE_DEMAND(static_cast<int>(Psi->size()) == cache.num_quads());
    DoCalcElasticEnergyDensity(cache, Psi);
  }

  /** Calculates the First Piola stress, in unit Pa, given the model cache. */
  std::vector<Matrix3<T>> CalcFirstPiolaStress(
      const DeformationGradientCache<T>& cache) const {
    std::vector<Matrix3<T>> P(cache.num_quads());
    CalcFirstPiolaStress(cache, &P);
    return P;
  }

  /** Alternative signature for CalcFirstPiolaStress that writes the result in
   the output argument. */
  void CalcFirstPiolaStress(const DeformationGradientCache<T>& cache,
                            std::vector<Matrix3<T>>* P) const {
    DRAKE_DEMAND(P != nullptr);
    DRAKE_DEMAND(static_cast<int>(P->size()) == cache.num_quads());
    DoCalcFirstPiolaStress(cache, P);
  }

 protected:
  /* Calculates the energy density, in unit J/m³, given the model cache. */
  virtual void DoCalcElasticEnergyDensity(
      const DeformationGradientCache<T>& cache, std::vector<T>* Psi) const = 0;

  /* Calculates the First Piola stress, in unit Pa, given the model cache. */
  virtual void DoCalcFirstPiolaStress(const DeformationGradientCache<T>& cache,
                                      std::vector<Matrix3<T>>* P) const = 0;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
