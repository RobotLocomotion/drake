#pragma once

#include <array>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fixed_size_deformation_gradient_cache_entry.h"

namespace drake {
namespace multibody {
namespace fem {
template <class>
class FixedSizeConstitutiveModel;
/** A constitutive model relates the strain to the stress of the material. It
 governs the material response under deformation. This constitutive relationship
 is defined through the potential energy, which increases with non-rigid
 deformation from the initial state.
 @tparam_nonsymbolic_scalar T.
 @tparam NumLocations The number of locations to evaluate the energy density,
 stress and its derivatives. */
template <template <typename, int> class DerivedConstitutiveModel, typename T,
          int NumLocations>
class FixedSizeConstitutiveModel<DerivedConstitutiveModel<T, NumLocations>> {
 public:
  using Derived = DerivedConstitutiveModel<T, NumLocations>;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedSizeConstitutiveModel)

  FixedSizeConstitutiveModel() = default;

  ~FixedSizeConstitutiveModel() {}

  /** The number of locations at which the constitutive relationship is
   evaluated. */
  static constexpr int num_locations() { return NumLocations; }

  /** @name "Calc" Methods
   "Calc" methods that calculate the energy density and stress given the cache
   entry required for these calculations. The constitutive model expects
   that the input cache entries are up-to-date. FixedSizeFemElement is
   responsible for updating these cache entries. FixedSizeConstitutiveModel will
   not and cannot verify the cache entries provided are up-to-date. */

  /** Calculates the energy density, in unit J/m³, given the model cache entry.
   */
  /* This is a work around for Derived::DeformationGradientCacheEntryType not
   being available. Ideally, the signature of the method would be
   void CalcElasticEnergyDensity(
        const typename Derived::DeformationGradientCacheEntryType&,
        std::array<T, NumLocations>*)   */
  template <template <typename, int> class DeformationGradientCacheEntryType>
  void CalcElasticEnergyDensity(
      const DeformationGradientCacheEntryType<T, NumLocations>& cache_entry,
      std::array<T, NumLocations>* Psi) const {
    DRAKE_ASSERT(Psi != nullptr);
    static_cast<const Derived*>(this)->DoCalcElasticEnergyDensity(cache_entry,
                                                                  Psi);
  }

  /** Calculates the First Piola stress, in unit Pa, given the model cache
   entry. */
  /* This is a work around for Derived::DeformationGradientCacheEntryType not
   being available. Ideally, the signature of the method would be
   void CalcFirstPiolaStress(
        const typename Derived::DeformationGradientCacheEntryType&,
        std::array<Matrix3<T>, NumLocations>*)   */
  template <template <typename, int> class DeformationGradientCacheEntryType>
  void CalcFirstPiolaStress(
      const DeformationGradientCacheEntryType<T, NumLocations>& cache_entry,
      std::array<Matrix3<T>, NumLocations>* P) const {
    DRAKE_ASSERT(P != nullptr);
    static_cast<const Derived*>(this)->DoCalcFirstPiolaStress(cache_entry, P);
  }

  /** Calculates the derivative of First Piola stress with respect to the
   deformation gradient, given the model cache entry. */
  /* This is a work around for Derived::DeformationGradientCacheEntryType not
   being available. Ideally, the signature of the method would be
   void CalcFirstPiolaStressDerivative(
        const typename Derived::DeformationGradientCacheEntryType&,
        std::array<Eigen::Matrix<T, 9, 9>, NumLocations>*)   */
  template <template <typename, int> class DeformationGradientCacheEntryType>
  void CalcFirstPiolaStressDerivative(
      const DeformationGradientCacheEntryType<T, NumLocations>& cache_entry,
      std::array<Eigen::Matrix<T, 9, 9>, NumLocations>* dPdF) const {
    DRAKE_ASSERT(dPdF != nullptr);
    static_cast<const Derived*>(this)->DoCalcFirstPiolaStressDerivative(
        cache_entry, dPdF);
  }

  /** Creates a FixedSizeDeformationGradientCacheEntry that is compatible
   with this FixedSizeConstitutiveModel. */
  /* This is a work around for Derived::DeformationGradientCacheEntryType not
   being available. Ideally, the signature of the method would be
   typename Derived::DeformationGradientCacheEntryType
        MakeDeformationGradientCacheEntry(ElementIndex). */
  auto MakeDeformationGradientCacheEntry(ElementIndex element_index) const {
    static_cast<const Derived*>(this)->DoMakeDeformationGradientCacheEntry(
        element_index);
  }
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
