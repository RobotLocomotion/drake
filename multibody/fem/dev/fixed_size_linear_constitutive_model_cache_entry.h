#pragma once

#include <array>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"
#include "drake/multibody/fem/dev/fixed_size_deformation_gradient_cache_entry.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cache entry for the FixedSizeLinearConstitutiveModel constitutive model.
 See FixedSizeLinearConstitutiveModel for how the cache entry is used. See
 FixedSizeDeformationGradientCacheEntry for more about cached quantities for
 constitutive models.
 @tparam_nonsymbolic_scalar T.
 @tparam NumLocations Number of locations at which the cached quantities are
 evaluated. */
template <typename T, int NumLocations>
class FixedSizeLinearConstitutiveModelCacheEntry
    : public FixedSizeDeformationGradientCacheEntry<
          FixedSizeLinearConstitutiveModelCacheEntry<T, NumLocations>> {
 public:
  using Base = FixedSizeDeformationGradientCacheEntry<
      FixedSizeLinearConstitutiveModelCacheEntry<T, NumLocations>>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      FixedSizeLinearConstitutiveModelCacheEntry);

  ~FixedSizeLinearConstitutiveModelCacheEntry() = default;

  /** Constructs a %FixedSizeLinearConstitutiveModelCacheEntry with the given
   element index.
   @param element_index The index of the FemElement associated with this
   DeformationGradientCacheEntry. */
  explicit FixedSizeLinearConstitutiveModelCacheEntry(
      ElementIndex element_index)
      : FixedSizeDeformationGradientCacheEntry<
            FixedSizeLinearConstitutiveModelCacheEntry<T, NumLocations>>(
            element_index) {
    std::fill(strain_.begin(), strain_.end(), Matrix3<T>::Zero());
    std::fill(trace_strain_.begin(), trace_strain_.end(), 0);
  }

  /** Returns the infinitesimal strains evaluated at the quadrature locations
   for the associated element. */
  const std::array<Matrix3<T>, NumLocations>& strain() const { return strain_; }

  /** Returns the traces of the infinitesimal strains evaluated at the
   quadrature locations for the associated element. */
  const std::array<T, NumLocations>& trace_strain() const {
    return trace_strain_;
  }

 private:
  friend Base;
  /* Implements the NVI FixedSizeDeformationCacheEntry::UpdateCacheEntry.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element. */
  void DoUpdateCacheEntry(const std::array<Matrix3<T>, NumLocations>& F) {
    for (int i = 0; i < NumLocations; ++i) {
      strain_[i] = 0.5 * (F[i] + F[i].transpose()) - Matrix3<T>::Identity();
      trace_strain_[i] = strain_[i].trace();
    }
  }
  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::array<Matrix3<T>, NumLocations> strain_;
  // Trace of `strain_`.
  std::array<T, NumLocations> trace_strain_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
