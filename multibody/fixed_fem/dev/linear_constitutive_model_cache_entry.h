#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/deformation_gradient_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Cache entry for the LinearConstitutiveModel constitutive model.
 See LinearConstitutiveModel for how the cache entry is used. See
 DeformationGradientCacheEntry for more about cached quantities for
 constitutive models.
 @tparam_nonsymbolic_scalar T.
 @tparam num_locations Number of locations at which the cached quantities are
 evaluated. */
template <typename T, int num_locations>
class LinearConstitutiveModelCacheEntry
    : public DeformationGradientCacheEntry<
          LinearConstitutiveModelCacheEntry<T, num_locations>> {
 public:
  using Base = DeformationGradientCacheEntry<
      LinearConstitutiveModelCacheEntry<T, num_locations>>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModelCacheEntry);

  ~LinearConstitutiveModelCacheEntry() = default;

  /** Constructs a %LinearConstitutiveModelCacheEntry with zero strain. */
  LinearConstitutiveModelCacheEntry() {
    std::fill(strain_.begin(), strain_.end(), Matrix3<T>::Zero());
    std::fill(trace_strain_.begin(), trace_strain_.end(), 0);
  }

  /** Returns the infinitesimal strains evaluated at the quadrature locations
   for the associated element. */
  const std::array<Matrix3<T>, num_locations>& strain() const {
    return strain_;
  }

  /** Returns the traces of the infinitesimal strains evaluated at the
   quadrature locations for the associated element. */
  const std::array<T, num_locations>& trace_strain() const {
    return trace_strain_;
  }

 private:
  friend Base;

  /* Implements the interface DeformationGradientCacheEntry::UpdateCacheEntry().
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element. */
  void DoUpdateCacheEntry(const std::array<Matrix3<T>, num_locations>& F) {
    for (int i = 0; i < num_locations; ++i) {
      strain_[i] = 0.5 * (F[i] + F[i].transpose()) - Matrix3<T>::Identity();
      trace_strain_[i] = strain_[i].trace();
    }
  }
  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::array<Matrix3<T>, num_locations> strain_;
  // Trace of `strain_`.
  std::array<T, num_locations> trace_strain_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
