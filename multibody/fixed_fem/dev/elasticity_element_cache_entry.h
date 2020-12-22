#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/deformation_gradient_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Cache entries that are used in ElasticityElement.
 @tparam ConstitutiveModel The ConstitutiveModel type that the ElasticityElement
 corresponding to this %ElasticityElementCacheEntry uses. */
template <class ConstitutiveModel>
class ElasticityElementCacheEntry : public ElementCacheEntry {
 public:
  using T = typename ConstitutiveModel::T;
  using DeformationGradientCacheEntryType =
      typename ConstitutiveModel::DeformationGradientCacheEntryType;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElasticityElementCacheEntry);

  /** Constructs a new %ElasticityElementCacheEntry.
   @param element_index The index of the ElasticityElement associated with this
   %ElasticityElementCacheEntry.
   @param deformation_gradient_cache_entry The DeformationGradientCacheEntry
   associated with this element. */
  explicit ElasticityElementCacheEntry(ElementIndex element_index)
      : ElementCacheEntry(element_index),
        deformation_gradient_cache_entry_(element_index) {}

  ~ElasticityElementCacheEntry() = default;

  /** @name Getter for the const cache entries.
   @{ */
  const DeformationGradientCacheEntryType& deformation_gradient_cache_entry()
      const {
    return deformation_gradient_cache_entry_;
  }
  /** @} */

  /** @name Getter for the mutable cache entries.
   @{ */
  DeformationGradientCacheEntryType& mutable_deformation_gradient_cache_entry()
      const {
    return deformation_gradient_cache_entry_;
  }
  /** @} */

 private:
  mutable DeformationGradientCacheEntryType deformation_gradient_cache_entry_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
