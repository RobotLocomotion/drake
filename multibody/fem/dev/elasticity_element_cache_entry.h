#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache_entry.h"
#include "drake/multibody/fem/dev/element_cache_entry.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cache entries that are used in ElasticityElement. Implements
 the abstract interface ElementCacheEntry.

 See ElasticityElement for the corresponding FemElement for
 %ElasticityElementCacheEntry.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElasticityElementCacheEntry : public ElementCacheEntry<T> {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  ElasticityElementCacheEntry(ElasticityElementCacheEntry&&) = delete;
  ElasticityElementCacheEntry& operator=(ElasticityElementCacheEntry&&) =
      delete;
  ElasticityElementCacheEntry& operator=(const ElasticityElementCacheEntry&) =
      delete;
  /** @} */

  /** Constructs a new %ElasticityElementCacheEntry.
   @param element_index The index of the ElasticityElement associated with this
   %ElasticityElementCacheEntry.
   @param num_quadrature_points The number of quadrature locations at which
   cached quantities need to be evaluated.
   @param deformation_gradient_cache_entry The DeformationGradientCacheEntry
   associated with this element. Must be compatible with the ConstitutiveModel
   in the ElasticityElement routine corresponding to this
   %ElasticityElementCacheEntry.
   @pre `num_quadrature_points` must be positive.
   @warning The input `deformation_gradient_cache_entry` must be compatible with
   the ConstitutiveModel in the ElasticityElement that shares the same element
   index with this %ElasticityElementCacheEntry. More specifically, if the
   ConstitutiveModel in the corresponding ElasticityElement is of type
   "FooModel", then the input `deformation_gradient_cache_entry` must be of type
   "FooModelCacheEntry". */
  ElasticityElementCacheEntry(ElementIndex element_index,
                              int num_quadrature_points,
                              std::unique_ptr<DeformationGradientCacheEntry<T>>
                                  deformation_gradient_cache_entry)
      : ElementCacheEntry<T>(element_index, num_quadrature_points),
        deformation_gradient_cache_entry_(
            std::move(deformation_gradient_cache_entry)) {}

  virtual ~ElasticityElementCacheEntry() = default;

  /** @name Getter for the const cache entries.
   @{ */
  const DeformationGradientCacheEntry<T>& deformation_gradient_cache_entry()
      const {
    return *deformation_gradient_cache_entry_;
  }
  /** @} */

  /** @name Getter for the mutable cache entries.
   @{ */
  DeformationGradientCacheEntry<T>& mutable_deformation_gradient_cache_entry() {
    return *deformation_gradient_cache_entry_;
  }
  /** @} */

 private:
  /* Copy constructor to facilitate the `DoClone()` method. */
  ElasticityElementCacheEntry(const ElasticityElementCacheEntry&) = default;

  /* Creates an identical copy of the ElasticityElementCacheEntry object. */
  std::unique_ptr<ElementCacheEntry<T>> DoClone() const final {
    /* Can't use std::make_unique because the copy constructor is protected. */
    return std::unique_ptr<ElementCacheEntry<T>>(
        new ElasticityElementCacheEntry<T>(*this));
  }

  copyable_unique_ptr<DeformationGradientCacheEntry<T>>
      deformation_gradient_cache_entry_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
