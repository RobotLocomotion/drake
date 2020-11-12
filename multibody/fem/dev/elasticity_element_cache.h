#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache.h"
#include "drake/multibody/fem/dev/element_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cached quantities per element that are used in ElasticityElement. Implements
 the abstract interface ElementCache.

 See ElasticityElement for the corresponding FemElement for
 %ElasticityElementCache.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElasticityElementCache : public ElementCache<T> {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  ElasticityElementCache(ElasticityElementCache&&) = delete;
  ElasticityElementCache& operator=(ElasticityElementCache&&) = delete;
  ElasticityElementCache& operator=(const ElasticityElementCache&) = delete;
  /** @} */

  /** Constructs a new %ElasticityElementCache.
   @param element_index The index of the ElasticityElement associated with this
   %ElasticityElementCache.
   @param num_quadrature_points The number of quadrature locations at which
   cached quantities need to be evaluated.
   @param deformation_gradient_cache The DeformationGradientCache associated
   with this element. Must be compatible with the ConstitutiveModel in the
   ElasticityElement routine corresponding to this %ElasticityElementCache.
   @pre `num_quadrature_points` must be positive.
   @warning The input `deformation_gradient_cache` must be compatible with the
   ConstitutiveModel in the ElasticityElement that shares the same element index
   with this %ElasticityElementCache. More specifically, if the
   ConstitutiveModel in the corresponding ElasticityElement is of type
   "FooModel", then the input `deformation_gradient_cache` must be of type
   "FooModelCache". */
  ElasticityElementCache(
      ElementIndex element_index, int num_quadrature_points,
      std::unique_ptr<DeformationGradientCache<T>> deformation_gradient_cache)
      : ElementCache<T>(element_index, num_quadrature_points),
        deformation_gradient_cache_(std::move(deformation_gradient_cache)) {}

  virtual ~ElasticityElementCache() = default;

  /** @name Getter for the const cache entries.
   @{ */
  const DeformationGradientCache<T>& deformation_gradient_cache() const {
    return *deformation_gradient_cache_;
  }
  /** @} */

  /** @name Getter for the mutable cache entries.
   @{ */
  DeformationGradientCache<T>& mutable_deformation_gradient_cache() {
    return *deformation_gradient_cache_;
  }
  /** @} */

 private:
  /* Copy constructor to facilitate the `DoClone()` method. */
  ElasticityElementCache(const ElasticityElementCache&) = default;

  /* Creates an identical copy of the ElasticityElementCache object. */
  std::unique_ptr<ElementCache<T>> DoClone() const final {
    /* Can't use std::make_unique because the copy constructor is protected. */
    return std::unique_ptr<ElementCache<T>>(
        new ElasticityElementCache<T>(*this));
  }

  copyable_unique_ptr<DeformationGradientCache<T>> deformation_gradient_cache_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
