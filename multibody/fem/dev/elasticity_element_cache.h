#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache.h"
#include "drake/multibody/fem/dev/element_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cached quantities per element that are used in the element routine
 FemElasticity. Implements the abstract interface ElementCache.

 See FemElasticity for the corresponding FemElement for %ElasticityElementCache.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ElasticityElementCache : public ElementCache<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElasticityElementCache);

  /** Constructs a new %ElasticityElementCache.
   @param element_index The index of the FemElasticity associated with this
   %ElasticityElementCache.
   @param num_quads The number of quadrature locations at which cached
   quantities need to be evaluated.
   @param deformation_gradient_cache The DeformationGradientCache associated
   with this element. Must be compatible with the ConstitutiveModel in the
   FemElasticity routine corresponding to this %ElasticityElementCache.
   @pre `num_quads` must be positive.
   @warning The input `deformation_gradient_cache` must be compatible with the
   ConstitutiveModel in the FemElasticity that shares the same element index
   with this %ElasticityElementCache. More specifically, if the
   ConstitutiveModel in the corresponding FemElasticity is of type "FooModel",
   then the input `deformation_gradient_cache` must be of type "FooModelCache".
  */
  ElasticityElementCache(
      ElementIndex element_index, int num_quads,
      std::unique_ptr<DeformationGradientCache<T>> deformation_gradient_cache)
      : ElementCache<T>(element_index, num_quads),
        deformation_gradient_cache_(std::move(deformation_gradient_cache)) {}

  virtual ~ElasticityElementCache() = default;

  /// Getter for the const cache entries.
  /// @{
  const std::unique_ptr<DeformationGradientCache<T>>&
  deformation_gradient_cache() const {
    return deformation_gradient_cache_;
  }
  /// @}

  /// Getter for the mutable cache entries.
  /// @{
  std::unique_ptr<DeformationGradientCache<T>>&
  mutable_deformation_gradient_cache() {
    return deformation_gradient_cache_;
  }
  /// @}

 private:
  std::unique_ptr<DeformationGradientCache<T>> deformation_gradient_cache_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
