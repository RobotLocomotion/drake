#pragma once

#include <array>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
template <class>
class FixedSizeDeformationGradientCacheEntry;
/** %FixedSizeDeformationGradientCacheEntry stores per element cached quantities
 that work in tandem with FixedSizeConstitutiveModel. It is a static
 interface that concrete constitutive models must inherit from to store
 the set of specific quantities that need to be cached for the specific model.
 There should be a one-to-one correspondence between the constitutive model
 `FixedSizeFoo` that inherits from FixedSizeConstitutiveModel and its cached
 quantities `FixedSizeFooCacheEntry` that inherits from
 %FixedSizeDeformationGradientCacheEntry. These cached quantities depend solely
 on deformation gradients, and they facilitate calculations such as energy
 density, stress and stress derivative in the constitutive model.
 FixedSizeConstitutiveModel takes the corresponding cache entry as an argument
 when performing various calculations.
 @tparam_nonsymbolic_scalar T.
 @tparam NumLocations Number of locations at which the cached quantities
 are evaluated. */
template <template <typename, int> class DerivedDeformationGradientCacheEntry,
          typename T, int NumLocations>
class FixedSizeDeformationGradientCacheEntry<
    DerivedDeformationGradientCacheEntry<T, NumLocations>> {
 public:
  using Derived = DerivedDeformationGradientCacheEntry<T, NumLocations>;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      FixedSizeDeformationGradientCacheEntry);

  ~FixedSizeDeformationGradientCacheEntry() = default;

  /** Updates the cache entry with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element. */
  void UpdateCacheEntry(const std::array<Matrix3<T>, NumLocations>& F) {
    deformation_gradient_ = F;
    static_cast<Derived*>(this)->DoUpdateCacheEntry(F);
  }

  /** The index of the FemElement associated with this
   %FixedSizeDeformationGradientCacheEntry. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which the cache entry needs to be
   evaluated. */
  static constexpr int num_quadrature_points() { return NumLocations; }

  const std::array<Matrix3<T>, NumLocations>& deformation_gradient() const {
    return deformation_gradient_;
  }

 protected:
  /* Constructs a FixedSizeDeformationGradientCacheEntry with the given element
   index and. Users should not directly construct
   FixedSizeDeformationGradientCacheEntry. They should construct the specific
   constitutive model cache entry (e.g.
   FixedSizeLinearConstitutiveModelCacheEntry) that invokes the base
   constructor.
   @param element_index The index of the FemElement associated with this
   FixedSizeDeformationGradientCacheEntry. */
  explicit FixedSizeDeformationGradientCacheEntry(ElementIndex element_index)
      : element_index_(element_index) {
    DRAKE_ASSERT(element_index.is_valid());
    std::fill(deformation_gradient_.begin(), deformation_gradient_.end(),
              Matrix3<T>::Identity());
  }

 private:
  ElementIndex element_index_;
  std::array<Matrix3<T>, NumLocations> deformation_gradient_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
