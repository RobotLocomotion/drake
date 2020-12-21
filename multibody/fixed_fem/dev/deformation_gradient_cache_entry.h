#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
template <class>
class DeformationGradientCacheEntry;
/** %DeformationGradientCacheEntry stores per element cached quantities
 that work in tandem with ConstitutiveModel. It is a static
 interface that concrete constitutive model cache entries must inherit from to
 store the set of specific quantities that need to be cached for the specific
 model. There should be a one-to-one correspondence between the constitutive
 model `Foo` that inherits from ConstitutiveModel and its cached quantities
 `FooCacheEntry` that inherits from %DeformationGradientCacheEntry. These cached
 quantities depend solely on deformation gradients, and they facilitate
 calculations such as energy density, stress and stress derivative in the
 constitutive model. ConstitutiveModel takes the corresponding cache entry as an
 argument when performing various calculations. Similar to ConstitutiveModel,
 this class also utilizes CRTP to eliminate the need for virtual methods and
 facilitate inlining.
 @tparam_nonsymbolic_scalar T.
 @tparam num_locations Number of locations at which the cached quantities
 are evaluated. */
template <template <typename, int> class DerivedDeformationGradientCacheEntry,
          typename T, int num_locations>
class DeformationGradientCacheEntry<
    DerivedDeformationGradientCacheEntry<T, num_locations>> {
 public:
  using Derived = DerivedDeformationGradientCacheEntry<T, num_locations>;

  ~DeformationGradientCacheEntry() = default;

  /** Updates the cache entry with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element. */
  void UpdateCacheEntry(const std::array<Matrix3<T>, num_locations>& F) {
    deformation_gradient_ = F;
    static_cast<Derived*>(this)->DoUpdateCacheEntry(F);
  }

  /** The index of the FemElement associated with this
   %DeformationGradientCacheEntry. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which the cache entry needs to be
   evaluated. */
  static constexpr int num_quadrature_points() { return num_locations; }

  const std::array<Matrix3<T>, num_locations>& deformation_gradient() const {
    return deformation_gradient_;
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformationGradientCacheEntry);

  /** Constructs a $DeformationGradientCacheEntry with the given element
   index and. Users should not directly construct
   %DeformationGradientCacheEntry. They should construct the specific
   constitutive model cache entry (e.g.
   LinearConstitutiveModelCacheEntry) that invokes the base
   constructor.
   @param element_index The index of the FemElement associated with this
   %DeformationGradientCacheEntry. */
  explicit DeformationGradientCacheEntry(ElementIndex element_index)
      : element_index_(element_index) {
    DRAKE_ASSERT(element_index.is_valid());
    std::fill(deformation_gradient_.begin(), deformation_gradient_.end(),
              Matrix3<T>::Identity());
  }

 private:
  ElementIndex element_index_;
  std::array<Matrix3<T>, num_locations> deformation_gradient_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
