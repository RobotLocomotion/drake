#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** %DeformationGradientCacheEntry stores per element cached quantities that
 work in tandem with ConstitutiveModel. It is an abstract interface that actual
 concrete constitutive models must inherit from to store the set of specific
 quantities that need to be cached for the specific model. There should be a
 one-to-one correspondence between the constitutive model `Foo` that inherits
 from ConstitutiveModel and its cached quantities `FooCacheEntry` that inherits
 from %DeformationGradientCacheEntry. These cached quantities depend solely on
 deformation gradients, and they facilitate calculations such as energy density,
 stress and stress derivative in the constitutive model. ConstitutiveModel takes
 the corresponding cache entry as an argument when performing various
 calculations.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class DeformationGradientCacheEntry {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  DeformationGradientCacheEntry(DeformationGradientCacheEntry&&) = delete;
  DeformationGradientCacheEntry& operator=(DeformationGradientCacheEntry&&) =
      delete;
  DeformationGradientCacheEntry& operator=(
      const DeformationGradientCacheEntry&) = delete;
  /** @} */

  /** Creates an identical copy of the concrete DeformationGradientCacheEntry
   object. */
  std::unique_ptr<DeformationGradientCacheEntry<T>> Clone() const {
    return DoClone();
  }

  virtual ~DeformationGradientCacheEntry() = default;

  /** Updates the cache entry with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element.
   @pre The size of `F` must be the same as `num_quadrature_points()`. */
  void UpdateCacheEntry(const std::vector<Matrix3<T>>& F) {
    DRAKE_ASSERT(static_cast<int>(F.size()) == num_quadrature_points_);
    deformation_gradient_ = F;
    DoUpdateCacheEntry(F);
  }

  /** The index of the FemElement associated with this
   %DeformationGradientCacheEntry. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which the cache entry needs to be
   evaluated. */
  int num_quadrature_points() const { return num_quadrature_points_; }

  const std::vector<Matrix3<T>>& deformation_gradient() const {
    return deformation_gradient_;
  }

 protected:
  /* Constructs a DeformationGradientCacheEntry with the given element index and
   number of quadrature locations. Users should not directly construct
   DeformationGradientCacheEntry. They should construct the specific
   constitutive model cache entry (e.g. LinearConstitutiveModelCacheEntry) that
   invokes the base constructor.
   @param element_index The index of the FemElement associated with this
   DeformationGradientCacheEntry.
   @param num_quadrature_points The number of quadrature locations at which
   cached quantities need to be evaluated.
   @pre `num_quadrature_points` must be positive. */
  DeformationGradientCacheEntry(ElementIndex element_index,
                                int num_quadrature_points)
      : element_index_(element_index),
        num_quadrature_points_(num_quadrature_points),
        deformation_gradient_(num_quadrature_points, Matrix3<T>::Identity()) {
    DRAKE_ASSERT(element_index.is_valid());
    DRAKE_ASSERT(num_quadrature_points > 0);
  }

  /** Copy constructor for the base DeformationGradientCacheEntry class to
   facilitate `DoClone()` in derived classes. */
  DeformationGradientCacheEntry(const DeformationGradientCacheEntry&) = default;

  /** Creates an identical copy of the concrete DeformationGradientCacheEntry
   object. Derived classes must implement this so that it performs the complete
   deep copy of the object, including all base class members. */
  virtual std::unique_ptr<DeformationGradientCacheEntry<T>> DoClone() const = 0;

  /** Updates the cached quantities with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element.
   @pre The size of `F` must be the same as `num_quadrature_points()`. */
  virtual void DoUpdateCacheEntry(const std::vector<Matrix3<T>>& F) = 0;

 private:
  ElementIndex element_index_;
  int num_quadrature_points_{-1};
  std::vector<Matrix3<T>> deformation_gradient_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
