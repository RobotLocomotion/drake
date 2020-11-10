#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** %DeformationGradientCache stores per element cached quantities that work in
 tandem with ConstitutiveModel. It is an abstract interface that actual concrete
 constitutive models must inherit from to store the set of specific quantities
 that need to be cached for the specific model. There should be a one-to-one
 correspondence between the constitutive model `Foo` that inherits from
 ConstitutiveModel and its cached quantities `FooCache` that inherits from
 %DeformationGradientCache. These cached quantities depend solely on deformation
 gradients, and they facilitate calculations such as energy density, stress and
 stress derivative in the constitutive model. ConstitutiveModel takes
 the corresponding cache as an argument when performing various calculations.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class DeformationGradientCache {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  DeformationGradientCache(DeformationGradientCache&&) = delete;
  DeformationGradientCache& operator=(DeformationGradientCache&&) = delete;
  DeformationGradientCache& operator=(const DeformationGradientCache&) = delete;
  /** @} */

  /** Creates an identical copy of the concrete DeformationGradientCache object.
   */
  std::unique_ptr<DeformationGradientCache<T>> Clone() const {
    return DoClone();
  }

  virtual ~DeformationGradientCache() = default;

  /** Updates the cached quantities with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element.
   @pre The size of `F` must be the same as `num_quads()`. */
  void UpdateCache(const std::vector<Matrix3<T>>& F) {
    DRAKE_ASSERT(static_cast<int>(F.size()) == num_quads_);
    deformation_gradient_ = F;
    DoUpdateCache(F);
  }

  /** The index of the FemElement associated with this
   %DeformationGradientCache. */
  ElementIndex element_index() const { return element_index_; }

  /** The number of quadrature locations at which cached quantities need to be
   evaluated. */
  int num_quads() const { return num_quads_; }

  const std::vector<Matrix3<T>>& deformation_gradient() const {
    return deformation_gradient_;
  }

 protected:
  /* Constructs a DeformationGradientCache with the given element index and
   number of quadrature locations. Users should not directly construct
   DeformationGradientCache. They should construct specific constitutive model
   caches (e.g. LinearElasticityModelCache) that invoke the base constructor.
   @param element_index The index of the FemElement associated with this
   DeformationGradientCache.
   @param num_quads The number of quadrature locations at which cached
   quantities need to be evaluated.
   @pre `num_quads` must be positive. */
  DeformationGradientCache(ElementIndex element_index, int num_quads)
      : element_index_(element_index),
        num_quads_(num_quads),
        deformation_gradient_(num_quads, Matrix3<T>::Identity()) {
    DRAKE_ASSERT(element_index.is_valid());
    DRAKE_ASSERT(num_quads > 0);
  }

  /* Copy constructor for the base DeformationGradientCache class to facilitate
   `DoClone()` in derived classes. */
  DeformationGradientCache(const DeformationGradientCache&) = default;

  /* Creates an identical copy of the concrete DeformationGradientCache object.
   Derived classes must implement this so that it performs the complete
   deep copy of the object, including all base class members. */
  virtual std::unique_ptr<DeformationGradientCache<T>> DoClone() const = 0;

  /* Updates the cached quantities with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element.
   @pre The size of `F` must be the same as `num_quads()`. */
  virtual void DoUpdateCache(const std::vector<Matrix3<T>>& F) = 0;

 private:
  ElementIndex element_index_;
  int num_quads_{-1};
  std::vector<Matrix3<T>> deformation_gradient_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
