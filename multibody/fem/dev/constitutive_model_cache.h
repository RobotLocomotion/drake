#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {
namespace fem {
    // TODO (xuchenhan-tri): Maybe there's better place to put this alias.
/** Index used to identify an element among all FEM elements. */
using ElementIndex = TypeSafeIndex<class ElementTag>;

/** ConstitutiveModel stores per element cached quantities that work in tandem
 with @ref ConstitutiveModel. It is an abstract interface that actual concrete
 constitutive models must inherit from to store the set of specific quantities
 that need to be cached for the specific model. There should be a one-to-one
 correspondence between constitutive model `Foo` that inherits from
 @ref ConstitutiveModel and its cached quantities `FooCache` that inherits from
 ConstitutiveModelCache. These cache quantities depend solely on deformation
 gradients, and they facilitate calculations such as energy density, stress and
 stress derivative in the constitutive model. The @ref ConstitutiveModel takes
 the corresponding cache as an argument when performing various calculations.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
template <typename T>
class ConstitutiveModelCache {
 public:
  ElementIndex element_index() const { return element_index_; }

  int num_quads() const { return num_quads_; }

  const std::vector<Matrix3<T>>& deformation_gradient() const {
    return deformation_gradient_;
  }

 protected:
  /* Constructs a ConstitutiveModelCache with the given element index and
   number of quadrature locations.
   @param element_index The index of the element that `this`
   ConstitutiveModelCache lives on.
   @param num_quads The number of quadrature locations at which cached
   quantities need to be evaluated. */
  ConstitutiveModelCache(ElementIndex element_index, int num_quads)
      : element_index_(element_index),
        num_quads_(num_quads),
        deformation_gradient_(num_quads) {
    DRAKE_DEMAND(element_index >= 0);
    DRAKE_DEMAND(num_quads > 0);
  }

 private:
  const ElementIndex element_index_{-1};
  const int num_quads_{-1};
  std::vector<Matrix3<T>> deformation_gradient_;
};

/** Cached quantities for the LinearElasticity constitutive model.
 See @ref linear_elasticity ConstitutiveModel for how the cache is used.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class LinearElasticityCache : public ConstitutiveModelCache<T> {
public:
  LinearElasticityCache(int element_index, int num_quads)
      : ConstitutiveModelCache<T>(element_index, num_quads),
        strain_(num_quads),
        trace_strain_(num_quads) {}

private:
  std::vector<Matrix3<T>> strain_;  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::vector<T> trace_strain_;     // Trace of `strain`.
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
