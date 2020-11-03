#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cached quantities for the LinearElasticityModel constitutive model.
 See LinearElasticityModel for how the cache is used. See
 DeformationGradientCache for more about cached quantities for constitutive
 models.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class LinearElasticityModelCache : public DeformationGradientCache<T> {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is used only to facilitate implementation of Clone()
   in derived classes. */
  LinearElasticityModelCache(LinearElasticityModelCache&&) = delete;
  LinearElasticityModelCache& operator=(LinearElasticityModelCache&&) = delete;
  LinearElasticityModelCache& operator=(const LinearElasticityModelCache&) =
      delete;
  /** @} */

  /** Constructs a %LinearElasticityModelCache with the given element index and
   number of quadrature locations.
   @param element_index The index of the FemElement associated with this
   DeformationGradientCache.
   @param num_quads The number of quadrature locations at which cached
   quantities need to be evaluated.
   @pre `num_quads` must be positive. */
  LinearElasticityModelCache(ElementIndex element_index, int num_quads)
      : DeformationGradientCache<T>(element_index, num_quads),
        strain_(num_quads),
        trace_strain_(num_quads) {}

  /** Returns the infinitesimal strains evaluated at the quadrature locations
   for the associated element. */
  const std::vector<Matrix3<T>>& strain() const { return strain_; }

  /** Returns the traces of the infinitesimal strains evaluated at the
   quadrature locations for the associated element. */
  const std::vector<T>& trace_strain() const { return trace_strain_; }

 protected:
  /* Copy constructor to facilitate the `DoClone()` method. */
  LinearElasticityModelCache(const LinearElasticityModelCache&) = default;

  /* Updates the cached quantities with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element.
   @pre The size of `F` must be the same as `num_quads()`. */
  void DoUpdateCache(const std::vector<Matrix3<T>>& F) final;

  /* Creates an identical copy of the LinearElasticityModelCache object. */
  std::unique_ptr<DeformationGradientCache<T>> DoClone() const final {
    // Can't use std::make_unique because the copy constructor is protected.
    return std::unique_ptr<DeformationGradientCache<T>>(
        new LinearElasticityModelCache<T>(*this));
  }

 private:
  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::vector<Matrix3<T>> strain_;
  // Trace of `strain_`.
  std::vector<T> trace_strain_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::LinearElasticityModelCache);
