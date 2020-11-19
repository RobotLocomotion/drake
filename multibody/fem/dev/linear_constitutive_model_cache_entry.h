#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache_entry.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cache entry for the LinearConstitutiveModel constitutive model.
 See LinearConstitutiveModel for how the cache entry is used. See
 DeformationGradientCacheEntry for more about cached quantities for constitutive
 models.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class LinearConstitutiveModelCacheEntry
    : public DeformationGradientCacheEntry<T> {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  LinearConstitutiveModelCacheEntry(LinearConstitutiveModelCacheEntry&&) =
      delete;
  LinearConstitutiveModelCacheEntry& operator=(
      LinearConstitutiveModelCacheEntry&&) = delete;
  LinearConstitutiveModelCacheEntry& operator=(
      const LinearConstitutiveModelCacheEntry&) = delete;
  /** @} */

  /** Constructs a %LinearConstitutiveModelCacheEntry with the given element
   index and number of quadrature locations.
   @param element_index The index of the FemElement associated with this
   DeformationGradientCacheEntry.
   @param num_quadrature_points The number of quadrature locations at which the
   cache entry needs to be evaluated.
   @pre `num_quadrature_points` must be positive. */
  LinearConstitutiveModelCacheEntry(ElementIndex element_index,
                                    int num_quadrature_points)
      : DeformationGradientCacheEntry<T>(element_index, num_quadrature_points),
        strain_(num_quadrature_points, Matrix3<T>::Zero()),
        trace_strain_(num_quadrature_points, 0) {}

  /** Returns the infinitesimal strains evaluated at the quadrature locations
   for the associated element. */
  const std::vector<Matrix3<T>>& strain() const { return strain_; }

  /** Returns the traces of the infinitesimal strains evaluated at the
   quadrature locations for the associated element. */
  const std::vector<T>& trace_strain() const { return trace_strain_; }

 private:
  /* Copy constructor to facilitate the `DoClone()` method. */
  LinearConstitutiveModelCacheEntry(const LinearConstitutiveModelCacheEntry&) =
      default;

  /* Updates the cache entry with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element.
   @pre The size of `F` must be the same as `num_quadrature_points()`. */
  void DoUpdateCacheEntry(const std::vector<Matrix3<T>>& F) final;

  /* Creates an identical copy of the LinearConstitutiveModelCacheEntry object.
   */
  std::unique_ptr<DeformationGradientCacheEntry<T>> DoClone() const final {
    // Can't use std::make_unique because the copy constructor is protected.
    return std::unique_ptr<DeformationGradientCacheEntry<T>>(
        new LinearConstitutiveModelCacheEntry<T>(*this));
  }

  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::vector<Matrix3<T>> strain_;
  // Trace of `strain_`.
  std::vector<T> trace_strain_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::LinearConstitutiveModelCacheEntry);
