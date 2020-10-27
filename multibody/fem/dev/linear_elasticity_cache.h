#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/fem/dev/constitutive_model_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cached quantities for the LinearElasticity constitutive model.
 See LinearElasticity for how the cache is used. See ConstitutiveModelCache for
 more about cached quantities for constitutive models.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class LinearElasticityCache : public ConstitutiveModelCache<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearElasticityCache);

  /** Constructs a LinearElasticityCache with the given element index and
   number of quadrature locations.
   @param element_index The index of the element that `this`
   ConstitutiveModelCache lives on.
   @param num_quads The number of quadrature locations at which cached
   quantities need to be evaluated.
   @pre `num_quads` must be positive. */
  LinearElasticityCache(ElementIndex element_index, int num_quads)
      : ConstitutiveModelCache<T>(element_index, num_quads),
        strain_(num_quads),
        trace_strain_(num_quads) {}

  /** Updates the cached quantities with the given deformation gradients.
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations in this element.
   @pre The size of `F` must be the same as `num_quads()`. */
  void UpdateCache(const std::vector<Matrix3<T>>& F) final;

  /** Returns the infinitesimal strains evaluated at the quadrature locations in
   this element. */
  const std::vector<Matrix3<T>>& strain() const { return strain_; }

  /** Returns the traces of the infinitesimal strains evaluated at the
   quadrature locations in this element. */
  const std::vector<T>& trace_strain() const { return trace_strain_; }

 private:
  // Infinitesimal strain = 0.5 * (F + Fᵀ) - I.
  std::vector<Matrix3<T>> strain_;
  // Trace of `strain_`.
  std::vector<T> trace_strain_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::LinearElasticityCache);
