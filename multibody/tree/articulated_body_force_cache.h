#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// This class is one of the cache entries in the Context. It stores the force
// and acceleration bias terms needed by the ABA forward dynamics. Please refer
// to @ref internal_forward_dynamics
// "Articulated Body Algorithm Forward Dynamics" for further mathematical
// background and implementation details. In particular, refer to @ref
// abi_and_bias_force "Articulated Body Inertia and Force Bias" for details on
// the force bias terms.
//
// @tparam_default_scalar
template <typename T>
class ArticulatedBodyForceCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyForceCache);

  // Constructs an %ArticulatedBodyForceCache object properly sized to
  // store the force bias terms for a model with the given `topology`.
  explicit ArticulatedBodyForceCache(const internal::SpanningForest& forest)
      : num_mobods_(forest.num_mobods()) {
    Allocate();
  }

  // The articulated body inertia residual force `Zplus_PB_W` for this mobilized
  // body projected across its inboard mobilizer to frame P.
  const SpatialForce<T>& get_Zplus_PB_W(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return Zplus_PB_W_[mobod_index];
  }

  // Mutable version of get_Zplus_PB_W().
  SpatialForce<T>& get_mutable_Zplus_PB_W(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return Zplus_PB_W_[mobod_index];
  }

  // The articulated body inertia innovations generalized force `e_B` for this
  // mobilized body's mobilizer.
  const VectorUpTo6<T>& get_e_B(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return e_B_[mobod_index];
  }

  // Mutable version of get_e_B().
  VectorUpTo6<T>& get_mutable_e_B(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return e_B_[mobod_index];
  }

 private:
  // Allocates resources for this articulated body cache.
  void Allocate() {
    Zplus_PB_W_.resize(num_mobods_);
    e_B_.resize(num_mobods_);
  }

  // Number of mobilized bodies in the corresponding MultibodyTree.
  int num_mobods_{0};

  // Pools indexed by MobodIndex.
  std::vector<SpatialForce<T>> Zplus_PB_W_;
  std::vector<VectorUpTo6<T>> e_B_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ArticulatedBodyForceCache);
