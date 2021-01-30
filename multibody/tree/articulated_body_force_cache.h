#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

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
template<typename T>
class ArticulatedBodyForceCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyForceCache)

  // Constructs an %ArticulatedBodyForceCache object properly sized to
  // store the force bias terms for a model with the given `topology`.
  explicit ArticulatedBodyForceCache(
      const MultibodyTreeTopology& topology) :
      num_nodes_(topology.num_bodies()) {
    Allocate();
  }

  // The articulated body inertia residual force `Zplus_PB_W` for this body
  // projected across its inboard mobilizer to frame P.
  const SpatialForce<T>& get_Zplus_PB_W(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Zplus_PB_W_[body_node_index];
  }

  // Mutable version of get_Zplus_PB_W().
  SpatialForce<T>& get_mutable_Zplus_PB_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Zplus_PB_W_[body_node_index];
  }

  // The articulated body inertia innovations generalized force `e_B` for this
  // body's mobilizer.
  const VectorUpTo6<T>& get_e_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return e_B_[body_node_index];
  }

  // Mutable version of get_e_B().
  VectorUpTo6<T>& get_mutable_e_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return e_B_[body_node_index];
  }

 private:
  // Allocates resources for this articulated body cache.
  void Allocate() {
    Zplus_PB_W_.resize(num_nodes_);
    e_B_.resize(num_nodes_);
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};

  // Pools indexed by BodyNodeIndex.
  std::vector<SpatialForce<T>> Zplus_PB_W_;
  std::vector<VectorUpTo6<T>> e_B_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ArticulatedBodyForceCache)
