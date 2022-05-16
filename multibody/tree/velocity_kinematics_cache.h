#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Should cache qdot here.

// This class is one of the cache entries in the Context. It holds the
// kinematics results of computations that depend not only on the generalized
// positions of the system, but also on its generalized velocities.
// Velocity kinematics results include:
//
// - Spatial velocity `V_WB` for each body B in the model as measured and
//   expressed in the world frame W.
// - Spatial velocity `V_PB` for each body B in the model as measured and
//   expressed in the inboard (or parent) body frame P.
// - Spatial velocity `V_FMb_W` of frame Mb measured in the inboard frame F and
//   expressed in W. Mb is an "offset" frame rigidly fixed to M, whose axes are
//   parallel to M but whose origin is at Bo rather than Mo.
//
// @tparam_default_scalar
template <typename T>
class VelocityKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VelocityKinematicsCache)

  // Constructs a velocity kinematics cache entry for the given
  // MultibodyTreeTopology.
  // In Release builds specific entries are left uninitialized resulting in a
  // zero cost operation. However in Debug builds those entries are set to NaN
  // so that operations using this uninitialized cache entry fail fast, easing
  // bug detection.
  explicit VelocityKinematicsCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.num_bodies()) {
    Allocate();
    DRAKE_ASSERT_VOID(InitializeToNaN());
    // Sets defaults.
    V_WB_pool_[world_index()].SetZero();  // World's velocity is always zero.
    V_FM_pool_[world_index()].SetNaN();  // It must never be used.
    V_PB_W_pool_[world_index()].SetNaN();  // It must never be used.
  }

  // Initializes `this` %VelocityKinematicsCache as if all generalized
  // velocities of the corresponding MultibodyTree model were zero.
  void InitializeToZero() {
    for (BodyNodeIndex body_node_index(0); body_node_index < num_nodes_;
         ++body_node_index) {
      V_WB_pool_[body_node_index].SetZero();
      V_FM_pool_[body_node_index].SetZero();
      V_PB_W_pool_[body_node_index].SetZero();
    }
  }

  // Returns V_WB, body B's spatial velocity in the world frame W.
  // @param[in] body_node_index The unique index for the computational
  //                            BodyNode object associated with body B.
  // @retval V_WB_W body B's spatial velocity in the world frame W,
  // expressed in W (for point Bo, the body frame's origin).
  const SpatialVelocity<T>& get_V_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_WB_pool_[body_node_index];
  }

  // Mutable version of get_V_WB().
  SpatialVelocity<T>& get_mutable_V_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_WB_pool_[body_node_index];
  }

  // Returns a const reference to the across-mobilizer (associated with node
  // `body_node_index`) spatial velocity `V_FM` of the outboard frame M in the
  // inboard frame F.
  const SpatialVelocity<T>& get_V_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_FM_pool_[body_node_index];
  }

  // Mutable version of get_V_FM().
  SpatialVelocity<T>& get_mutable_V_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_FM_pool_[body_node_index];
  }

  // Returns a const reference to the spatial velocity `V_PB_W` of the
  // body B associated with node `body_node_index` in the parent node's body
  // frame P, expressed in the world frame W.
  const SpatialVelocity<T>& get_V_PB_W(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_PB_W_pool_[body_node_index];
  }

  // Mutable version of get_V_PB_W().
  SpatialVelocity<T>& get_mutable_V_PB_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_PB_W_pool_[body_node_index];
  }

 private:
  // Pools store entries in the same order multibody tree nodes are
  // ordered in the tree, i.e. in DFT (Depth-First Traversal) order. Therefore
  // clients of this class will access entries by BodyNodeIndex, see
  // `get_V_WB()` for instance.

  // The type of the pools for storing spatial velocities.
  typedef std::vector<SpatialVelocity<T>> SpatialVelocity_PoolType;

  // Allocates resources for this position kinematics cache.
  void Allocate() {
    V_WB_pool_.resize(num_nodes_);
    V_FM_pool_.resize(num_nodes_);
    V_PB_W_pool_.resize(num_nodes_);
  }

  // Initializes all pools to have NaN values to ease bug detection when entries
  // are accidentally left uninitialized.
  void InitializeToNaN() {
    for (BodyNodeIndex body_node_index(0); body_node_index < num_nodes_;
         ++body_node_index) {
      V_WB_pool_[body_node_index].SetNaN();
      V_FM_pool_[body_node_index].SetNaN();
      V_PB_W_pool_[body_node_index].SetNaN();
    }
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};
  SpatialVelocity_PoolType V_WB_pool_;
  SpatialVelocity_PoolType V_FM_pool_;
  SpatialVelocity_PoolType V_PB_W_pool_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::VelocityKinematicsCache)
