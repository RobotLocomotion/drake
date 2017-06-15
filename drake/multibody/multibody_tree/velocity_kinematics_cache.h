#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

template <typename T>
class VelocityKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VelocityKinematicsCache)

  /// Constructs a velocity kinematics cache entry for the given
  /// MultibodyTreeTopology.
  explicit VelocityKinematicsCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.get_num_bodies()) {
    Allocate();
  }

  /// Returns a constant reference to the spatial velocity `V_WB` of the body B
  /// (associated with node @p body_node_index) as measured and expressed in the
  /// world frame W.
  /// @param[in] body_node_index The unique index for the computational
  ///                            BodyNode object associated with body B.
  /// @returns `X_WB` the pose of the the body frame B measured and
  ///                 expressed in the world frame W.
  const SpatialVelocity<T>& get_V_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_WB_pool_[body_node_index];
  }

  /// Mutable version of get_V_WB().
  SpatialVelocity<T>& get_mutable_V_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_WB_pool_[body_node_index];
  }

 private:
  // Pool types:
  // Pools store entries in the same order multibody tree nodes are
  // ordered in the tree, i.e. in BFT (Breadth-First Traversal) order. Therefore
  // clients of this class will access entries by BodyNodeIndex, see
  // `get_V_WB()` for instance.

  // The type of the pools for storing spatial velocities.
  typedef std::vector<SpatialVelocity<T>> SpatialVelocity_PoolType;

  // Allocates resources for this position kinematics cache.
  void Allocate() {
    V_WB_pool_.resize(num_nodes_);
    V_WB_pool_[world_index()].SetZero();  // World's velocity is always zero.
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};
  SpatialVelocity_PoolType V_WB_pool_;   // Indexed by BodyNodeIndex.
};

}  // namespace multibody
}  // namespace drake
