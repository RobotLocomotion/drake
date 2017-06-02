#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

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

#if 0
  /// @returns a constant reference to the spatial velocity of the body node's
  /// body `B` measured in its parent body `P` and expressed in the world
  /// frame `W`.
  const GeneralSpatialVector<T>& get_V_PB_W(BodyNodeIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return V_PB_W_pool_[body_id];
  }

  /// @returns a mutable reference to the spatial velocity of the body node's
  /// body `B` measured in its parent body `P` and expressed in the world
  /// frame `W`.
  GeneralSpatialVector<T>& get_mutable_V_PB_W(BodyNodeIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return V_PB_W_pool_[body_id];
  }
#endif

  const SpatialVelocity<T>& get_V_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_WB_pool_[body_node_index];
  }

  SpatialVelocity<T>& get_mutable_V_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return V_WB_pool_[body_node_index];
  }

 private:
  // Pool types:
  // Pools store entries in the same order multibody tree nodes are
  // ordered in the tree, i.e. in BFT (Breadth-First Traversal) order. Therefore
  // clients of this class will access entries by BodyNodeIndex, see
  // `get_X_WB()` for instance.

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
