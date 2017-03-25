#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

template <typename T>
class PositionKinematicsCache {
 public:
  typedef eigen_aligned_std_vector<Isometry3<T>> X_PoolType;

  /// Returns a constant reference to the pose `X_WB` of the body `B`
  /// (associated with node @p body_node_id) as measured and expressed in the
  /// world frame `W`.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_WB` the pose of the the body frame `B` measured and
  ///                 expressed in the world frame `W`.
  const Isometry3<T>& get_X_WB(BodyNodeIndex body_node_id) const {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_WB_pool_[body_node_id];
  }

  /// Returns a mutable reference to the pose `X_WB` of the body `B`
  /// (associated with node @p body_node_id) as measured and expressed in the
  /// world frame `W`.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_WB` the pose of the the body frame `B` measured and
  ///                 expressed in the world frame `W`.
  Isometry3<T>& get_mutable_X_WB(BodyNodeIndex body_node_id) {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_WB_pool_[body_node_id];
  }

  // Do we need an argument here? get it at construction?
  // Obtain this from the constructor?
  //void Allocate(const MultibodyTreeTopology& tree_topology) {
  void Allocate(int num_body_nodes) {
    // Obtain this from the constructor?
    //num_nodes_ = tree_topology.get_num_body_nodes();
    num_nodes_ = num_body_nodes;

    X_WB_pool_.resize(num_nodes_);
    X_WB_pool_[world_index()] = Isometry3<T>::Identity();
  }

 private:
  int num_nodes_{0};
  // TODO(amcastro-tri): This MUST be indexed by BodyNodeIndex to avoid paging.
  // In this first prototype we index by BodyIndex just as a proof of concept to
  // introduce MultibodyTreeContext and a reasonable cache entry.
  X_PoolType X_WB_pool_;  // Indexed by BodyNodeIndex.
};

}  // namespace multibody
}  // namespace drake
