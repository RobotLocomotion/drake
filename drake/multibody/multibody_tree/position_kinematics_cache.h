#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

/// This class is one of the cache entries in MultibodyTreeContext. It holds the
/// kinematics results of computations that only depend on the generalized
/// positions of the system.
/// Kinematics results include:
/// - Body frame B poses X_WB measured and expressed in the world frame W.
/// - Pose X_FM of a mobilizer's outboard frame M measured and expressed in the
///   inboard frame F.
/// - Mobilizer's matrices H_FM (with F and M defined above) that map the
///   mobilizer's generalized velocities v to cross-joint spatial velocities
///   V_FM = H_FM * v.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template <typename T>
class PositionKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PositionKinematicsCache)

  /// Constructs a position kinematics cache entry for the given
  /// MultibodyTreeTopology.
  explicit PositionKinematicsCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.get_num_bodies()) {
    Allocate();
  }

  /// Returns a constant reference to the pose `X_WB` of the body `B`
  /// (associated with node @p body_node_index) as measured and expressed in the
  /// world frame `W`.
  /// @param[in] body_node_index The unique index for the computational
  ///                            BodyNode object associated with body `B`.
  /// @returns `X_WB` the pose of the the body frame `B` measured and
  ///                 expressed in the world frame `W`.
  const Isometry3<T>& get_X_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return X_WB_pool_[body_node_index];
  }

  /// Returns a mutable reference to the pose `X_WB` of the body `B`
  /// (associated with node @p body_node_index) as measured and expressed in the
  /// world frame `W`.
  /// @param[in] body_node_index The unique index for the computational
  ///                            BodyNode object associated with body `B`.
  /// @returns `X_WB` the pose of the the body frame `B` measured and
  ///                 expressed in the world frame `W`.
  Isometry3<T>& get_mutable_X_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return X_WB_pool_[body_node_index];
  }

 private:
  // Pool types:
  // Pools are stored as arrays with BSF (Breadth-First Search) ordering in
  // order to minimize cache misses (in this context we mean **hardware cache**)
  // during MultibodyTree traversals. Cache misses result in memory access with
  // a much longer latency.

  // The type of pools for storing poses.
  typedef eigen_aligned_std_vector<Isometry3<T>> X_PoolType;

  // Allocates resources for this position kinematics cache.
  void Allocate() {
    X_WB_pool_.resize(num_nodes_);
    X_WB_pool_[world_index()] = Isometry3<T>::Identity();
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};
  X_PoolType X_WB_pool_;  // Indexed by BodyNodeIndex.
};

}  // namespace multibody
}  // namespace drake
