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

  /// Returns a constant reference to the pose `X_WB` of the body B
  /// (associated with node @p body_node_index) as measured and expressed in the
  /// world frame W.
  /// @param[in] body_node_index The unique index for the computational
  ///                            BodyNode object associated with body B.
  /// @returns `X_WB` the pose of the the body frame B measured and
  ///                 expressed in the world frame W.
  const Isometry3<T>& get_X_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return X_WB_pool_[body_node_index];
  }

  /// See documentation on the const version get_X_WB() for details.
  Isometry3<T>& get_mutable_X_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return X_WB_pool_[body_node_index];
  }

  /// Returns a const reference to the pose `X_PB` of the body frame B
  /// as measured and expressed in its parent body frame P.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body B.
  /// @returns `X_PB` a const reference to the pose of the the body frame B
  ///                 measured and expressed in the parent body frame P.
  const Isometry3<T>& get_X_PB(BodyNodeIndex body_node_id) const {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_PB_pool_[body_node_id];
  }

  /// See documentation on the const version get_X_PB() for details.
  Isometry3<T>& get_mutable_X_PB(BodyNodeIndex body_node_id) {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_PB_pool_[body_node_id];
  }

  /// For the mobilizer associated with the body node indexed by
  /// `body_node_index`, this method returns a const reference to the pose
  /// `X_FM` of the outboard frame M as measured and expressed in the inboard
  /// frame F.
  ///
  /// @param[in] body_node_index The unique index for the computational
  ///                            BodyNode object associated with the mobilizer
  ///                            of interest.
  /// @returns A const reference to the pose `X_FM` of the outboard frame M
  ///          as measured and expressed in the inboard frame F.
  const Isometry3<T>& get_X_FM(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return X_FM_pool_[body_node_index];
  }

  /// See documentation on the const version get_X_FM() for details.
  Isometry3<T>& get_mutable_X_FM(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return X_FM_pool_[body_node_index];
  }

 private:
  // Pool types:
  // Pools store entries in the same order multibody tree nodes are
  // ordered in the tree, i.e. in BFT (Breadth-First Traversal) order. Therefore
  // clients of this class will access entries by BodyNodeIndex, see
  // `get_X_WB()` for instance.

  // The type of pools for storing poses.
  typedef eigen_aligned_std_vector<Isometry3<T>> X_PoolType;

  // Allocates resources for this position kinematics cache.
  void Allocate() {
    X_WB_pool_.resize(num_nodes_);
    X_WB_pool_[world_index()] = Isometry3<T>::Identity();

    X_PB_pool_.resize(num_nodes_);
    X_PB_pool_[world_index()] = NaNPose();  // It should never be used.

    X_FM_pool_.resize(num_nodes_);
    X_FM_pool_[world_index()] = NaNPose();  // It should never be used.

    X_MB_pool_.resize(num_nodes_);
    X_MB_pool_[world_index()] = NaNPose();  // It should never be used.
  }

  // Helper method to initialize poses to NaN.
  static Isometry3<T> NaNPose() {
    return Isometry3<T>(
        Matrix4<T>::Constant(Eigen::NumTraits<double>::quiet_NaN()));
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};
  X_PoolType X_WB_pool_;  // Indexed by BodyNodeIndex.
  X_PoolType X_PB_pool_;  // Indexed by BodyNodeIndex.
  X_PoolType X_FM_pool_;  // Indexed by BodyNodeIndex.
  X_PoolType X_MB_pool_;  // Indexed by BodyNodeIndex.
};

}  // namespace multibody
}  // namespace drake
