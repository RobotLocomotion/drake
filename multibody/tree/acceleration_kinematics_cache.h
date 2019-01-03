#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

/// This class is one of the cache entries in MultibodyTreeContext. It holds the
/// kinematics results of computations that depend not only on the generalized
/// positions and generalized velocities, but also on the time derivatives of
/// the generalized coordinates.
/// Acceleration kinematics results include:
/// - Spatial acceleration `A_WB` for each body B in the model as measured and
///   expressed in the world frame W.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template <typename T>
class AccelerationKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationKinematicsCache)

  /// Constructs an acceleration kinematics cache entry for the given
  /// MultibodyTreeTopology.
  /// In Release builds specific entries are left uninitialized resulting in a
  /// zero cost operation. However in Debug builds those entries are set to NaN
  /// so that operations using this uninitialized cache entry fail fast, easing
  /// bug detection.
  explicit AccelerationKinematicsCache(const MultibodyTreeTopology& topology) {
    Allocate(topology);
    DRAKE_ASSERT_VOID(InitializeToNaN());
    // Sets defaults: drake::multibody::world_index() defines the unique index
    // to the world body and is defined in multibody_tree_indexes.h.
    // World's acceleration is always zero.
    A_WB_pool_[world_index()].SetZero();
  }

  /// Returns a constant reference to the spatial acceleration `A_WB` of the
  /// body B (associated with node @p body_node_index) as measured and expressed
  /// in the world frame W.
  /// This method aborts in Debug builds if `body_node_index` does not
  /// correspond to a valid BodyNode in the MultibodyTree.
  /// @param[in] body_node_index The unique index for the computational
  ///                            BodyNode object associated with body B.
  /// @retval A_WB the spatial acceleration of the body frame B measured and
  ///              expressed in the world frame W.
  const SpatialAcceleration<T>& get_A_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < get_num_nodes());
    return A_WB_pool_[body_node_index];
  }

  /// Mutable version of get_A_WB().
  SpatialAcceleration<T>& get_mutable_A_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < get_num_nodes());
    return A_WB_pool_[body_node_index];
  }

  /// Returns a const reference to the pool of body accelerations.
  /// The pool is returned as a `std::vector` of SpatialAcceleration objects
  /// ordered by BodyNodeIndex.
  /// Most users should not need to call this method.
  const std::vector<SpatialAcceleration<T>>& get_A_WB_pool() const {
    return A_WB_pool_;
  }

  /// Mutable version of get_A_WB_pool().
  std::vector<SpatialAcceleration<T>>& get_mutable_A_WB_pool() {
    return A_WB_pool_;
  }

 private:
  // Pools store entries in the same order that multibody tree nodes are
  // ordered in the tree, i.e. in BFT (Breadth-First Traversal) order. Therefore
  // clients of this class will access entries by BodyNodeIndex, see
  // `get_A_WB()` for instance.

  // The type of the pools for storing spatial accelerations.
  typedef std::vector<SpatialAcceleration<T>> SpatialAcceleration_PoolType;

  // Helper method to return the number of nodes in this multibody tree cache.
  // It eliminates having to use static_cast<int>() when requesting a pool size.
  int get_num_nodes() const {
    return static_cast<int>(A_WB_pool_.size());
  }

  // Allocates resources for this acceleration kinematics cache.
  void Allocate(const MultibodyTreeTopology& topology) {
    const int num_nodes = topology.num_bodies();
    A_WB_pool_.resize(num_nodes);
    DRAKE_ASSERT(static_cast<int>(A_WB_pool_.size()) == num_nodes);
  }

  // Initializes all pools to have NaN values to ease bug detection when entries
  // are accidentally left uninitialized.
  void InitializeToNaN() {
    for (BodyNodeIndex body_node_index(0); body_node_index < get_num_nodes();
         ++body_node_index) {
      A_WB_pool_[body_node_index].SetNaN();
    }
  }

  // Number of body nodes in the corresponding MultibodyTree.
  SpatialAcceleration_PoolType A_WB_pool_;   // Indexed by BodyNodeIndex.
};

}  // namespace internal

/// WARNING: This will be removed on or around 2019/03/01.
template <typename T>
using AccelerationKinematicsCache
DRAKE_DEPRECATED(
    "This public alias is deprecated, and will be removed around 2019/03/01.")
    = internal::AccelerationKinematicsCache<T>;

}  // namespace multibody
}  // namespace drake
