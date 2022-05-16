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

// This class is one of the cache entries in the Context. It holds the
// kinematics results of computations that depend not only on the generalized
// positions and generalized velocities, but also on the time derivatives of
// the generalized coordinates.
// Acceleration kinematics results include:
// - Spatial acceleration `A_WB` for each body B in the model as measured and
//   expressed in the world frame W.
// - Generalized accelerations `vdot` for the entire model.
//
// @tparam_default_scalar
template <typename T>
class AccelerationKinematicsCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationKinematicsCache)

  // Constructs an acceleration kinematics cache entry for the given
  // MultibodyTreeTopology.
  // In Release builds specific entries are left uninitialized resulting in a
  // zero cost operation. However in Debug builds those entries are set to NaN
  // so that operations using this uninitialized cache entry fail fast, easing
  // bug detection.
  explicit AccelerationKinematicsCache(const MultibodyTreeTopology& topology) {
    Allocate(topology);
    DRAKE_ASSERT_VOID(InitializeToNaN());
    // Sets defaults: drake::multibody::world_index() defines the unique index
    // to the world body and is defined in multibody_tree_indexes.h.
    // World's acceleration is always zero.
    A_WB_pool_[world_index()].SetZero();
    vdot_.setZero();
  }

  // For the body B associated with node @p body_node_index, returns A_WB,
  // body B's spatial acceleration in the world frame W.
  // This method aborts in Debug builds if `body_node_index` does not
  // correspond to a valid BodyNode in the MultibodyTree.
  // @param[in] body_node_index The unique index for the computational
  //                            BodyNode object associated with body B.
  // @retval A_WB_W body B's spatial acceleration in the world frame W,
  // expressed in W (for point Bo, the body's origin).
  const SpatialAcceleration<T>& get_A_WB(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < get_num_nodes());
    return A_WB_pool_[body_node_index];
  }

  // Mutable version of get_A_WB().
  SpatialAcceleration<T>& get_mutable_A_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < get_num_nodes());
    return A_WB_pool_[body_node_index];
  }

  // Returns a const reference to the pool of body accelerations.
  // The pool is returned as a `std::vector` of SpatialAcceleration objects
  // ordered by BodyNodeIndex.
  // Most users should not need to call this method.
  const std::vector<SpatialAcceleration<T>>& get_A_WB_pool() const {
    return A_WB_pool_;
  }

  // Mutable version of get_A_WB_pool().
  std::vector<SpatialAcceleration<T>>& get_mutable_A_WB_pool() {
    return A_WB_pool_;
  }

  // Returns a constant reference to the generalized accelerations `vdot` for
  // the entire model.
  const VectorX<T>& get_vdot() const {
    return vdot_;
  }

  // Mutable version of get_vdot().
  VectorX<T>& get_mutable_vdot() {
    return vdot_;
  }

 private:
  // Pools store entries in the same order that multibody tree nodes are
  // ordered in the tree, i.e. in DFT (Depth-First Traversal) order. Therefore
  // clients of this class will access entries by BodyNodeIndex, see
  // `get_A_WB()` for instance.

  // Helper method to return the number of nodes in this multibody tree cache.
  // It eliminates having to use static_cast<int>() when requesting a pool size.
  int get_num_nodes() const {
    return static_cast<int>(A_WB_pool_.size());
  }

  // Allocates resources for this acceleration kinematics cache.
  void Allocate(const MultibodyTreeTopology& topology) {
    const int num_nodes = topology.num_bodies();
    A_WB_pool_.resize(num_nodes);
    const int num_velocities = topology.num_velocities();
    vdot_.resize(num_velocities);
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
  std::vector<SpatialAcceleration<T>> A_WB_pool_;  // Indexed by BodyNodeIndex.
  VectorX<T> vdot_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::AccelerationKinematicsCache)
