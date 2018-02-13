#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/articulated_body_inertia.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

/// This class is one of the cache entries in MultibodyTreeContext. It holds the
/// results of computations that are used in the recursive implementation of the
/// articulated body algorithm.
///
/// Articulated body inertia cache entries include:
/// - Articulated body inertia `Pplus_PB_W` for the articulated body subsystem
///   formed by all bodies outboard from body B, projected across its inboard
///   mobilizer, about point Bo, and expressed in the world frame W.
///
/// @note Another way to view `Pplus_PB_W` is to replace the parent body P with
///       a massless, inertialess, 0-size body Z that is connected by the same
///       inboard joint at Bo. The ordinary articulated body inertia `P_ZB_W`
///       of body Z about point Bo is the same as the quantity `Pplus_PB_W`.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
template<typename T>
class ArticulatedBodyInertiaCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyInertiaCache)

  /// Constructs an articulated body cache entry for the given
  /// MultibodyTreeTopology.
  explicit ArticulatedBodyInertiaCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.get_num_bodies()) {
    Allocate();
  }

  /// Articulated body inertia `Pplus_PB_W` for the articulated body subsystem
  /// formed by all bodies outboard from body B, projected across its inboard
  /// mobilizer to frame P, about point Bo, and expressed in the world frame W.
  const ArticulatedBodyInertia<T>& get_Pplus_PB_W(
      BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Pplus_PB_W_[body_node_index];
  }

  /// Mutable version of get_Pplus_PB_W().
  ArticulatedBodyInertia<T>& get_mutable_Pplus_PB_W(
      BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Pplus_PB_W_[body_node_index];
  }

 private:
  // The type of the pools for storing articulated body inertias.
  typedef std::vector<ArticulatedBodyInertia<T>> ABI_PoolType;

  // Allocates resources for this articulated body cache.
  void Allocate() {
    Pplus_PB_W_.resize(num_nodes_);
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};

  // Pools.
  ABI_PoolType Pplus_PB_W_{};  // Indexed by BodyNodeIndex.
};

}  // namespace multibody
}  // namespace drake
