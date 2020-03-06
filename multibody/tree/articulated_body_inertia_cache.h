#pragma once

#include <limits>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/articulated_body_inertia.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

/// This class is one of the cache entries in the Context. It is used
/// to store the results from the first pass of the articulated body algorithm
/// for the computation of articulated body inertias. Please refer to @ref
/// internal_forward_dynamics "Articulated Body Algorithm Forward Dynamics" for
/// further mathematical background and implementation details.
///
/// Articulated body inertia cache entries include:
/// - Articulated body inertia `P_B_W` of body B taken about Bo and expressed
///   in W.
/// - Articulated body inertia `Pplus_PB_W`, which can be thought of as the
///   articulated body inertia of parent body P as though it were inertialess,
///   but taken about Bo and expressed in W.
/// - LDLT factorization `ldlt_D_B` of the articulated body hinge inertia.
/// - The Kalman gain `g_PB_W = P_B_W * H_PB_W * D_B⁻¹`.
///
/// @tparam_default_scalar
template<typename T>
class ArticulatedBodyInertiaCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyInertiaCache)

  /// Constructs an articulated body cache entry for the given
  /// MultibodyTreeTopology.
  explicit ArticulatedBodyInertiaCache(const MultibodyTreeTopology& topology) :
      num_nodes_(topology.num_bodies()) {
    Allocate();
  }

  /// Articulated body inertia `P_B_W` of the body taken about Bo and expressed
  /// in W.
  const ArticulatedBodyInertia<T>& get_P_B_W(
      BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_B_W_[body_node_index];
  }

  /// Mutable version of get_P_B_W().
  ArticulatedBodyInertia<T>& get_mutable_P_B_W(
      BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return P_B_W_[body_node_index];
  }

  /// Articulated body inertia `Pplus_PB_W`, which can be thought of as the
  /// articulated body inertia of parent body P as though it were inertialess,
  /// but taken about Bo and expressed in W.
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

  /// LDLT factorization `ldlt_D_B` of the articulated body hinge inertia.
  const Eigen::LDLT<MatrixUpTo6<T>>& get_ldlt_D_B(
      BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return ldlt_D_B_[body_node_index];
  }

  /// Mutable version of get_ldlt_D_B().
  Eigen::LDLT<MatrixUpTo6<T>>& get_mutable_ldlt_D_B(
      BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return ldlt_D_B_[body_node_index];
  }

  /// The Kalman gain `g_PB_W` of the body.
  const Matrix6xUpTo6<T>& get_g_PB_W(
      BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_PB_W_[body_node_index];
  }

  /// Mutable version of get_g_PB_W().
  Matrix6xUpTo6<T>& get_mutable_g_PB_W(
      BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_PB_W_[body_node_index];
  }

 private:
  // Allocates resources for this articulated body cache.
  void Allocate() {
    P_B_W_.resize(num_nodes_);
    Pplus_PB_W_.resize(num_nodes_);
    ldlt_D_B_.resize(num_nodes_);
    g_PB_W_.resize(num_nodes_);

    // Initialize entries corresponding to world index to NaNs, since they
    // should not be used.
    P_B_W_[world_index()] = ArticulatedBodyInertia<T>();
    Pplus_PB_W_[world_index()] = ArticulatedBodyInertia<T>();
    g_PB_W_[world_index()] = Matrix6<T>::Constant(nan());
  }

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};

  // Pools indexed by BodyNodeIndex.
  std::vector<ArticulatedBodyInertia<T>> P_B_W_;
  std::vector<ArticulatedBodyInertia<T>> Pplus_PB_W_;
  std::vector<Eigen::LDLT<MatrixUpTo6<T>>> ldlt_D_B_;
  std::vector<Matrix6xUpTo6<T>> g_PB_W_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::ArticulatedBodyInertiaCache)
