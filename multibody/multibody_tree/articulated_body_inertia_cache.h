#pragma once

#include <limits>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/articulated_body_inertia.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

/// This class is one of the cache entries in MultibodyTreeContext. It is used
/// to store the results from the first pass of the articulated body algorithm
/// for the computation of articulated body inertias. In addition to storing the
/// articulated body inertias themselves, this cache entry also serves to store
/// quantities computed during the first pass but that are also needed during
/// the second pass. Quantities in this cache entry are function of the
/// generalized position vector q only.
///
/// Articulated body inertia cache entries include:
/// - Articulated body inertia `P_B_W` of body B taken about Bo and expressed
///   in W.
/// - Articulated body inertia `Pplus_PB_W`, which can be thought of as the
///   articulated body inertia of parent body P as though it were inertialess,
///   but taken about Bo and expressed in W.
/// - LDLT factorization `ldlt_D_B` of the articulated body hinge inertia.
/// - The Kalman gain `g_PB_W` of the body.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
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
  const MatrixUpTo6<T>& get_g_PB_W(
      BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_PB_W_[body_node_index];
  }

  /// Mutable version of get_g_PB_W().
  MatrixUpTo6<T>& get_mutable_g_PB_W(
      BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return g_PB_W_[body_node_index];
  }


 private:
  // The type of the pools for storing articulated body inertias.
  typedef std::vector<ArticulatedBodyInertia<T>> ABI_PoolType;

  // The type of the pools for storing LDLT factorizations of matrices up to
  // 6x6.
  typedef std::vector<Eigen::LDLT<MatrixUpTo6<T>>> LDLT_MatrixUpTo6_PoolType;

  // The type of the pools for storing matrices up to 6x6.
  typedef std::vector<MatrixUpTo6<T>> MatrixUpTo6_PoolType;

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
    // TODO(bobbyluig): Initialize LDLT at world index to NaNs.
    g_PB_W_[world_index()] = Matrix6<T>::Constant(nan());
  }

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }


  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};

  // Pools.
  ABI_PoolType P_B_W_{};  // Indexed by BodyNodeIndex.
  ABI_PoolType Pplus_PB_W_{};  // Indexed by BodyNodeIndex.
  LDLT_MatrixUpTo6_PoolType ldlt_D_B_{};  // Indexed by BodyNodeIndex.
  MatrixUpTo6_PoolType g_PB_W_{};  // Indexed by BodyNodeIndex.
};

}  // namespace multibody
}  // namespace drake
