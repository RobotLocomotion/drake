#pragma once

#include <limits>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/linear_solve.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/articulated_body_inertia.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// This class is one of the cache entries in the Context. It is used
// to store the results from the first pass of the articulated body algorithm
// for the computation of articulated body inertias. Please refer to @ref
// internal_forward_dynamics "Articulated Body Algorithm Forward Dynamics" for
// further mathematical background and implementation details.
//
// Articulated body inertia cache entries include:
// - Articulated body inertia `P_B_W` of body B taken about Bo and expressed
//   in W.
// - Articulated body inertia `Pplus_PB_W`, which can be thought of as the
//   articulated body inertia of parent body P as though it were inertialess,
//   but taken about Bo and expressed in W.
// - LLT factorization `llt_D_B` of the articulated body hinge inertia.
// - The Kalman gain `g_PB_W = P_B_W * H_PB_W * D_B⁻¹`.
//
// @tparam_default_scalar
template <typename T>
class ArticulatedBodyInertiaCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyInertiaCache);

  // Constructs an articulated body cache entry for the given
  // MultibodyTreeTopology.
  explicit ArticulatedBodyInertiaCache(const internal::SpanningForest& forest)
      : num_mobods_(forest.num_mobods()) {
    Allocate();
  }

  // Articulated body inertia `P_B_W` of the body taken about Bo and expressed
  // in W.
  const ArticulatedBodyInertia<T>& get_P_B_W(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return P_B_W_[mobod_index];
  }

  // Mutable version of get_P_B_W().
  ArticulatedBodyInertia<T>& get_mutable_P_B_W(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return P_B_W_[mobod_index];
  }

  // Articulated body inertia `Pplus_PB_W`, which can be thought of as the
  // articulated body inertia of parent body P as though it were inertialess,
  // but taken about Bo and expressed in W.
  const ArticulatedBodyInertia<T>& get_Pplus_PB_W(
      MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return Pplus_PB_W_[mobod_index];
  }

  // Mutable version of get_Pplus_PB_W().
  ArticulatedBodyInertia<T>& get_mutable_Pplus_PB_W(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return Pplus_PB_W_[mobod_index];
  }

  // LLT factorization `llt_D_B` of the articulated body hinge inertia.
  const math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& get_llt_D_B(
      MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return llt_D_B_[mobod_index];
  }

  // Mutable version of get_llt_D_B().
  math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>& get_mutable_llt_D_B(
      MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return llt_D_B_[mobod_index];
  }

  // The Kalman gain `g_PB_W` of the body.
  const Matrix6xUpTo6<T>& get_g_PB_W(MobodIndex mobod_index) const {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return g_PB_W_[mobod_index];
  }

  // Mutable version of get_g_PB_W().
  Matrix6xUpTo6<T>& get_mutable_g_PB_W(MobodIndex mobod_index) {
    DRAKE_ASSERT(0 <= mobod_index && mobod_index < num_mobods_);
    return g_PB_W_[mobod_index];
  }

 private:
  // Allocates resources for this articulated body cache.
  void Allocate() {
    P_B_W_.resize(num_mobods_);
    Pplus_PB_W_.resize(num_mobods_);
    llt_D_B_.resize(num_mobods_);
    g_PB_W_.resize(num_mobods_);

    // Initialize entries corresponding to world index to NaNs, since they
    // should not be used.
    P_B_W_[world_mobod_index()] = ArticulatedBodyInertia<T>();
    Pplus_PB_W_[world_mobod_index()] = ArticulatedBodyInertia<T>();
    g_PB_W_[world_mobod_index()] = Matrix6<T>::Constant(nan());
  }

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Number of mobilized bodies in the corresponding multibody forest.
  int num_mobods_{0};

  // Pools indexed by MobodIndex.
  std::vector<ArticulatedBodyInertia<T>> P_B_W_;
  std::vector<ArticulatedBodyInertia<T>> Pplus_PB_W_;
  std::vector<math::LinearSolver<Eigen::LLT, MatrixUpTo6<T>>> llt_D_B_;
  std::vector<Matrix6xUpTo6<T>> g_PB_W_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::ArticulatedBodyInertiaCache);
