#pragma once

#include <numeric>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class PatchConstraintsDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintsDataPool);

  // The number of patches in the pool.
  int num_patches() const { return num_patches_; }

  // The total number of contact pairs among all patches.
  int num_pairs() const { return num_pairs_; }

  /* Default constructor for an empty pool. */
  PatchConstraintsDataPool() = default;

  PatchConstraintsDataPool(int num_velocities,
                           const std::vector<int>& patch_size) {
    Resize(num_velocities, patch_size);
  }

  /* @param patch_size Number of contact pairs for the k-th patch.
     @param num_velocities Number of velocities for the k-th patch. */
  void Resize(int, const std::vector<int>& patch_size) {
    num_patches_ = ssize(patch_size);
    num_pairs_ = std::accumulate(patch_size.begin(), patch_size.end(), 0);

    // Data per patch.
    cost_pool_.resize(num_patches_);
    G_Bp_pool_.Resize(num_patches_);
    Gamma_Bo_W_.Resize(num_patches_);

    // Data per pair.
    v_AcBc_W_.Resize(num_pairs_);
  }

  EigenPool<Matrix6<T>>& G_Bp_pool() { return G_Bp_pool_; }
  const EigenPool<Matrix6<T>>& G_Bp_pool() const { return G_Bp_pool_; }
  const EigenPool<Vector3<T>>& v_AcBc_W_pool() const { return v_AcBc_W_; }
  EigenPool<Vector3<T>>& v_AcBc_W_pool() { return v_AcBc_W_; }
  const EigenPool<Vector6<T>>& Gamma_Bo_W_pool() const { return Gamma_Bo_W_; }
  EigenPool<Vector6<T>>& Gamma_Bo_W_pool() { return Gamma_Bo_W_; }

  const T& cost() const { return cost_; }
  T& cost() { return cost_; }
  const std::vector<T>& cost_pool() const { return cost_pool_; }
  std::vector<T>& cost_pool() { return cost_pool_; }

 private:
  int num_patches_{0};
  int num_pairs_{0};

  T cost_{0.0};

  // Data per patch.
  std::vector<T> cost_pool_;
  EigenPool<Matrix6<T>> G_Bp_pool_;
  EigenPool<Vector6<T>> Gamma_Bo_W_;  // Spatial impulse on body B.

  /* Data per patch and per pair. */
  EigenPool<Vector3<T>> v_AcBc_W_;  // Contact velocity.
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::
        PatchConstraintsDataPool);
