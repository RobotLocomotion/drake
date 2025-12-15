#pragma once

#include <numeric>
#include <span>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Data pool for contact constraints. This data is updated at each solver
iteration, as opposed to the PatchConstraintsPool, which defines the
constraints themselves and is fixed for the lifetime of the optimization
problem.

Per-patch data holds num_patches() elements. Per-pair data is indexed by global
pair index, and holds num_pairs() >= num_patches() elements. See
PatchConstraintsPool for further indexing details.

@tparam_nonsymbolic_scalar */
template <typename T>
class PatchConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PatchConstraintsDataPool);

  /* Constructs an empty pool. */
  PatchConstraintsDataPool() = default;

  ~PatchConstraintsDataPool();

  /* Returns the number of patches in the pool. */
  int num_patches() const { return num_patches_; }

  /* Returns the total number of contact pairs among all patches. */
  int num_pairs() const { return num_pairs_; }

  /* Returns the number of constraints (patches) in the pool. */
  int num_constraints() const { return num_patches_; }

  /* Resizes the data pool to hold constraints of the given sizes.
  @param patch_size Number of contact pairs for the k-th patch. */
  void Resize(std::span<const int> patch_sizes);

  /* Returns the Hessian block for each patch. */
  const EigenPool<Matrix6<T>>& G_Bp_pool() const { return G_Bp_pool_; }
  EigenPool<Matrix6<T>>& mutable_G_Bp_pool() { return G_Bp_pool_; }

  /* Returns contact velocities for each pair. */
  const EigenPool<Vector3<T>>& v_AcBc_W_pool() const { return v_AcBc_W_; }
  EigenPool<Vector3<T>>& mutable_v_AcBc_W_pool() { return v_AcBc_W_; }

  /* Returns constraint impulses (gradients) for each patch. */
  const EigenPool<Vector6<T>>& Gamma_Bo_W_pool() const { return Gamma_Bo_W_; }
  EigenPool<Vector6<T>>& mutable_Gamma_Bo_W_pool() { return Gamma_Bo_W_; }

  /* Returns the cost contribution for each patch. */
  const std::vector<T>& cost_pool() const { return cost_pool_; }
  std::vector<T>& mutable_cost_pool() { return cost_pool_; }

  /* Returns the total cost over all patches. */
  const T& cost() const { return cost_; }
  T& mutable_cost() { return cost_; }

 private:
  int num_patches_{0};
  int num_pairs_{0};
  T cost_{NAN};

  // Data per patch.
  std::vector<T> cost_pool_;
  EigenPool<Matrix6<T>> G_Bp_pool_;   // Constraint Hessian for patch p.
  EigenPool<Vector6<T>> Gamma_Bo_W_;  // Spatial impulse on body B.

  // Data per patch and per pair.
  EigenPool<Vector3<T>> v_AcBc_W_;  // Contact velocity.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        PatchConstraintsDataPool);
