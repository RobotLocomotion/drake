#pragma once

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

/* Data pool for torque-limited actuation constraints τ = clamp(−K⋅v + b, e).
This data is updated at each solver iteration, as opposed to the
GainConstraintsPool, which defines the constraints themselves and is fixed for
the lifetime of the optimization problem.

@tparam_nonsymbolic_scalar */
template <typename T>
class GainConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GainConstraintsDataPool);

  using VectorXView = typename EigenPool<VectorX<T>>::MatrixView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  GainConstraintsDataPool() = default;

  ~GainConstraintsDataPool();

  /* Resizes the data pool to hold constraints of the given sizes.
  @param constraint_size The number of velocities for each gain constraint. */
  void Resize(std::span<const int> constraint_size);

  /* Returns the number of gain constraints this data pool is for. */
  int num_constraints() const { return gamma_pool_.size(); }

  /* Returns the constraint impulse γ = -∇ℓ(v) for the k-th constraint in the
  pool.  See GainConstraintsPool for details. */
  ConstVectorXView gamma(int k) const { return gamma_pool_[k]; }
  VectorXView mutable_gamma(int k) { return gamma_pool_[k]; }

  /* Returns the total constraint cost ℓ(v) over all constraints in the pool.
  @pre all cost(k) are valid. */
  T ComputeCost() const {
    return std::reduce(cost_pool_.begin(), cost_pool_.end());
  }

  /* Returns the constraint cost ℓ(v) for the k-th constraint in the pool. */
  const T& cost(int k) const { return cost_pool_[k]; }
  T& mutable_cost(int k) { return cost_pool_[k]; }

  /* Returns the (diagonal) Hessian block G = -∂γ/∂v for the k-th constraint in
  the pool. */
  ConstVectorXView G(int k) const { return G_pool_[k]; }
  VectorXView mutable_G(int k) { return G_pool_[k]; }

 private:
  std::vector<T> cost_pool_;          // cost per constraint.
  EigenPool<VectorX<T>> gamma_pool_;  // Generalized impulses per constraint.
  EigenPool<VectorX<T>> G_pool_;      // Diagonal Hessians G = -∂γ/∂v ≥ 0.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        GainConstraintsDataPool);
