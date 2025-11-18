#pragma once

#include <span>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Data pool for joint limit constraints (qu ≥ q ≥ ql). This data is updated at
each solver iteration, as opposed to the LimitConstraintsPool, which defines
the constraints themselves and is fixed for the lifetime of the optimization
problem. */
template <typename T>
class LimitConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LimitConstraintsDataPool);

  using VectorXView = typename EigenPool<VectorX<T>>::MatrixView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  LimitConstraintsDataPool() = default;

  /* Resizes the data pool to hold constraints of the given sizes.
  @param constraint_size The size (number of velocities) for each gain
                         constraint. */
  void Resize(std::span<const int> constraint_size);

  /* Returns the number of limit constraints. */
  int num_constraints() const { return gamma_lower_pool_.size(); }

  /* Returns the Hessian block G = -∂γ/∂v (diagonal) for lower limits. */
  ConstVectorXView G_lower(int k) const { return G_lower_pool_[k]; }
  VectorXView G_lower(int k) { return G_lower_pool_[k]; }

  /* Returns the Hessian block G = -∂γ/∂v (diagonal) for upper limits. */
  ConstVectorXView G_upper(int k) const { return G_upper_pool_[k]; }
  VectorXView G_upper(int k) { return G_upper_pool_[k]; }

  /* Returns the constraint impulse γ = -∇ℓ(v) for lower limits. */
  ConstVectorXView gamma_lower(int k) const { return gamma_lower_pool_[k]; }
  VectorXView gamma_lower(int k) { return gamma_lower_pool_[k]; }

  /* Returns the constraint impulse γ = -∇ℓ(v) for upper limits. */
  ConstVectorXView gamma_upper(int k) const { return gamma_upper_pool_[k]; }
  VectorXView gamma_upper(int k) { return gamma_upper_pool_[k]; }

  /* Returns the total cost over all limit constraints (including both lower and
  upper limits). */
  const T& cost() const { return cost_; }
  T& cost() { return cost_; }

 private:
  T cost_{0.0};
  EigenPool<VectorX<T>> gamma_lower_pool_;
  EigenPool<VectorX<T>> gamma_upper_pool_;
  EigenPool<VectorX<T>> G_lower_pool_;  // G = -∂γ/∂v ≥ 0 is Diagonal.
  EigenPool<VectorX<T>> G_upper_pool_;  // G = -∂γ/∂v ≥ 0 is Diagonal.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        LimitConstraintsDataPool);
