#pragma once

#include <numeric>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/**
 * Data pool for joint limit constraints (qu ≥ q ≥ ql). This data is updated at
 * each solver iteration, as opposed to the LimitConstraintsPool, which defines
 * the constraints themselves and is fixed for the lifetime of the optimization
 * problem.
 */
template <typename T>
class LimitConstraintsDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LimitConstraintsDataPool);

  using VectorXView = typename EigenPool<VectorX<T>>::ElementView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;
  using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;

  // Default constructor for an empty pool.
  LimitConstraintsDataPool() = default;

  /**
   * Resize the data pool to hold constraints of the given sizes.
   *
   * @param constraint_size The size (number of velocities) for each gain
   * constraint.
   */
  void Resize(const std::vector<int>& constraint_size) {
    gamma_lower_pool_.Resize(constraint_size);
    G_lower_pool_.Resize(constraint_size, constraint_size);
    gamma_upper_pool_.Resize(constraint_size);
    G_upper_pool_.Resize(constraint_size, constraint_size);

    // We will only every update the diagonal entries in Gk, so that
    // off-diagonal entires will forever remain zero.
    const int num_constraints = constraint_size.size();
    for (int k = 0; k < num_constraints; ++k) {
      G_lower_pool_[k].setZero();
      G_upper_pool_[k].setZero();
    }
  }

  // Returns the number of limit constraints.
  int num_constraints() const { return gamma_lower_pool_.size(); }

  // Hessian block G = -∂γ/∂v (diagonal) for lower limits.
  ConstMatrixXView G_lower(int k) const { return G_lower_pool_[k]; }
  MatrixXView G_lower(int k) { return G_lower_pool_[k]; }

  // Hessian block G = -∂γ/∂v (diagonal) for upper limits.
  ConstMatrixXView G_upper(int k) const { return G_upper_pool_[k]; }
  MatrixXView G_upper(int k) { return G_upper_pool_[k]; }

  // Constraint impulse γ = -∇ℓ(v) for lower limits.
  ConstVectorXView gamma_lower(int k) const { return gamma_lower_pool_[k]; }
  VectorXView gamma_lower(int k) { return gamma_lower_pool_[k]; }

  // Constraint impulse γ = -∇ℓ(v) for upper limits.
  ConstVectorXView gamma_upper(int k) const { return gamma_upper_pool_[k]; }
  VectorXView gamma_upper(int k) { return gamma_upper_pool_[k]; }

  // Total cost over all limit constraints (including both lower and upper).
  const T& cost() const { return cost_; }
  T& cost() { return cost_; }

 private:
  T cost_{0.0};
  EigenPool<VectorX<T>> gamma_lower_pool_;
  EigenPool<VectorX<T>> gamma_upper_pool_;

  // TODO(vincekurtz): consider storing as VectorX<T> since diagonal.
  EigenPool<MatrixX<T>> G_lower_pool_;  // G = -∂γ/∂v ≥ is Diagonal.
  EigenPool<MatrixX<T>> G_upper_pool_;  // G = -∂γ/∂v ≥ is Diagonal.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        LimitConstraintsDataPool);
