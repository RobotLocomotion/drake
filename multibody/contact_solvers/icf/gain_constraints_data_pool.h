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
 * Data pool for torque-limited actuation constraints τ = clamp(−K⋅v + b, e).
 * This data is updated at each solver iteration, as opposed to the
 * GainConstraintsPool, which defines the constraints themselves and is fixed
 * for the lifetime of the optimization problem.
 */
template <typename T>
class GainConstraintsDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GainConstraintsDataPool);

  using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;
  using VectorXView = typename EigenPool<VectorX<T>>::ElementView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;

  // Default constructor for an empty pool.
  GainConstraintsDataPool() = default;

  /**
   * Resize the data pool to hold constraints of the given sizes.
   *
   * @param constraint_size The size (number of velocities) for each gain
   * constraint.
   */
  void Resize(const std::vector<int>& constraint_size) {
    gamma_pool_.Resize(constraint_size);
    G_pool_.Resize(constraint_size, constraint_size);

    // We will only ever update the diagonal entries in Gk, so that off-diagonal
    // entires will forever remain zero.
    const int num_constraints = constraint_size.size();
    for (int k = 0; k < num_constraints; ++k) {
      G_pool_[k].setZero();
    }
  }

  // Number of gain constraints.
  int num_constraints() const { return gamma_pool_.size(); }

  // Hessian block G = -∂γ/∂v (diagonal).
  ConstMatrixXView G(int k) const { return G_pool_[k]; }
  MatrixXView G(int k) { return G_pool_[k]; }

  // Constraint impulse γ = -∇ℓ(v).
  ConstVectorXView gamma(int k) const { return gamma_pool_[k]; }
  VectorXView gamma(int k) { return gamma_pool_[k]; }

  // Constraint cost ℓ(v).
  const T& cost() const { return cost_; }
  T& cost() { return cost_; }

 private:
  T cost_{0.0};                       // Total cost over all gain constraints.
  EigenPool<VectorX<T>> gamma_pool_;  // Generalized impulses per constraint.

  // TODO(vincekurtz): consider storing as VectorX<T> since diagonal.
  EigenPool<MatrixX<T>> G_pool_;  // G = -∂γ/∂v ≥ is diagonal.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        GainConstraintsDataPool);
