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
class GainConstraintsDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GainConstraintsDataPool);

  using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;
  using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;
  using VectorXView = typename EigenPool<VectorX<T>>::ElementView;
  using ConstVectorXView = typename EigenPool<VectorX<T>>::ConstElementView;

  /* Default constructor for an empty pool. */
  GainConstraintsDataPool() = default;

  /* @param constraint_size The size (number of velocties) for the k-th gain
   * constraint. */
  void Resize(const std::vector<int>& constraint_size) {
    // const int num_equations =
    //     std::accumulate(constraint_size.begin(), constraint_size.end(), 0);
    // cost_pool_.resize(num_equations);
    gamma_pool_.Resize(constraint_size);
    G_pool_.Resize(constraint_size, constraint_size);

    // We will only every update the diagonal entries in Gk, so that
    // off-diagonal entires will forever remain zero.
    const int num_constraints = constraint_size.size();
    for (int k = 0; k < num_constraints; ++k) {
      G_pool_[k].setZero();
    }
  }

  /* Returns the number of gain constraints. */
  int num_constraints() const { return gamma_pool_.size(); }

  ConstMatrixXView G(int k) const { return G_pool_[k]; }
  MatrixXView G(int k) { return G_pool_[k]; }
  ConstVectorXView gamma(int k) const { return gamma_pool_[k]; }
  VectorXView gamma(int k) { return gamma_pool_[k]; }
  const T& cost() const { return cost_; }
  T& cost() { return cost_; }

 private:
  T cost_{0.0};  // Total cost over all gain constraints.
  // EigenPool<VectorX<T>> cost_pool_;   // Cost per constraint.
  EigenPool<VectorX<T>> gamma_pool_;  // Generalized impulses per constraint.
  EigenPool<MatrixX<T>> G_pool_;      // G = -∂γ/∂v ≥ is Diagonal.
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::
        GainConstraintsDataPool);
