#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Stores data for coupler constraints. This data is updated at each solver
iteration, as opposed to the CouplerConstraintsPool, which helps define the
optimization problem.

@tparam_nonsymbolic_scalar */
template <typename T>
class CouplerConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CouplerConstraintsDataPool);

  /* Constructs an empty pool. */
  CouplerConstraintsDataPool() = default;

  ~CouplerConstraintsDataPool();

  /* Resizes the pool, allocating memory only as necessary. */
  void Resize(int num_couplers) {
    cost_pool_.resize(num_couplers);
    gamma_pool_.resize(num_couplers);
  }

  /* Returns the number of coupler constraints this data is for. */
  int num_constraints() const { return ssize(gamma_pool_); }

  /* Returns the constraint impulse γ = -∇ℓ(v) for the k-th constraint in the
  pool. See CouplerConstraintsPool for details. */
  const T& gamma(int k) const { return gamma_pool_[k]; }
  T& mutable_gamma(int k) { return gamma_pool_[k]; }

  /* Returns the total constraint cost ℓ(v) over all constraints in the pool.
  @pre all cost(k) are valid. */
  T cost() const { return std::reduce(cost_pool_.begin(), cost_pool_.end()); }

  /* Returns the constraint cost ℓ(v) for the k-th constraint in the pool. */
  const T& cost(int k) const { return cost_pool_[k]; }
  T& mutable_cost(int k) { return cost_pool_[k]; }

 private:
  std::vector<T> cost_pool_;   // cost per constraint.
  std::vector<T> gamma_pool_;  // Constraint impulses
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        CouplerConstraintsDataPool);
