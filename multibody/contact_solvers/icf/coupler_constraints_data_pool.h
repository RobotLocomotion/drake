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
  void Resize(int num_couplers) { gamma_pool_.resize(num_couplers); }

  /* Returns the number of coupler constraints this data is for. */
  int num_constraints() const { return ssize(gamma_pool_); }

  /* Returns the constraint impulse γ = -∇ℓ(v) for the k-th constraint in the
  pool. See CouplerConstraintsPool for details. */
  const T& gamma(int k) const { return gamma_pool_[k]; }
  T& mutable_gamma(int k) { return gamma_pool_[k]; }

  /* Returns the total constraint cost ℓ(v) for all coupler constraints in the
  pool. */
  const T& cost() const { return cost_; }
  T& mutable_cost() { return cost_; }

 private:
  T cost_{NAN};                // Total cost over all coupler constraints.
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
