#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/*
 * Stores data for coupler constraints. This data is updated at each solver
 * iteration, as opposed to the CouplerConstraintsPool, which helps define the
 * optimization problem.
 */
template <typename T>
class CouplerConstraintsDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CouplerConstraintsDataPool);

  /* Default constructor for an empty pool. */
  CouplerConstraintsDataPool() = default;

  /* Resize the pool, allocating memory only as necessary. */
  void Resize(int num_couplers) { gamma_pool_.resize(num_couplers); }

  /* Return the number of gain constraints this data is for. */
  int num_constraints() const { return gamma_pool_.size(); }

  /* The constraint impulse γ = -∇ℓ_c(v) */
  const T& gamma(int k) const { return gamma_pool_[k]; }
  T& gamma(int k) { return gamma_pool_[k]; }

  /* The constraint cost ℓ_c(v)*/
  const T& cost() const { return cost_; }
  T& cost() { return cost_; }

 private:
  T cost_{0.0};                // Total cost over all coupler constraints.
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
