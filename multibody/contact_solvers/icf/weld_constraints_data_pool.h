#pragma once

#include <cmath>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Stores data for weld constraints. This data is updated at each solver
iteration, as opposed to the WeldConstraintsPool, which helps define the
optimization problem.

Each weld constraint has a 6-dimensional impulse γ = (γᵣ, γₜ) ∈ ℝ⁶, where γᵣ
is the rotational component and γₜ is the translational component.

@tparam_nonsymbolic_scalar */
template <typename T>
class WeldConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldConstraintsDataPool);

  /* Constructs an empty pool. */
  WeldConstraintsDataPool() = default;

  ~WeldConstraintsDataPool();

  /* Resizes the pool, allocating memory only as necessary. */
  void Resize(int num_welds) { gamma_pool_.resize(num_welds); }

  /* Returns the number of weld constraints this data is for. */
  int num_constraints() const { return ssize(gamma_pool_); }

  /* Returns the constraint impulse γ ∈ ℝ⁶ for the k-th constraint in the pool.
  See WeldConstraintsPool for details. */
  const Vector6<T>& gamma(int k) const { return gamma_pool_[k]; }
  Vector6<T>& mutable_gamma(int k) { return gamma_pool_[k]; }

  /* Returns the total constraint cost ℓ(v) for all weld constraints in the
  pool. */
  const T& cost() const { return cost_; }
  T& mutable_cost() { return cost_; }

 private:
  T cost_{NAN};                         // Total cost over all weld constraints.
  std::vector<Vector6<T>> gamma_pool_;  // Constraint impulses, size 6 each.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        WeldConstraintsDataPool);
