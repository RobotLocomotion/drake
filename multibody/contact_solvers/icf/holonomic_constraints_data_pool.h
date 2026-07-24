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

/* Stores per-iteration solver data for a pool of holonomic constraints: the
total cost over all constraints and the per-constraint impulses, γ ∈ ℝᴺ (where
each constraint has N equations). This is the data companion to
HolonomicConstraintsPool<T, N>, updated at each solver iteration.

@tparam_nonsymbolic_scalar
@tparam N The number of equations per constraint. */
template <typename T, int N>
class HolonomicConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HolonomicConstraintsDataPool);

  using ConstraintVector = Eigen::Matrix<T, N, 1>;

  HolonomicConstraintsDataPool() = default;
  ~HolonomicConstraintsDataPool();

  /* Resizes the pool, allocating memory only as necessary. */
  void Resize(int num_constraints);

  /* Returns the number of constraints this data is for. */
  int num_constraints() const { return ssize(gamma_pool_); }

  /* Returns the constraint impulse γ ∈ ℝᴺ for the k-th constraint. */
  const ConstraintVector& gamma(int k) const { return gamma_pool_[k]; }
  ConstraintVector& mutable_gamma(int k) { return gamma_pool_[k]; }

  /* Returns the total constraint cost ℓ(v) over all constraints in the pool. */
  const T& cost() const { return cost_; }
  T& mutable_cost() { return cost_; }

 private:
  T cost_{NAN};  // Total cost over all constraints.
  std::vector<ConstraintVector> gamma_pool_;  // Constraint impulses.
};

/* Named data pools for the concrete holonomic constraints. Each is just the
generic pool at the constraint's equation count (weld: 6, ball: 3). */
template <typename T>
using WeldConstraintsDataPool = HolonomicConstraintsDataPool<T, 6>;
template <typename T>
using BallConstraintsDataPool = HolonomicConstraintsDataPool<T, 3>;

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
