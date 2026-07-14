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

/* Stores data for ball constraints. This data is updated at each solver
iteration, as opposed to the BallConstraintsPool, which helps define the
optimization problem.

Each ball constraint has a 3-dimensional impulse γ ∈ ℝ³, applied at the
midpoint M between the constraint points P and Q.

@tparam_nonsymbolic_scalar */
template <typename T>
class BallConstraintsDataPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BallConstraintsDataPool);

  /* Constructs an empty pool. */
  BallConstraintsDataPool() = default;

  ~BallConstraintsDataPool();

  /* Resizes the pool, allocating memory only as necessary. */
  void Resize(int num_balls) { gamma_pool_.resize(num_balls); }

  /* Returns the number of ball constraints this data is for. */
  int num_constraints() const { return ssize(gamma_pool_); }

  /* Returns the constraint impulse γ ∈ ℝ³ for the k-th constraint in the pool.
  See BallConstraintsPool for details. */
  const Vector3<T>& gamma(int k) const { return gamma_pool_[k]; }
  Vector3<T>& mutable_gamma(int k) { return gamma_pool_[k]; }

  /* Returns the total constraint cost ℓ(v) for all ball constraints in the
  pool. */
  const T& cost() const { return cost_; }
  T& mutable_cost() { return cost_; }

 private:
  T cost_{NAN};                         // Total cost over all ball constraints.
  std::vector<Vector3<T>> gamma_pool_;  // Constraint impulses, size 3 each.
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        BallConstraintsDataPool);
