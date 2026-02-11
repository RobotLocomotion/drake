#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Data for performing efficient exact line search.

For line search, we consider the cost
  ℓ(v) = 0.5 vᵀAv - rᵀv + ℓᶜ(v)
along a search direction w as a function of the step size α:
  ℓ̃ (α) = ℓ(v + α⋅w).
Here A and r define the unconstrained momentum cost, and ℓᶜ(v) is the
constraints cost.

This struct holds precomputed terms (e.g., a, b, c) that allow us to efficiently
evaluate
  ℓ̃ (α) = aα²/2 + bα + c + ℓᶜ(v+α⋅w),
and its derivatives for different values of α.

@tparam_nonsymbolic_scalar */
template <typename T>
struct IcfSearchDirectionData {
  VectorX<T> w;  // Search direction.

  // Precomputed quadratic coefficients
  T a;  // = ‖w‖²/2 (A norm)
  T b;  // = w⋅(v+r)
  T c;  // = ‖v‖²/2 + r⋅v  (momentum cost at v)

  // Spatial velocities U = J⋅w for each body.
  EigenPool<Vector6<T>> U;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
