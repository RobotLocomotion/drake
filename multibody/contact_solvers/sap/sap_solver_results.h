#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// This struct stores the results from a computation performed with
// a SapSolver.
template <class T>
struct SapSolverResults {
  // Resizes `this` object to store the contact results for a problem with
  // `num_velocities` and `num_constraint_equations`.
  void Resize(int num_velocities, int num_constraint_equations) {
    v.resize(num_velocities);
    gamma.resize(num_constraint_equations);
    vc.resize(num_constraint_equations);
    j.resize(num_velocities);
  }

  // Vector of generalized velocities at the next time step.
  VectorX<T> v;

  // Constraints' impulses, of size `num_constraint_equations`.
  VectorX<T> gamma;

  // Constraints' velocities vc = J⋅v, where J is the contact Jacobian. Of size
  // `num_constraint_equations`.
  VectorX<T> vc;

  // Vector of generalized impulses j = Jᵀ⋅γ due to constraints, where J is the
  // contact Jacobian. Of size `num_velocities`.
  VectorX<T> j;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::contact_solvers::internal::SapSolverResults);
