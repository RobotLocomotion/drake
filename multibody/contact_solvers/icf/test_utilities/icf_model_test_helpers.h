#pragma once

#include <vector>

#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Sets up a simple dummy model, without any constraints. This can create two
versions of the same model: one with everything stuffed into a single clique,
and one with three separate cliques. */
template <typename T>
void MakeUnconstrainedModel(IcfModel<T>* model, bool single_clique = false,
                            double time_step = 0.01);

/* Makes a model reducible by setting the ReductionParameters such that
`dofs_to_remove` would be removed in the reduced model.

For test purposes, it is valid to pass an empty `dofs_to_remove`. The resulting
model's reduction parameters are valid, so `model.ReduceInto()` can be used to
effectively copy the model. However, the described problem is not smaller than
the original, so `model.is_reducible()` would return false.

@pre `model` != nullptr.
@pre `dofs_to_remove` is a subsequence of the velocity indices:
  * all non-negative,
  * all < `model.num_velocities()`,
  * sequence is sorted,
  * elements are unique.
*/
template <typename T>
void MakeModelReducible(IcfModel<T>* model,
                        const std::vector<int>& dofs_to_remove);

/* Adds a coupler constraint to the given model. */
template <typename T>
void AddCouplerConstraint(IcfModel<T>* model);

/* Adds some dummy gain constraints to the model.

@returns The (K, u, e) vectors used to set the gain constraints on two cliques.
*/
template <typename T>
std::vector<VectorX<T>> AddGainConstraints(IcfModel<T>* model);

/* Adds limit constraints to the given model. The model can have 1 or 3 cliques.
In either case, the same constraints are added to represent the same logical
limits. */
template <typename T>
void AddLimitConstraints(IcfModel<T>* model);

/* Adds some dummy contact constraints to the model. */
template <typename T>
void AddPatchConstraints(IcfModel<T>* model);

/* Adds weld constraints to the given model. Two weld constraints are added:
one between the world (body 0) and body 1, and one between body 2 and body 3
(cross-clique in the multi-clique case). */
template <typename T>
void AddWeldConstraints(IcfModel<T>* model);

/* Adds ball constraints to the given model. Two ball constraints are added:
one between the world (body 0) and body 1, and one between body 2 and body 3
(cross-clique in the multi-clique case). */
template <typename T>
void AddBallConstraints(IcfModel<T>* model);

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
