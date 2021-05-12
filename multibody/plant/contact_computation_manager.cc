#include "drake/multibody/plant/contact_computation_manager.h"

#include "drake/multibody/plant/multibody_plant_contact_attorney.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
/* Exposed MultibodyPlant private/protected methods. */
const MultibodyTree<T>& ContactComputationManager<T>::internal_tree() const {
  return MultibodyPlantContactAttorney<T>::internal_tree(plant());
}

template <typename T>
const contact_solvers::internal::ContactSolverResults<T>&
ContactComputationManager<T>::EvalContactSolverResults(
    const systems::Context<T>& context) const {
  return MultibodyPlantContactAttorney<T>::EvalContactSolverResults(plant(),
                                                                    context);
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ContactComputationManager);
