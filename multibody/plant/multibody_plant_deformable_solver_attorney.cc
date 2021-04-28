#include "drake/multibody/plant/multibody_plant_deformable_solver_attorney.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void MultibodyPlantDeformableSolverAttorney<T>::DeclareDeformableState(
    const VectorX<T>& q, const VectorX<T>& qdot, const VectorX<T>& qddot,
    MultibodyPlant<T>* mbp) {
  mbp->DeclareDeformableState(q, qdot, qddot);
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::MultibodyPlantDeformableSolverAttorney);
