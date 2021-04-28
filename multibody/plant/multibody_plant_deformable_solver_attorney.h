#pragma once
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
namespace drake {
namespace multibody {
template <typename T>
class MultibodyPlant;

namespace internal {
template <typename T>
class DeformableSolverBase;

/* This is an attorney-client pattern providing DeformableSolverBase access to
 the private data/methods of MultibodyPlant in order to be able construct
 contact solver data. This class is meant to be a short-term solution to
 quickly facilitate integration of Softsim with MultibodyPlant without moving
 Softsim out of the dev/ directory. */
template <typename T>
class MultibodyPlantDeformableSolverAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantDeformableSolverAttorney);
  MultibodyPlantDeformableSolverAttorney() = delete;

 private:
  friend class DeformableSolverBase<T>;

  /* Declares a deformable state with the given position, velocity and
   acceleration in the given `mbp`. */
  static void DeclareDeformableState(const VectorX<T>& q,
                                     const VectorX<T>& qdot,
                                     const VectorX<T>& qddot,
                                     MultibodyPlant<T>* mbp);
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::MultibodyPlantDeformableSolverAttorney);
