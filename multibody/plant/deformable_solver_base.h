#pragma once
#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
template <typename T>
class MultibodyPlant;

namespace internal {
/* DeformableSolverBase is a pure virtual deformable interface class that serves
 as the one-stop shop for all deformable-related functionalities for the owning
 MbP to advance its forward dynamics. It serves as a temporary solution for MbP
 to access the deformable functionalities that live in dev now (because MbP,
 being out of dev, cannot depend on dev functionalities). This class should be
 retired when the deformable functionalities come out of time. When that
 happens, MbP should be able to directly depend on these deformable
 functionalities. */
template <typename T>
class DeformableSolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableSolverBase)

  /* Creates a DeformableSolverBase that is owned by the given `mbp`. */
  explicit DeformableSolverBase(multibody::MultibodyPlant<T>* mbp) : mbp_(mbp) {
    DRAKE_DEMAND(mbp_ != nullptr);
  }

  virtual ~DeformableSolverBase() = default;

  /* Calculates the state of the deformable body indexed with
   `deformable_body_index` in the absense of contact force over a single time
   step.
   @param state0                 The deformable body's state at the beginning of
                                 the time step.
   @param deformable_body_index  The index of the deformable body. */
  virtual VectorX<T> CalcFreeMotion(const VectorX<T>& state0,
                                    int deformable_body_index) const = 0;

 protected:
  /* Returns the gravity vector of the owning MbP. */
  const Vector3<double>& gravity() const;

  /* Returns the discrete time step of the owning MbP. */
  double dt() const;

  /* Declares a discrete deformable state in the owning MbP with the given `q`,
   `qdot` and `qddot` as model values. */
  void DeclareDeformableState(const VectorX<T>& q, const VectorX<T>& qdot,
                              const VectorX<T>& qddot);

 private:
  // Back pointer to owning MbP.
  MultibodyPlant<T>* mbp_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::DeformableSolverBase);
