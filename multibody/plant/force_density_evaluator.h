#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/force_density_field.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* Wrapper around ForceDensityField and a pointer to the owning
 MultibodyPlant's context to help evaluate the force field with the plant
 context. The evaluator should not be persisted. It is more advisable to acquire
 it for evaluation in a limited scope and then discard it. Constructing an
 evaluator is cheap. */
template <typename T>
class ForceDensityEvaluator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ForceDensityEvaluator)

  /* Constructs an evaluator with the given `force_field` and the context of the
   MultibodyPlant owning the force field.
   @pre input pointers are non-null.
   @pre `plant_context` is the context of the MultibodyPlant that owns
   `force_field`. */
  ForceDensityEvaluator(const ForceDensityField<T>* force_field,
                        const systems::Context<T>* plant_context);

  /* Evaluates the force field (with unit [N/m³]) at the given point Q in world
   frame.*/
  Vector3<T> EvaluateAt(const Vector3<T>& p_WQ) const {
    return force_field_->EvaluateAt(*plant_context_, p_WQ);
  }

  template <typename U>
  friend bool operator==(const ForceDensityEvaluator<U>& lhs,
                         const ForceDensityEvaluator<U>& rhs);

 private:
  const ForceDensityField<T>* force_field_{nullptr};
  const systems::Context<T>* plant_context_{nullptr};
};

/* Compares two evaluators for equality. Used for unit tests. */
template <typename T>
bool operator==(const ForceDensityEvaluator<T>& lhs,
                const ForceDensityEvaluator<T>& rhs) {
  return lhs.force_field_ == rhs.force_field_ &&
         lhs.plant_context_ == rhs.plant_context_;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ForceDensityEvaluator)
