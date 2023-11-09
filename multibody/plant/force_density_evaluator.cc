#include "drake/multibody/plant/force_density_evaluator.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
ForceDensityEvaluator<T>::ForceDensityEvaluator(
    const ForceDensityField<T>* force_field,
    const systems::Context<T>* plant_context)
    : force_field_(force_field), plant_context_(plant_context) {
  DRAKE_DEMAND(force_field_ != nullptr);
  DRAKE_DEMAND(plant_context_ != nullptr);
  if (force_field->has_parent_system()) {
    force_field_->parent_system_or_throw().ValidateContext(*plant_context);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ForceDensityEvaluator)
