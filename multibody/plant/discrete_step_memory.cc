#include "drake/multibody/plant/discrete_step_memory.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DiscreteStepMemory::Data<T>::Data(const internal::SpanningForest& forest)
    : acceleration_kinematics_cache(forest) {}

template <typename T>
DiscreteStepMemory::Data<T>::~Data() = default;

template <typename T>
DiscreteStepMemory::Data<T>& DiscreteStepMemory::Allocate(
    const internal::SpanningForest& forest) {
  auto new_data = std::make_shared<Data<T>>(forest);
  Data<T>* result = new_data.get();
  data = std::move(new_data);
  return *result;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &DiscreteStepMemory::Allocate<T>
));
// clang-format on

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::DiscreteStepMemory::Data);
