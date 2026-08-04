#include "drake/multibody/plant/desired_state_input.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DesiredStateInput<T>::DesiredStateInput() = default;

template <typename T>
DesiredStateInput<T>::~DesiredStateInput() = default;

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::DesiredStateInput);
