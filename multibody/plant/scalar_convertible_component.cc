#include "drake/multibody/plant/scalar_convertible_component.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
ScalarConvertibleComponent<T>::~ScalarConvertibleComponent() = default;

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScalarConvertibleComponent);
