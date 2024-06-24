#include "drake/multibody/tree/force_element.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <typename T>
ForceElement<T>::~ForceElement() = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::ForceElement);
