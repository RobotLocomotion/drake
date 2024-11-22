#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_element.h"

namespace drake {
namespace multibody {

// This second (dummy) template argument is required so that
// DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS
// isn't a no-op.
template <typename T>
template <typename>
const MultibodyPlant<T>& MultibodyElement<T>::GetParentPlant() const {
  HasParentTreeOrThrow();

  const auto plant =
      dynamic_cast<const MultibodyPlant<T>*>(&get_parent_tree().tree_system());

  if (plant == nullptr) {
    throw std::logic_error(
        "This multibody element was not owned by a MultibodyPlant.");
  }

  return *plant;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &MultibodyElement<T>::template GetParentPlant<void>
));
// clang-format on

}  // namespace multibody
}  // namespace drake
