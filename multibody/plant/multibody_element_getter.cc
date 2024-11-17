#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_element.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename MultibodyPlantDeferred>
const MultibodyPlantDeferred& MultibodyElement<T>::GetParentPlant() const {
  HasParentTreeOrThrow();

  const auto plant = dynamic_cast<const MultibodyPlantDeferred*>(
      &get_parent_tree().tree_system());

  if (plant == nullptr) {
    throw std::logic_error(
        "This multibody element was not owned by a MultibodyPlant.");
  }

  return *plant;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &MultibodyElement<T>::template GetParentPlant<MultibodyPlant<T>>
));
// clang-format on

}  // namespace multibody
}  // namespace drake
