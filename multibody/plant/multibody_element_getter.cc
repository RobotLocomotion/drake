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
  if (parent_tree_ != nullptr) {
    const internal::MultibodyTreeSystem<T>& system =
        parent_tree_->tree_system();
    const auto* plant = dynamic_cast<const MultibodyPlant<T>*>(&system);
    if (plant != nullptr) {
      return *plant;
    }
  }
  throw std::logic_error(
      "This multibody element was not owned by a MultibodyPlant.");
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &MultibodyElement<T>::template GetParentPlant<void>
));
// clang-format on

}  // namespace multibody
}  // namespace drake
