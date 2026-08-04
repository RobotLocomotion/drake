#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/force_density_field.h"

namespace drake {
namespace multibody {

// This second (dummy) template argument is required so that
// DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS
// isn't a no-op.
template <typename T>
template <typename>
void ForceDensityField<T>::DeclareSystemResources(
    internal::MultibodyTreeSystem<T>* tree_system) {
  DRAKE_DEMAND(tree_system != nullptr);
  tree_system_ = tree_system;
  auto plant = dynamic_cast<MultibodyPlant<T>*>(tree_system);
  if (plant == nullptr) {
    throw std::logic_error(
        "The given MultibodyTreeSystem does not belong to a MultibodyPlant.");
  }
  /* Only cache entries and input ports are supported for now. More system
   resources (e.g. parameters) can be declared if needed in the future. */
  DeclareCacheEntries(plant);
  DeclareInputPorts(plant);
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &ForceDensityField<T>::template DeclareSystemResources<void>
));
// clang-format on

}  // namespace multibody
}  // namespace drake
