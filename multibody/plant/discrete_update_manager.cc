#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
const MultibodyTree<T>& DiscreteUpdateManager<T>::internal_tree() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::internal_tree(plant());
}

template <typename T>
systems::CacheEntry& DiscreteUpdateManager<T>::DeclareCacheEntry(
    MultibodyPlant<T>* plant, std::string description,
    systems::CacheEntry::AllocCallback alloc_function,
    systems::CacheEntry::CalcCallback calc_function,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::DeclareCacheEntry(
      plant, std::move(description), std::move(alloc_function),
      std::move(calc_function), std::move(prerequisites_of_calc));
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
