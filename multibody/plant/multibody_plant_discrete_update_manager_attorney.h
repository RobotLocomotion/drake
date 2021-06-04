#pragma once
#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class DiscreteUpdateManager;

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to DiscreteUpdateManager.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantDiscreteUpdateManagerAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantDiscreteUpdateManagerAttorney);

  friend class DiscreteUpdateManager<T>;

  static const MultibodyTree<T>& internal_tree(const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static systems::CacheEntry& DeclareCacheEntry(
      MultibodyPlant<T>* plant, std::string description,
      systems::CacheEntry::AllocCallback alloc_function,
      systems::CacheEntry::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::SystemBase::all_sources_ticket()}) {
    return plant->DeclareCacheEntry(
        std::move(description), std::move(alloc_function),
        std::move(calc_function), std::move(prerequisites_of_calc));
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
