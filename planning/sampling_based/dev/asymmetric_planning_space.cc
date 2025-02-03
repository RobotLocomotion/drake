#include "planning/asymmetric_planning_space.h"

namespace anzu {
namespace planning {

template<typename StateType>
AsymmetricPlanningSpace<StateType>::~AsymmetricPlanningSpace() = default;

template<typename StateType>
AsymmetricPlanningSpace<StateType>::AsymmetricPlanningSpace(
    const AsymmetricPlanningSpace<StateType>& other) = default;

template<typename StateType>
AsymmetricPlanningSpace<StateType>::AsymmetricPlanningSpace(
    const uint64_t seed, const Parallelism parallelism)
    : PlanningSpace<StateType>(seed, parallelism, false) {}

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::AsymmetricPlanningSpace)
