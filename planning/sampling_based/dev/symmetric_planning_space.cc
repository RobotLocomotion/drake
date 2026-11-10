#include "drake/planning/sampling_based/dev/symmetric_planning_space.h"

namespace drake {
namespace planning {

template <typename StateType>
SymmetricPlanningSpace<StateType>::~SymmetricPlanningSpace() = default;

template <typename StateType>
SymmetricPlanningSpace<StateType>::SymmetricPlanningSpace(
    const SymmetricPlanningSpace<StateType>& other) = default;

template <typename StateType>
SymmetricPlanningSpace<StateType>::SymmetricPlanningSpace(
    const uint64_t seed, const Parallelism parallelism)
    : PlanningSpace<StateType>(seed, parallelism, true) {}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::SymmetricPlanningSpace)
