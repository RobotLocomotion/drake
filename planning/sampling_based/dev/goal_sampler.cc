#include "planning/goal_sampler.h"

namespace anzu {
namespace planning {

template <typename StateType>
GoalSampler<StateType>::~GoalSampler() = default;

template <typename StateType>
GoalSampler<StateType>::GoalSampler() = default;

template <typename StateType>
GoalSampler<StateType>::GoalSampler(const GoalSampler& other) = default;

}  // namespace planning
}  // namespace anzu

ANZU_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::GoalSampler)
