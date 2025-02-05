#include "drake/planning/sampling_based/dev/goal_sampler.h"

namespace drake {
namespace planning {

template <typename StateType>
GoalSampler<StateType>::~GoalSampler() = default;

template <typename StateType>
GoalSampler<StateType>::GoalSampler() = default;

template <typename StateType>
GoalSampler<StateType>::GoalSampler(const GoalSampler& other) = default;

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::GoalSampler)
