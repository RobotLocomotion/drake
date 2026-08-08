#include "drake/planning/sampling_based/dev/asymmetric_collision_checker_planning_space.h"

#include <utility>

namespace drake {
namespace planning {

template <typename StateType>
AsymmetricCollisionCheckerPlanningSpace<
    StateType>::~AsymmetricCollisionCheckerPlanningSpace() = default;

template <typename StateType>
AsymmetricCollisionCheckerPlanningSpace<StateType>::
    AsymmetricCollisionCheckerPlanningSpace(
        const AsymmetricCollisionCheckerPlanningSpace<StateType>& other) =
        default;

template <typename StateType>
AsymmetricCollisionCheckerPlanningSpace<StateType>::
    AsymmetricCollisionCheckerPlanningSpace(
        std::unique_ptr<CollisionChecker> collision_checker,
        const JointLimits& joint_limits, const uint64_t seed)
    : CollisionCheckerPlanningSpace<StateType>(std::move(collision_checker),
                                               joint_limits, seed, false) {}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::AsymmetricCollisionCheckerPlanningSpace)
