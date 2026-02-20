#pragma once

#include <cstdint>
#include <memory>

#include "drake/planning/collision_checker.h"
#include "drake/planning/sampling_based/dev/collision_checker_planning_space.h"
#include "drake/planning/sampling_based/dev/default_state_types.h"

namespace drake {
namespace planning {
/// Base class for implementations of asymmetric planning spaces with a
/// CollisionChecker and JointLimits.
template <typename StateType>
class AsymmetricCollisionCheckerPlanningSpace
    : public CollisionCheckerPlanningSpace<StateType> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  AsymmetricCollisionCheckerPlanningSpace(
      AsymmetricCollisionCheckerPlanningSpace<StateType>&&) = delete;
  AsymmetricCollisionCheckerPlanningSpace& operator=(
      const AsymmetricCollisionCheckerPlanningSpace<StateType>&) = delete;
  AsymmetricCollisionCheckerPlanningSpace& operator=(
      AsymmetricCollisionCheckerPlanningSpace<StateType>&&) = delete;

  ~AsymmetricCollisionCheckerPlanningSpace() override;

 protected:
  // Copy constructor for use in Clone().
  AsymmetricCollisionCheckerPlanningSpace(
      const AsymmetricCollisionCheckerPlanningSpace<StateType>& other);

  /// Constructor.
  /// @param collision_checker Collision checker to use.
  /// @param seed Seed for per-thread random source.
  AsymmetricCollisionCheckerPlanningSpace(
      std::unique_ptr<CollisionChecker> collision_checker,
      const JointLimits& joint_limits, uint64_t seed);
};

}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::AsymmetricCollisionCheckerPlanningSpace)
