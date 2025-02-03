#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/planning/collision_checker.h"
#include "planning/default_state_types.h"
#include "planning/joint_limits.h"
#include "planning/parallelism.h"
#include "planning/per_instance_qs.h"
#include "planning/planning_space.h"

namespace anzu {
namespace planning {
// Forward declarations.
template<typename StateType>
class AsymmetricCollisionCheckerPlanningSpace;
template<typename StateType>
class SymmetricCollisionCheckerPlanningSpace;

/// Base type for PlanningSpaces that have a CollisionChecker and a JointLimits.
/// Note: users cannot inherit directly from CollisionCheckerPlanningSpace, and
/// must instead inherit from AsymmetricCollisionCheckerPlanningSpace or
/// SymmetricCollisionCheckerPlanningSpace depending on whether or not their
/// space is symmetric (i.e. each pair of *Forward* and *Backwards* methods is
/// provided by a single implementation.)
template<typename StateType>
class CollisionCheckerPlanningSpace : public PlanningSpace<StateType> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  CollisionCheckerPlanningSpace(
      CollisionCheckerPlanningSpace<StateType>&&) = delete;
  CollisionCheckerPlanningSpace& operator=(
      const CollisionCheckerPlanningSpace<StateType>&) = delete;
  CollisionCheckerPlanningSpace& operator=(
      CollisionCheckerPlanningSpace<StateType>&&) = delete;

  ~CollisionCheckerPlanningSpace() override;

  const drake::planning::CollisionChecker& collision_checker() const {
    return *collision_checker_;
  }

  drake::planning::CollisionChecker& mutable_collision_checker() {
    return *collision_checker_;
  }

  const JointLimits& nominal_joint_limits() const {
    return nominal_joint_limits_;
  }

  const JointLimits& joint_limits() const { return joint_limits_; }

  void ResetJointLimitsToNominal() { SetJointLimits(nominal_joint_limits()); }

  /// Sets new joint limits for sampling.
  /// @param joint_limits New joint limits. @pre size of position limits must
  /// match the configuration size of the collision checker.
  void SetJointLimits(const JointLimits& joint_limits) {
    DRAKE_THROW_UNLESS(joint_limits.num_positions() ==
                       collision_checker().GetZeroConfiguration().size());
    joint_limits_ = joint_limits;
  }

  JointLimits MakeJointLimits(
      const JointLimits& joint_limits, const PerInstanceQs& fixed_qs) const;

  Eigen::VectorXd MakeCombinedQ(
      const PerInstanceQs& active_qs, const PerInstanceQs& passive_qs) const;

  /// Provided for convenience where replacing MultibodyPlantWrapper.
  const drake::multibody::MultibodyPlant<double>& plant() const {
    return collision_checker().plant();
  }

  /// Compute relative transform X_AB for the provided positions. Provided for
  /// convenience where replacing MultibodyPlantWrapper.
  /// @param q Plant positions.
  /// @param frame_A Frame A.
  /// @param frame_B Frame B.
  /// @param thread_number Optional thread number.
  drake::math::RigidTransformd CalcRelativeTransform(
      const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
      const drake::multibody::Frame<double>& frame_B,
      std::optional<int> thread_number = std::nullopt) const {
    const auto& plant_context = collision_checker().UpdatePositions(
        q, PlanningSpace<StateType>::ResolveThreadNumber(thread_number));
    return plant().CalcRelativeTransform(plant_context, frame_A, frame_B);
  }

 protected:
  /// Copy constructor for use in Clone().
  CollisionCheckerPlanningSpace(
      const CollisionCheckerPlanningSpace<StateType>& other);

 private:
  friend class AsymmetricCollisionCheckerPlanningSpace<StateType>;
  friend class SymmetricCollisionCheckerPlanningSpace<StateType>;

  // Constructor. This is private so that only
  // AsymmetricCollisionCheckerPlanningSpace and
  // SymmetricCollisionCheckerPlanningSpace (declared as friends above) can
  // directly construct a CollisionCheckerPlanningSpace.
  // @param collision_checker Collision checker to use.
  // @param joint_limits Joint limits to use.
  // @param seed Seed for per-thread random source.
  // @param is_symmetric Is the planning space symmetric?
  CollisionCheckerPlanningSpace(
      std::unique_ptr<drake::planning::CollisionChecker> collision_checker,
      const JointLimits& joint_limits, uint64_t seed, bool is_symmetric);

  drake::copyable_unique_ptr<drake::planning::CollisionChecker>
      collision_checker_;
  JointLimits joint_limits_;
  JointLimits nominal_joint_limits_;
};

}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::CollisionCheckerPlanningSpace)
