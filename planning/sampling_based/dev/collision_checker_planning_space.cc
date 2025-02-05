#include "drake/planning/sampling_based/dev/collision_checker_planning_space.h"

#include <map>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace planning {
namespace {
Parallelism GetCollisionCheckerParallelism(
    const drake::planning::CollisionChecker* collision_checker) {
  DRAKE_THROW_UNLESS(collision_checker != nullptr);
  return Parallelism(collision_checker->num_allocated_contexts());
}
}  // namespace

template <typename StateType>
CollisionCheckerPlanningSpace<StateType>::~CollisionCheckerPlanningSpace() =
    default;

template <typename StateType>
JointLimits CollisionCheckerPlanningSpace<StateType>::MakeJointLimits(
    const JointLimits& joint_limits, const PerInstanceQs& fixed_qs) const {
  DRAKE_THROW_UNLESS(joint_limits.num_positions() ==
                     collision_checker().GetZeroConfiguration().size());

  Eigen::VectorXd position_lower = joint_limits.position_lower();
  Eigen::VectorXd position_upper = joint_limits.position_upper();

  Eigen::VectorXd velocity_lower = joint_limits.velocity_lower();
  Eigen::VectorXd velocity_upper = joint_limits.velocity_upper();

  Eigen::VectorXd acceleration_lower = joint_limits.acceleration_lower();
  Eigen::VectorXd acceleration_upper = joint_limits.acceleration_upper();

  for (const auto& [instance, instance_q] : fixed_qs) {
    DRAKE_THROW_UNLESS(plant().num_positions(instance) == instance_q.size());
    plant().SetPositionsInArray(instance, instance_q, &position_lower);
    plant().SetPositionsInArray(instance, instance_q, &position_upper);

    const auto instance_zeros =
        Eigen::VectorXd::Zero(plant().num_velocities(instance));
    plant().SetPositionsInArray(instance, instance_zeros, &velocity_lower);
    plant().SetPositionsInArray(instance, instance_zeros, &velocity_upper);

    plant().SetPositionsInArray(instance, instance_zeros, &acceleration_lower);
    plant().SetPositionsInArray(instance, instance_zeros, &acceleration_upper);
  }

  if (!joint_limits.CheckInPositionLimits(position_lower)) {
    drake::log()->warn(
        "MakeJointLimits fixed joint positions exceed existing joint limits:\n"
        "Existing lower limits: {}\nNew lower limits: {}\n"
        "Existing upper limits: {}\nNew upper limits: {}",
        drake::fmt_eigen(joint_limits.position_lower()),
        drake::fmt_eigen(position_lower),
        drake::fmt_eigen(joint_limits.position_upper()),
        drake::fmt_eigen(position_upper));
  }

  return JointLimits(position_lower, position_upper, velocity_lower,
                     velocity_upper, acceleration_lower, acceleration_upper);
}

template <typename StateType>
Eigen::VectorXd CollisionCheckerPlanningSpace<StateType>::MakeCombinedQ(
    const PerInstanceQs& active_qs, const PerInstanceQs& passive_qs) const {
  // Track what we've written.
  std::map<drake::multibody::ModelInstanceIndex, int> instances_written;
  int size = 0;

  Eigen::VectorXd combined_q = collision_checker().GetZeroConfiguration();

  for (const auto& [instance, instance_q] : active_qs) {
    instances_written[instance] += 1;
    size += instance_q.size();
    plant().SetPositionsInArray(instance, instance_q, &combined_q);
  }

  for (const auto& [instance, instance_q] : passive_qs) {
    instances_written[instance] += 1;
    size += instance_q.size();
    plant().SetPositionsInArray(instance, instance_q, &combined_q);
  }

  // Make sure we have only written each instance once.
  for (const auto& [instance, write_count] : instances_written) {
    drake::unused(instance);
    DRAKE_THROW_UNLESS(write_count == 1);
  }

  // Make sure we have written a full configuration.
  DRAKE_THROW_UNLESS(size == combined_q.size());

  return combined_q;
}

template <typename StateType>
CollisionCheckerPlanningSpace<StateType>::CollisionCheckerPlanningSpace(
    const CollisionCheckerPlanningSpace<StateType>& other) = default;

template <typename StateType>
CollisionCheckerPlanningSpace<StateType>::CollisionCheckerPlanningSpace(
    std::unique_ptr<CollisionChecker> collision_checker,
    const JointLimits& joint_limits, const uint64_t seed,
    const bool is_symmetric)
    : PlanningSpace<StateType>(
          seed, GetCollisionCheckerParallelism(collision_checker.get()),
          is_symmetric),
      collision_checker_(std::move(collision_checker)) {
  DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
  DRAKE_THROW_UNLESS(collision_checker_->num_allocated_contexts() ==
                     this->random_source().num_generators());
  // Note that this order means sanity checking is performed by SetJointLimits.
  SetJointLimits(joint_limits);
  nominal_joint_limits_ = joint_limits;
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::CollisionCheckerPlanningSpace)
