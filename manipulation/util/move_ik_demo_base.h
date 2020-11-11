#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace util {

/// This class provides some common functionality for generating IK plans for
/// robot arms, including things like creating a MultibodyPlant, setting joint
/// velocity limits, implementing a robot status update handler suitable for
/// invoking from an LCM callback, and generating plans to move a specified
/// link to a goal configuration.
///
/// This can be useful when building simple demonstration programs to move a
/// robot arm, for example when testing new arms which haven't been previously
/// used with Drake, or testing modifications to existing robot
/// configurations.  See the kuka_iiwa_arm and kinova_jaco_arm examples for
/// existing uses.
class MoveIkDemoBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MoveIkDemoBase);

  /// @param robot_description A description file to load of the robot to
  /// plan.
  ///
  /// @param base_link Name of the base link of the robot, will be welded to
  /// the world in the planning model.
  ///
  /// @param ik_link Name of the link to plan a pose for.
  ///
  /// @param print_interval Print an updated end effector position every N
  /// calls to HandleStatus.
  MoveIkDemoBase(std::string robot_description,
                 std::string base_link,
                 std::string ik_link,
                 int print_interval);

  ~MoveIkDemoBase();

  /// @return a reference to the internal plant.
  const multibody::MultibodyPlant<double>& plant() const { return plant_; }

  /// Set the joint velocity limts when building the plan.  The default
  /// velocity limits from the robot description will be used if this isn't
  /// set.
  ///
  /// @pre The size of the input vector must be equal to the number of
  /// velocities in the MultibodyPlant (see plant()).
  void set_joint_velocity_limits(const Eigen::Ref<const Eigen::VectorXd>&);

  /// Update the current robot status.
  ///
  /// @param q must be equal to the number of positions in the MultibodyPlant
  /// (see plant()).
  void HandleStatus(const Eigen::Ref<const Eigen::VectorXd>& q);

  /// Attempt to generate a plan moving ik_link (specified at construction
  /// time) from the joint configuration specified in the last call to
  /// `HandleStatus` to a configuration with ik_link at @p goal_pose.  Returns
  /// nullopt if planning failed.
  ///
  /// @throw If HandleStatus has not been invoked.
  std::optional<lcmt_robot_plan> Plan(
      const math::RigidTransformd& goal_pose);

  /// Returns a count of how many times `HandleStatus` has been called.
  int status_count() const { return status_count_; }

 private:
  std::string robot_description_;
  std::string ik_link_;
  int print_interval_{};
  multibody::MultibodyPlant<double> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd joint_velocity_limits_;
  int status_count_{0};
  planner::ConstraintRelaxingIk constraint_relaxing_ik_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
