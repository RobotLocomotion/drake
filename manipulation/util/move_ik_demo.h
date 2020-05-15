#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <robotlocomotion/robot_plan_t.hpp>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace util {

/// A utility class to create simple demonstration programs to move a robot
/// arm.  Creates a plan for a single-shot move of a specified link.  Such an
/// application can be helpful for testing new arms which haven't been
/// previously used with drake, or testing modifications to existing
/// robot configurations.
class MoveIkDemo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MoveIkDemo);

  /// @param robot_description A description file to load for the robot to
  /// move
  ///
  /// @param base_link Name of the base link of the robot, will be welded to
  /// the world in the planning model
  ///
  /// @param ik_link Name of the link to plan a pose for
  ///
  /// @param ik_pose Pose to plan to move ik_link to
  ///
  /// @param print_interval Print an updated end effector position every N
  /// calls to HandleStatus.
  MoveIkDemo(const std::string& robot_description,
             const std::string& base_link,
             const std::string& ik_link,
             const math::RigidTransformd& ik_pose,
             int print_interval);

  ~MoveIkDemo();

  /// Set the joint velocity limts when building the plan.  The default
  /// velocity limits from the robot description will be used if this isn't
  /// set.
  void set_joint_velocity_limits(const Eigen::Ref<const Eigen::VectorXd>&);

  /// Update the current robot status.  If an updated plan is available, it
  /// will be returned here.
  std::optional<robotlocomotion::robot_plan_t> HandleStatus(
      const Eigen::Ref<const Eigen::VectorXd>& q);

 private:
  std::string robot_description_;
  std::string ik_link_;
  math::RigidTransformd ik_pose_;
  int print_interval_{};
  multibody::MultibodyPlant<double> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd joint_velocity_limits_;
  int status_count_{0};
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
