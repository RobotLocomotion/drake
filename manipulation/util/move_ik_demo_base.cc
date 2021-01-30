#include "drake/manipulation/util/move_ik_demo_base.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/manipulation/util/robot_plan_utils.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace util {

using planner::ConstraintRelaxingIk;

MoveIkDemoBase::MoveIkDemoBase(std::string robot_description,
                               std::string base_link,
                               std::string ik_link,
                               int print_interval)
    : robot_description_(std::move(robot_description)),
      ik_link_(std::move(ik_link)),
      print_interval_(print_interval),
      plant_(0.0),
      constraint_relaxing_ik_(robot_description_, ik_link_) {
  multibody::Parser(&plant_).AddModelFromFile(robot_description_);
  plant_.WeldFrames(plant_.world_frame(),
                    plant_.GetBodyByName(base_link).body_frame());
  plant_.Finalize();
  context_ = plant_.CreateDefaultContext();
  joint_names_ = GetJointNames(plant_);
  joint_velocity_limits_ = plant_.GetVelocityUpperLimits();
}

MoveIkDemoBase::~MoveIkDemoBase() {}

void MoveIkDemoBase::set_joint_velocity_limits(
    const Eigen::Ref<const Eigen::VectorXd>& velocity_limits) {
  DRAKE_THROW_UNLESS(velocity_limits.size() ==
                     joint_velocity_limits_.size());
  joint_velocity_limits_ = velocity_limits;
}

void MoveIkDemoBase::HandleStatus(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  status_count_++;
  plant_.SetPositions(context_.get(), q);

  if (status_count_ % print_interval_ == 1) {
    const math::RigidTransform<double> current_link_pose =
        plant_.EvalBodyPoseInWorld(
            *context_, plant_.GetBodyByName(ik_link_));
    const math::RollPitchYaw<double> rpy(current_link_pose.rotation());
    drake::log()->info("{} at: {} {}",
                       ik_link_,
                       current_link_pose.translation().transpose(),
                       rpy.vector().transpose());
  }
}

std::optional<lcmt_robot_plan> MoveIkDemoBase::Plan(
    const math::RigidTransformd& goal_pose) {

  DRAKE_THROW_UNLESS(status_count_ > 0);

  // Create a single waypoint for our plan (the destination).
  // This results in a trajectory with two knot points (the
  // current pose (read from the status message currently being
  // processes and passed directly to PlanSequentialTrajectory as
  // iiwa_q) and the calculated final pose).
  ConstraintRelaxingIk::IkCartesianWaypoint wp;
  wp.pose = goal_pose;
  wp.constrain_orientation = true;
  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
  waypoints.push_back(wp);
  std::vector<Eigen::VectorXd> q_sol;
  const bool result =
      constraint_relaxing_ik_.PlanSequentialTrajectory(
          waypoints, plant_.GetPositions(*context_), &q_sol);
  drake::log()->info("IK result: {}", result);

  if (result) {
    drake::log()->info("IK sol size {}", q_sol.size());

    // Run the resulting plan over 2 seconds (which is a fairly
    // arbitrary choice).  This may be slowed down if executing
    // the plan in that time would exceed any joint velocity
    // limits.
    std::vector<double> times{0, 2};
    DRAKE_DEMAND(q_sol.size() == times.size());

    ApplyJointVelocityLimits(
        q_sol, joint_velocity_limits_, &times);
    lcmt_robot_plan plan = EncodeKeyFrames(
        joint_names_, times, q_sol);
    return plan;
  }

  return std::nullopt;
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
