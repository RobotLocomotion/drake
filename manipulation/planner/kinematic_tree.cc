#include "drake/manipulation/planner/kinematic_tree.h"

namespace drake {
namespace manipulation {
namespace planner {

bool KinematicTree::SatisfiesJointPositionLimits(VectorX<double> q) const {
  DRAKE_THROW_UNLESS(q.size() == num_positions());
  Eigen::VectorXd eps = Eigen::VectorXd::Constant(num_positions(), 1e-6);
  return ((joint_position_lower_limit() - eps).array() <= q.array()).all() &&
         ((joint_position_upper_limit() + eps).array() >= q.array()).all();
}

void KinematicTree::SetJointPositionLimits(const VectorX<double>& lower_limit,
                                           const VectorX<double>& upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit.size() == num_positions());
  DRAKE_THROW_UNLESS(upper_limit.size() == num_positions());
  for (int i = 0; i < num_positions(); ++i) {
    SetJointPositionLimits(i, lower_limit(i), upper_limit(i));
  }
}

void KinematicTree::SetJointPositionLimits(int position_index,
                                           double lower_limit,
                                           double upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit <= upper_limit);
  DRAKE_THROW_UNLESS(0 <= position_index && position_index < num_positions());
  DoSetJointPositionLimits(position_index, lower_limit, upper_limit);
}

void KinematicTree::SetJointVelocityLimits(const VectorX<double>& lower_limit,
                                           const VectorX<double>& upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit.size() == num_velocities());
  DRAKE_THROW_UNLESS(upper_limit.size() == num_velocities());
  for (int i = 0; i < num_velocities(); ++i) {
    SetJointVelocityLimits(i, lower_limit(i), upper_limit(i));
  }
}

void KinematicTree::SetJointVelocityLimits(int velocity_index,
                                           double lower_limit,
                                           double upper_limit) {
  DRAKE_THROW_UNLESS(lower_limit <= upper_limit);
  DRAKE_THROW_UNLESS(0 <= velocity_index && velocity_index < num_velocities());
  DoSetJointVelocityLimits(velocity_index, lower_limit, upper_limit);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
