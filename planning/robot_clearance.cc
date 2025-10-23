#include "drake/planning/robot_clearance.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace planning {

using multibody::BodyIndex;

RobotClearance::~RobotClearance() = default;

void RobotClearance::Reserve(int size) {
  robot_indices_.reserve(size);
  other_indices_.reserve(size);
  collision_types_.reserve(size);
  distances_.reserve(size);
  jacobians_.reserve(size * nq_);
}

void RobotClearance::Append(
    BodyIndex robot_index, BodyIndex other_index,
    RobotCollisionType collision_type, double distance,
    const Eigen::Ref<const Eigen::RowVectorXd>& jacobian) {
  DRAKE_THROW_UNLESS(jacobian.cols() == nq_);
  robot_indices_.push_back(robot_index);
  other_indices_.push_back(other_index);
  collision_types_.push_back(collision_type);
  distances_.push_back(distance);
  for (int j = 0; j < nq_; ++j) {
    jacobians_.push_back(jacobian[j]);
  }
}

}  // namespace planning
}  // namespace drake
