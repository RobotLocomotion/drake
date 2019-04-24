#include <vector>

#include "drake/manipulation/robot_plan_runner/robot_plans.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

void JointSpacePlan::UpdatePlan(
    std::unique_ptr<trajectories::PiecewisePolynomial<double>> q_traj_new) {
  DRAKE_ASSERT(q_traj_new->rows() == num_positions_);
  q_traj_ = std::move(q_traj_new);
};

void JointSpacePlan::Step(const Eigen::Ref<const Eigen::VectorXd>& q,
          const Eigen::Ref<const Eigen::VectorXd>& v,
          const Eigen::Ref<const Eigen::VectorXd>& tau_external, double t,
          Eigen::VectorXd* const q_cmd,
          Eigen::VectorXd* const tau_cmd) const {
  if (q_traj_) {
    // if q_traj_ is not NULL.
    *q_cmd = q_traj_->value(t);
    *tau_cmd = Eigen::VectorXd::Zero(num_positions_);
  }
}



}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
